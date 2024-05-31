#include "effibote3_hardware/effibote3_hardware.hpp"
#include "effibote3_hardware/effibote3_display.hpp"
#include <romea_mobile_base_hardware/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>


#define SLEEP_TIME_RECEIVING_NO_MESSAGE 100000 // in micro seconds, must be lower than 100000 micro seconds.
                                               // = Time to sleep at the end of the loop in the main function, if the receiving of messages
                                               // from the robot is deactivated.


namespace romea
{
namespace ros2
{


//-----------------------------------------------------------------------------
EffibotE3Hardware::EffibotE3Hardware()
: HardwareSystemInterface4WD(),
  serial_listening_thread_(nullptr),
  serial_listening_thread_run_(false),
  logger_(rclcpp::get_logger("effibote3_hardware"))
{

  try {
    serial_port_.setPort(
      "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0");
    serial_port_.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // Timeout = 1s
    serial_port_.setTimeout(to);
    serial_port_.open();
  } catch (serial::IOException & e) {
    RCLCPP_ERROR_STREAM(logger_, "Unable to open port ");
    std::cerr << e.what() << std::endl;
  }

// #ifndef NDEBUG
//   open_log_file_();
//   write_log_header_();
// #endif

}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::load_info_(
  const hardware_interface::HardwareInfo & hardware_info)
{

  RCLCPP_ERROR_STREAM(logger_, "load_info");

  try {
    front_wheel_radius_ = get_parameter<float>(hardware_info, "front_wheel_radius");
    rear_wheel_radius_ = get_parameter<float>(hardware_info, "rear_wheel_radius");
    front_track_ = get_parameter<float>(hardware_info, "front_track");
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EffibotE3Hardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::connect_()
{
  RCLCPP_INFO(logger_, "Init communication with robot");

  if (serial_port_.isOpen()) {
    send_null_command_();
    start_serial_listening_thread_();
    return hardware_interface::return_type::OK;
  } else {
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::disconnect_()
{
  RCLCPP_INFO(logger_, "Close communication with robot");
  send_null_command_();
  serial_sender_.sendSetInactiveCommand(serial_port_);
  stop_serial_listening_thread_();
  return hardware_interface::return_type::OK;
}


//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::read()
{
  // RCLCPP_INFO(logger_, "Read data from robot");
  // serial_sender_.sendSetActiveCommand(serial_port_);

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::write()
{
  // RCLCPP_INFO(logger_, "Send command to robot");

  get_hardware_command_();
  serial_sender_.sendHeartbeat(serial_port_);
  send_command_();

// #ifndef NDEBUG
//   write_log_data_();
// #endif

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::send_null_command_()
{
  serial_sender_.sendSetSpeedCommand(0.0, 0.0, serial_port_);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::send_command_()
{
  double v = (front_right_wheel_angular_speed_command_ + rear_left_wheel_angular_speed_command_) *
    0.5 * front_wheel_radius_;

  double w = (front_right_wheel_angular_speed_command_ - rear_left_wheel_angular_speed_command_) *
    front_wheel_radius_ / front_track_;

  serial_sender_.sendSetSpeedCommand(v, w, serial_port_);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::get_hardware_command_()
{
  core::HardwareCommand4WD command = hardware_interface_->get_command();
  front_left_wheel_angular_speed_command_ = command.frontLeftWheelSpinningSetPoint;
  front_right_wheel_angular_speed_command_ = command.frontRightWheelSpinningSetPoint;
  rear_left_wheel_angular_speed_command_ = command.rearLeftWheelSpinningSetPoint;
  rear_right_wheel_angular_speed_command_ = command.rearRightWheelSpinningSetPoint;
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::set_hardware_state_()
{
  core::HardwareState4WD state;
  state.frontLeftWheelSpinningMotion.velocity = front_left_wheel_angular_speed_measure_;
  state.frontLeftWheelSpinningMotion.torque = front_left_wheel_torque_measure_;
  state.frontRightWheelSpinningMotion.velocity = front_right_wheel_angular_speed_measure_;
  state.frontRightWheelSpinningMotion.torque = front_right_wheel_torque_measure_;
  state.rearLeftWheelSpinningMotion.velocity = rear_left_wheel_angular_speed_measure_;
  state.rearLeftWheelSpinningMotion.torque = rear_left_wheel_torque_measure_;
  state.rearRightWheelSpinningMotion.velocity = rear_right_wheel_angular_speed_measure_;
  state.rearRightWheelSpinningMotion.torque = rear_right_wheel_torque_measure_;
  hardware_interface_->set_state(state);
}


//-----------------------------------------------------------------------------
void EffibotE3Hardware::start_serial_listening_thread_()
{
  serial_listening_thread_run_ = true;
  serial_listening_thread_ = std::make_unique<std::thread>(&EffibotE3Hardware::receive_data_, this);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::stop_serial_listening_thread_()
{
  serial_listening_thread_run_ = false;
  if (serial_listening_thread_->joinable()) {
    serial_listening_thread_->join();
  }
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::receive_data_()
{
  serial_receiver_.reInitialize();

  std::string buffer;
  while (rclcpp::ok() && serial_listening_thread_run_) {

    // // // If there is a byte to read on serial port
    if (serial_port_.waitReadable()) {

      serial_port_.waitByteTimes(EFFIBOT_E3_MESSAGE_SIZE);
      std::string message = serial_port_.read(serial_port_.available());
      serial_receiver_.processMessage(message);

      // int serial_port_size = serial_port_string.size();


      // RCLCPP_WARN_STREAM(logger_, "\t\t" << convertMessageToHexa(buffer));


      // // If there is no incomplete frame and the first byte read is not a message header : do not process message
      // if (!(serial_receiver_.isIncompleteFrame()) &&
      //   (((unsigned char)serial_port_string[0]) != EFFIBOT_E3_MESSAGE_HEADER) )
      // {
      //   RCLCPP_WARN(
      //     logger_,
      //     "[E3Receiver] Serial port read : There is no incomplete frame and the first byte read is not a message header");
      //   RCLCPP_WARN_STREAM(logger_, "\t\t" << convertMessageToHexa(serial_port_string));

      //   // ReInitialize incomplete frame variables...
      //   serial_receiver_.reInitialize();
      // } else {
      //   serial_receiver_.processMessage(serial_port_string);
      // }
    } else {
      RCLCPP_WARN(logger_, "Timeout elapsed! No data available on serial port...");
    }

    // Sleep a time in order to use less CPU.
    // Do NOT sleep more than 100ms because spinOnce() must be executed
    // to send new speeds (via joyVelCallback then controllerTimerCallback)
    // usleep(SLEEP_TIME_RECEIVING_NO_MESSAGE);       // sleep 100ms
  }
}


#ifndef NDEBUG
// //-----------------------------------------------------------------------------
// void EffibotE3Hardware::open_log_file_()
// {
//   debug_file_.open(
//     std::string("effibote3.dat").c_str(),
//     std::fstream::in | std::fstream::out | std::fstream::trunc);
// }

// //-----------------------------------------------------------------------------
// void EffibotE3Hardware::write_log_header_()
// {
//   if (debug_file_.is_open()) {
//     debug_file_ << "# time, ";
//     debug_file_ << " FLS, " << " FRS, ";
//     debug_file_ << " RLS, " << " RRS, ";
//     debug_file_ << " FLS_cmd, " << " FRS_cmd, ";
//     debug_file_ << " RLS_cmd, " << " RRS_cmd, ";
//   }
// }

// //-----------------------------------------------------------------------------
// void EffibotE3Hardware::write_log_data_()
// {
//   // if (debug_file_.is_open()) {
//   //   auto now = std::chrono::system_clock::now();
//   //   auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

//   //   debug_file_ << std::setprecision(10);
//   //   debug_file_ << now_ns.time_since_epoch().count() << " ";
//   //   debug_file_ << front_left_wheel_speed_measure_ <<
//   //     " " << front_right_wheel_speed_measure_ << " ";
//   //   debug_file_ << rear_left_wheel_speed_measure_ <<
//   //     " " << rear_right_wheel_speed_measure_ << " ";
//   //   debug_file_ << front_left_wheel_speed_command_ <<
//   //     " " << front_right_wheel_speed_command_ << " ";
//   //   debug_file_ << rear_left_wheel_speed_command_ <<
//   //     " " << rear_right_wheel_speed_command_ << " \n";
//   // }
// }
#endif

}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ros2::EffibotE3Hardware, hardware_interface::SystemInterface)
