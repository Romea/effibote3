// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <string>
#include <memory>
#include <unistd.h>

// romea
#include "effibote3_hardware/effibote3_hardware.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"

namespace
{

std::string convertMessageToHexa(const std::string & mess)
{
  std::ostringstream strshex;

  // Conversion std::string to std::ostringstream
  for (size_t i = 0; i < mess.size(); i++) {
    strshex << "  " << std::hex << (int) ((unsigned char) (mess[i]));
  }

  return (std::string) (strshex.str());
}

enum SentMessageId
{
  OUT_SET_MOTOR_COMMAND         = 0x08,  // To send a motor command
  OUT_SET_SPEED_COMMAND         = 0x10,  // To send a speed command
  OUT_SET_ACTIVE_MODE           = 0x12,  // To go to active mode (enable power and release breaks)
  OUT_SET_INACTIVE_MODE         = 0x14,  // To go to inactive mode
  OUT_HEARTBEAT                 = 0x16,  // To tell the robot that we are alive
  OUT_REBOOT                    = 0x18,  // To tell the robot to reboot
  OUT_SET_OUTPUTS               = 0x1A,  // To set output states
};

enum ReceivedMessageId
{
  IN_SET_SPEED_COMMAND         = 0x11,  // Answer to OUT_SET_SPEED_COMMAND
  IN_SET_ACTIVE_MODE           = 0x13,  // Answer to OUT_SET_ACTIVE_MODE
  IN_SET_INACTIVE_MODE         = 0x15,  // Answer to OUT_SET_INACTIVE_MODE
  IN_REBOOT                    = 0x19,  // Answer to OUT_REBOOT
  IN_SET_OUTPUTS               = 0x1B,  // Answer to OUT_SET_OUTPUTS

  IN_RIGHT_WHEEL_SPEED         = 0x82,  // The speed of right wheels when sent periodically
  IN_LEFT_WHEEL_SPEED          = 0x84,  // The speed of left wheels when sent periodically
  IN_RIGHT_MOTOR_CURRENT       = 0x86,  // The current of right motors when sent periodically
  IN_LEFT_MOTOR_CURRENT        = 0x88,  // The current of left motors when sent periodically
  IN_STATUS                    = 0x8A,  // The status of the vehicle when sent periodically
  IN_ANGULAR_SPEEDS            = 0x8C,  // The angular speeds of the vehicle when sent periodically
  IN_ACCELERATIONS             = 0x8E,  // The accelerations of the vehicle when sent periodically

  IN_INVALID_MESSAGE_ERROR     = 0xFF,  // Sent when the vehicle recevied an invalid message from us
};


const uint8_t EFFIBOT_E3_MESSAGE_HEADER = 0xAA;
const unsigned int EFFIBOT_E3_MESSAGE_SIZE = 13;
const unsigned int EFFIBOT_E3_CRC_FIRST_BYTE = 1;
const unsigned int EFFIBOT_E3_CRC_LAST_BYTE = 2;
const unsigned int EFFIBOT_E3_POSITION_OF_FIRST_DATA_TO_COMPUTE_CRC = 3;
const unsigned int EFFIBOT_E3_POSITION_OF_LAST_DATA_TO_COMPUTE_CRC = 13;
const unsigned int EFFIBOT_E3_LENGTH_OF_DATA_TO_COMPUTE_CRC = 10;
const unsigned int EFFIBOT_E3_ID_OFFSET = 4;
const unsigned int EFFIBOT_E3_DATA_BEGIN = 5;
const unsigned int EFFIBOT_E3_DATA_END = EFFIBOT_E3_MESSAGE_SIZE;


template<class Iterator>
static uint16_t compute_crc(Iterator begin, Iterator end)
{
  uint16_t crc = 0xFFFF;
  uint16_t polynome = 0xA001;
  uint16_t parity = 0;
  for (Iterator it = begin; it != end; ++it) {
    crc ^= static_cast<uint8_t>(*it);
    for (unsigned int bitIndex = 0; bitIndex < 8; ++bitIndex) {
      parity = crc;
      crc >>= 1;
      if (parity % 2 == true) {
        crc ^= polynome;
      }
    }
  }
  return crc;
}

static uint16_t compute_crc(const std::array<unsigned char, 13> & data)
{
  return compute_crc(
    data.begin() + static_cast<std::ptrdiff_t>(EFFIBOT_E3_POSITION_OF_FIRST_DATA_TO_COMPUTE_CRC),
    data.begin() + static_cast<std::ptrdiff_t>(EFFIBOT_E3_POSITION_OF_LAST_DATA_TO_COMPUTE_CRC));
}

static void set_crc_in_message(uint16_t crc, std::array<unsigned char, 13> & message)
{
  assert(message.size() == EFFIBOT_E3_MESSAGE_SIZE);
  for (std::size_t i = EFFIBOT_E3_CRC_FIRST_BYTE;
    i <= EFFIBOT_E3_CRC_LAST_BYTE; ++i)
  {
    message[i] = static_cast<uint8_t>(crc);
    crc >>= 8;
  }
}

const double EFFIBOT_E3_WHEEL_SPEED_RESOLUTION = 1e-3;  // mm.s-1
const double EFFIBOT_E3_MOTOR_CURRENT_RESOLUTION = 1e-3;  // milliampere
const double EFFIBOT_E3_MIN_BATTERY_VOLTAGE = 20.0;
const double EFFIBOT_E3_MAX_BATTERY_VOLTAGE = 40.0;
const double EFFIBOT_E3_DIFFERENCE_BATTERY_VOLTAGE = 20.0;

const double LINEAR_SPEED_FACTOR = 1.0 / EFFIBOT_E3_WHEEL_SPEED_RESOLUTION;  // m.s-1 to mm.s-1)
const double ANGULAR_SPEED_FACTOR = 1000. * 180. / M_PI;  // radian to millidegrees (= about 57297)

const double MAX_LINEAR_SPEED_M_S = 3;  // m/s  // 2.25 m.s-1 wheel up
const double MAX_ANGULAR_SPEED_DEG_S = 286.0;  // degrees/s
const double MAX_ANGULAR_SPEED_RAD_S = MAX_ANGULAR_SPEED_DEG_S * M_PI / 180;  // rad/s
}  // namespace


namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
EffibotE3Hardware::EffibotE3Hardware()
: HardwareSystemInterface4WD(),
  serial_listening_thread_(nullptr),
  serial_listening_thread_run_(false),
  sequence_number_(0),
  front_track_(0),
  front_wheel_radius_(0),
  rear_wheel_radius_(0),
  front_left_wheel_angular_speed_command_(0),
  front_right_wheel_angular_speed_command_(0),
  rear_left_wheel_angular_speed_command_(0),
  rear_right_wheel_angular_speed_command_(0),
  front_left_wheel_angular_speed_measure_(0),
  front_right_wheel_angular_speed_measure_(0),
  rear_left_wheel_angular_speed_measure_(0),
  rear_right_wheel_angular_speed_measure_(0),
  front_left_wheel_torque_measure_(0),
  front_right_wheel_torque_measure_(0),
  rear_left_wheel_torque_measure_(0),
  rear_right_wheel_torque_measure_(0),
  logger_(rclcpp::get_logger("effibote3_hardware"))
{
  try {
    serial_port_.setPort(
      "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0");
    serial_port_.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);  // Timeout = 1s
    serial_port_.setTimeout(to);
    serial_port_.open();
  } catch (serial::IOException & e) {
    RCLCPP_ERROR_STREAM(logger_, "Unable to open port ");
    std::cerr << e.what() << std::endl;
  }

#ifndef NDEBUG
  open_log_file_();
  write_log_header_();
#endif
}

//-----------------------------------------------------------------------------
EffibotE3Hardware::~EffibotE3Hardware()
{
  // force deactive when interface has not been deactivated by controller manager but by ctrl-c
  if (lifecycle_state_.id() == 3) {
    on_deactivate(lifecycle_state_);
  }
}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::load_info_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_ERROR_STREAM(logger_, "load_info");

  try {
    // front_wheel_radius_ = get_parameter<float>(hardware_info, "front_wheel_radius");
    // rear_wheel_radius_ = get_parameter<float>(hardware_info, "rear_wheel_radius");
    // front_track_ = get_parameter<float>(hardware_info, "front_track");
    front_wheel_radius_ = get_front_wheel_radius(hardware_info);
    rear_wheel_radius_ = get_rear_wheel_radius(hardware_info);
    front_track_ = get_front_track(hardware_info);
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
    send_set_active_command_();
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
  send_set_inactive_command_();
  stop_serial_listening_thread_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::read()
{
  // RCLCPP_INFO(logger_, "Read data from robot");
  // serial_sender_.sendSetActiveCommand(serial_port_);

  set_hardware_state_();

#ifndef NDEBUG
  write_log_data_();
#endif

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::write()
{
  // RCLCPP_INFO(logger_, "Send command to robot");

  get_hardware_command_();
  send_heartbeat_();
  send_command_();

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::get_hardware_command_()
{
  core::HardwareCommand4WD command = hardware_interface_->get_hardware_command();
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
  hardware_interface_->set_feedback(state);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::send_data_(const uint8_t & command_id)
{
  uint16_t crc;
  sended_frame_data_[0] = EFFIBOT_E3_MESSAGE_HEADER;
  sended_frame_data_[3] = sequence_number_++;
  sended_frame_data_[4] = command_id;

  crc = compute_crc(sended_frame_data_);
  set_crc_in_message(crc, sended_frame_data_);

  if (sequence_number_ <= 0) {
    ++sequence_number_;
  }

  serial_port_.write(sended_frame_data_.data(), EFFIBOT_E3_MESSAGE_SIZE);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::send_null_command_()
{
  sended_frame_data_[5] = 0x00;
  sended_frame_data_[6] = 0x00;
  sended_frame_data_[7] = 0x00;
  sended_frame_data_[8] = 0x00;
  sended_frame_data_[9] = 0x00;
  sended_frame_data_[10] = 0x00;
  sended_frame_data_[11] = 0x00;
  sended_frame_data_[12] = 0x00;
  send_data_(OUT_SET_SPEED_COMMAND);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::send_command_()
{
  double linear_speed = 0.5 * front_wheel_radius_ *
    (front_right_wheel_angular_speed_command_ + rear_left_wheel_angular_speed_command_);

  // check if speed is in good range and modify its value if necessary
  if (linear_speed > MAX_LINEAR_SPEED_M_S) {
    linear_speed = MAX_LINEAR_SPEED_M_S;
  } else if (linear_speed < -MAX_LINEAR_SPEED_M_S) {
    linear_speed = -MAX_LINEAR_SPEED_M_S;
  }

  // insert the value in the 4 bytes mess[5..8] of the message
  int32_t linear_speed_int = static_cast<int32_t>(linear_speed * LINEAR_SPEED_FACTOR);
  sended_frame_data_[5] = (linear_speed_int >> 24) & 0xFF;
  sended_frame_data_[6] = (linear_speed_int >> 16) & 0xFF;
  sended_frame_data_[7] = (linear_speed_int >> 8) & 0xFF;
  sended_frame_data_[8] = linear_speed_int & 0xFF;

  double angular_speed = front_wheel_radius_ / front_track_ *
    (front_right_wheel_angular_speed_command_ - front_left_wheel_angular_speed_command_);


  // check if speed is in good range and modify its value if necessary
  if (angular_speed > MAX_ANGULAR_SPEED_RAD_S) {
    angular_speed = MAX_ANGULAR_SPEED_RAD_S;
  } else if (angular_speed < -MAX_ANGULAR_SPEED_RAD_S) {
    angular_speed = -MAX_ANGULAR_SPEED_RAD_S;
  }

  // insert the value in the 4 bytes mess[9..12] of the message
  int32_t angular_speed_int = int32_t(angular_speed * ANGULAR_SPEED_FACTOR);
  sended_frame_data_[9] = (angular_speed_int >> 24) & 0xFF;
  sended_frame_data_[10] = (angular_speed_int >> 16) & 0xFF;
  sended_frame_data_[11] = (angular_speed_int >> 8) & 0xFF;
  sended_frame_data_[12] = angular_speed_int & 0xFF;

  send_data_(OUT_SET_SPEED_COMMAND);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::send_control_command_(const uint8_t & command_id)
{
  sended_frame_data_[5] = 0x00;
  sended_frame_data_[6] = 0x00;
  sended_frame_data_[7] = 0x00;
  sended_frame_data_[8] = 0x00;
  sended_frame_data_[9] = 0x00;
  sended_frame_data_[10] = 0x00;
  sended_frame_data_[11] = 0x00;
  sended_frame_data_[12] = 0x00;
  send_data_(command_id);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::send_set_active_command_()
{
  send_control_command_(OUT_SET_ACTIVE_MODE);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::send_set_inactive_command_()
{
  send_control_command_(OUT_SET_INACTIVE_MODE);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::send_heartbeat_()
{
  send_control_command_(OUT_HEARTBEAT);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::start_serial_listening_thread_()
{
  serial_listening_thread_run_ = true;
  serial_listening_thread_ = std::make_unique<std::thread>(&EffibotE3Hardware::read_data_, this);
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
void EffibotE3Hardware::read_data_()
{
  std::string buffer;
  while (rclcpp::ok() && serial_listening_thread_run_) {
    if (serial_port_.waitReadable()) {
      serial_port_.waitByteTimes(EFFIBOT_E3_MESSAGE_SIZE);
      buffer += serial_port_.read(serial_port_.available());

      auto pos = buffer.find_first_of(EFFIBOT_E3_MESSAGE_HEADER);
      if (pos != std::string::npos) {
        buffer.erase(0, pos);

        while (buffer.size() >= EFFIBOT_E3_MESSAGE_SIZE + 1) {

          if (buffer[EFFIBOT_E3_MESSAGE_SIZE] == buffer[0]) {
            std::copy(
              buffer.begin(),
              buffer.begin() + EFFIBOT_E3_MESSAGE_SIZE,
              received_frame_data_.data());

            read_frame_();
          }

          pos = buffer.find_first_of(EFFIBOT_E3_MESSAGE_HEADER, 1);
          if (pos != std::string::npos) {
            buffer.erase(0, pos);
          } else {
            break;
          }
        }
      }
    } else {
      RCLCPP_WARN(logger_, "Timeout elapsed! No data available on serial port...");
    }
  }
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::read_frame_()
{
  uint16_t crc = compute_crc(received_frame_data_);
  if (received_frame_data_[EFFIBOT_E3_CRC_FIRST_BYTE] == uint8_t(crc) &&
    received_frame_data_[EFFIBOT_E3_CRC_LAST_BYTE] == uint8_t(crc >>= 8))
  {

    // Get the id of the frame
    unsigned char id = received_frame_data_[EFFIBOT_E3_ID_OFFSET];

    // RCLCPP_WARN_STREAM(logger_, "\t\t id" << id);

    // Process message
    switch (id) {
      case IN_RIGHT_WHEEL_SPEED:
        read_right_wheel_speeds_();
        break;
      case IN_LEFT_WHEEL_SPEED:
        read_left_wheel_speeds_();
        break;
      // case IN_STATUS:
      //   receiveStatusMessage_(frame);
      //   break;
      // case IN_SET_OUTPUTS:
      //   receiveSetupOutputsReplyMessage_(frame);
      //   break;
      // case IN_INVALID_MESSAGE_ERROR:
      //   receiveInvalidMessageErrorMessage_(frame);
      //   break;
      default:
        break;
    }
  }
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::read_right_wheel_speeds_()
{
  // Conversion FROM std::vector<unsigned char> TO int32_t
  // The ..._raw values are in mm.s-1
  int32_t rear_right_wheel_speed_raw =
    (int32_t) ((received_frame_data_[5] << 24) | (received_frame_data_[6] << 16) |
    (received_frame_data_[7] << 8) | received_frame_data_[8]);

  int32_t front_right_wheel_speed_raw =
    (int32_t) ((received_frame_data_[9] << 24) | (received_frame_data_[10] << 16) |
    (received_frame_data_[11] << 8) | received_frame_data_[12]);

  // Conversion FROM mm.s-1 TO rad.s-1
  rear_right_wheel_angular_speed_measure_ = EFFIBOT_E3_WHEEL_SPEED_RESOLUTION *
    static_cast<float>(rear_right_wheel_speed_raw) / rear_wheel_radius_;

  front_right_wheel_angular_speed_measure_ = EFFIBOT_E3_WHEEL_SPEED_RESOLUTION *
    static_cast<float>(front_right_wheel_speed_raw) / front_wheel_radius_;

// #ifndef NDEBUG
//   RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Right Wheel Speed :");
//   RCLCPP_INFO_STREAM(logger_, "\t\t\t rearRightSpeed  (m/s) = " << rear_right_wheel_speed_raw);
//   RCLCPP_INFO_STREAM(logger_, "\t\t\t frontRightSpeed (m/s) = " << front_right_wheel_speed_raw);
// #endif
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::read_left_wheel_speeds_()
{
  // Conversion FROM std::vector<unsigned char> TO int32_t
  // The ..._raw values are in mm.s-1
  int32_t rear_left_wheel_speed_raw =
    (int32_t) ((received_frame_data_[5] << 24) | (received_frame_data_[6] << 16) |
    (received_frame_data_[7] << 8) | received_frame_data_[8]);

  int32_t front_left_wheel_speed_raw =
    (int32_t) ((received_frame_data_[9] << 24) | (received_frame_data_[10] << 16) |
    (received_frame_data_[11] << 8) | received_frame_data_[12]);

  // Conversion FROM mm.s-1 TO rad.s-1
  rear_left_wheel_angular_speed_measure_ = EFFIBOT_E3_WHEEL_SPEED_RESOLUTION *
    static_cast<float>(rear_left_wheel_speed_raw) / rear_wheel_radius_;

  front_left_wheel_angular_speed_measure_ = EFFIBOT_E3_WHEEL_SPEED_RESOLUTION *
    static_cast<float>(front_left_wheel_speed_raw) / front_wheel_radius_;


  // #ifndef NDEBUG
  // RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Left Wheel Speed :");
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t rearLeftSpeed  (m/s) = " << rear_left_wheel_speed_raw);
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t frontLeftSpeed (m/s) = " << front_left_wheel_speed_raw);
  // #endif
}


#ifndef NDEBUG
//-----------------------------------------------------------------------------
void EffibotE3Hardware::open_log_file_()
{
  debug_file_.open(
    std::string("effibote3.dat").c_str(),
    std::fstream::in | std::fstream::out | std::fstream::trunc);
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::write_log_header_()
{
  if (debug_file_.is_open()) {
    debug_file_ << "# time, ";
    debug_file_ << " FLS, " << " FRS, ";
    debug_file_ << " RLS, " << " RRS, ";
    debug_file_ << " FLS_cmd, " << " FRS_cmd, ";
    debug_file_ << " RLS_cmd, " << " RRS_cmd, ";
  }
}

//-----------------------------------------------------------------------------
void EffibotE3Hardware::write_log_data_()
{
  if (debug_file_.is_open()) {
    auto now = std::chrono::system_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

    debug_file_ << std::setprecision(10);
    debug_file_ << now_ns.time_since_epoch().count() << " ";
    debug_file_ << front_left_wheel_angular_speed_measure_ * front_wheel_radius_ <<
      " " << front_right_wheel_angular_speed_measure_ * front_wheel_radius_ << " ";
    debug_file_ << rear_left_wheel_angular_speed_measure_ * rear_wheel_radius_ <<
      " " << rear_right_wheel_angular_speed_measure_ * rear_wheel_radius_ << " ";
    debug_file_ << front_left_wheel_angular_speed_command_ * front_wheel_radius_ <<
      " " << front_right_wheel_angular_speed_command_ * front_wheel_radius_ << " ";
    debug_file_ << rear_left_wheel_angular_speed_command_ * rear_wheel_radius_ <<
      " " << rear_right_wheel_angular_speed_command_ * rear_wheel_radius_ << " \n";
  }
}
#endif

}  // namespace ros2
}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ros2::EffibotE3Hardware, hardware_interface::SystemInterface)
