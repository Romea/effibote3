#ifndef EFFIBOTE3_HARDWARE_HPP_
#define EFFIBOTE3_HARDWARE_HPP_

#include "romea_mobile_base_hardware/hardware_system_interface.hpp"
#include "effibote3_hardware/effibote3_communicator_receiver.hpp"
#include "effibote3_hardware/effibote3_communicator_sender.hpp"

#include <rclcpp/macros.hpp>

#include <array>
#include <atomic>
#include <memory>
#include <thread>
#include <fstream>


namespace romea
{
namespace ros2
{

class EffibotE3Hardware : public HardwareSystemInterface4WD
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EffibotE3Hardware);

  EffibotE3Hardware();

  virtual hardware_interface::return_type read() override;

  virtual hardware_interface::return_type write() override;


// #ifndef NDEBUG
//   void open_log_file_();
//   void write_log_header_();
//   void write_log_data_();
// #endif

private:
  virtual hardware_interface::return_type connect_() override;

  virtual hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info) override;

  void send_command_();

  void send_null_command_();

  void start_serial_listening_thread_();

  void stop_serial_listening_thread_();

  void get_hardware_command_();

  void set_hardware_state_();

  void receive_data_();

private:
  std::unique_ptr<std::thread> serial_listening_thread_;
  std::atomic<bool> serial_listening_thread_run_;
  Effibote3CommunicatorReceiver serial_receiver_;
  Effibote3CommunicatorSender serial_sender_;

  float front_track_;
  float front_wheel_radius_;
  float rear_wheel_radius_;

  float front_left_wheel_angular_speed_command_;
  float front_right_wheel_angular_speed_command_;
  float rear_left_wheel_angular_speed_command_;
  float rear_right_wheel_angular_speed_command_;

  std::atomic<float> front_left_wheel_angular_speed_measure_;
  std::atomic<float> front_right_wheel_angular_speed_measure_;
  std::atomic<float> rear_left_wheel_angular_speed_measure_;
  std::atomic<float> rear_right_wheel_angular_speed_measure_;

  std::atomic<float> front_left_wheel_torque_measure_;
  std::atomic<float> front_right_wheel_torque_measure_;
  std::atomic<float> rear_left_wheel_torque_measure_;
  std::atomic<float> rear_right_wheel_torque_measure_;

  rclcpp::Logger logger_;
  serial::Serial serial_port_;

#ifndef NDEBUG
  std::fstream debug_file_;
#endif

};

}
}

#endif
