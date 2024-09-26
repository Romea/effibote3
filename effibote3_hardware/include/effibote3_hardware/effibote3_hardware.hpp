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

#ifndef EFFIBOTE3_HARDWARE__EFFIBOTE3_HARDWARE_HPP_
#define EFFIBOTE3_HARDWARE__EFFIBOTE3_HARDWARE_HPP_

// std
#include <array>
#include <atomic>
#include <memory>
#include <thread>
#include <fstream>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_common_utils/ros_versions.hpp"
#include "romea_mobile_base_hardware/hardware_system_interface.hpp"

// serial
#include "serial/serial.h"


namespace romea
{
namespace ros2
{

class EffibotE3Hardware : public HardwareSystemInterface4WD
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EffibotE3Hardware);

  EffibotE3Hardware();

  virtual ~EffibotE3Hardware();

#if ROS_DISTRO == ROS_GALACTIC
  hardware_interface::return_type read()override;

  hardware_interface::return_type write()override;
#else
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;
#endif

private:
  hardware_interface::return_type connect_() override;

  hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info) override;


  void send_data_(const uint8_t & command_id);

  void send_command_();

  void send_null_command_();

  void send_control_command_(const uint8_t & command_id);

  void send_set_active_command_();

  void send_set_inactive_command_();

  void send_heartbeat_();

  void read_frame_();

  void read_right_wheel_speeds_();

  void read_left_wheel_speeds_();

  void start_serial_listening_thread_();

  void stop_serial_listening_thread_();

  void get_hardware_command_();

  void set_hardware_state_();

  void read_data_();

#ifndef NDEBUG
  void open_log_file_();
  void write_log_header_();
  void write_log_data_();
#endif

private:
  std::unique_ptr<std::thread> serial_listening_thread_;
  std::atomic<bool> serial_listening_thread_run_;
  std::array<uint8_t, 13> sended_frame_data_;
  std::array<uint8_t, 13> received_frame_data_;
  uint8_t sequence_number_;

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

}  // namespace ros2
}  // namespace romea

#endif  // EFFIBOTE3_HARDWARE__EFFIBOTE3_HARDWARE_HPP_
