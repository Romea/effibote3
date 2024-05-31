#ifndef EFFIBOTE3_CONTROL_EFFIBOTE3_COMMUNICATOR_RECEIVER_H
#define EFFIBOTE3_CONTROL_EFFIBOTE3_COMMUNICATOR_RECEIVER_H

/*
 * Contains the declarations of the functions of the class Effibote3CommunicatorReceiver
 *
 * Written by Luc Pain. June 2017.
 */

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

// local
#include "effibote3_hardware/effibote3_param.hpp"
#include "effibote3_hardware/effibote3_display.hpp"

namespace romea
{
namespace ros2
{

class Effibote3CommunicatorReceiver
{
public:
  Effibote3CommunicatorReceiver();
  ~Effibote3CommunicatorReceiver() = default;

  /**
   * @brief Re-initialize variables used to manage incomplete frames and the counter of registered data.
   */
  void reInitialize();

  /**
   * @brief isIncompleteFrame
   * @return a bool : the is_incomplete_frame_ value
   */
  bool isIncompleteFrame();

  /**
   * @brief Process the message contained by serial_port_str
   * @param serial_port_str a std::string
   * @param date  : date when the serial_port_str has been read
   */
  void processMessage(std::string & serial_port_str);

private:
  void decodeFullPackOfData_(std::string & message_to_decode);

  /**
   * @brief Decode the frame given in argument.
   * @param frame  : frame to decode
   * @return id of the frame
   */
  unsigned char processFrame_(std::vector<unsigned char> & frame);

  void receiveSetSpeedCommandReplyMessage_(std::vector<unsigned char> & frame);
  void receiveActiveModeReplyMessage_(std::vector<unsigned char> & frame);
  void receiveInactiveModeReplyMessage_(std::vector<unsigned char> & frame);
  void receiveRightWheelSpeedMessage_(std::vector<unsigned char> & frame);
  void receiveLeftWheelSpeedMessage_(std::vector<unsigned char> & frame);
  void receiveRightMotorCurrentMessage_(std::vector<unsigned char> & frame);
  void receiveLeftMotorCurrentMessage_(std::vector<unsigned char> & frame);
  void receiveAngularSpeedsReplyMessage_(std::vector<unsigned char> & frame);
  void receiveAccelerationsReplyMessage_(std::vector<unsigned char> & frame);
  void receiveStatusMessage_(std::vector<unsigned char> & frame);
  void receiveSetupOutputsReplyMessage_(std::vector<unsigned char> & frame);
  void receiveInvalidMessageErrorMessage_(std::vector<unsigned char> & frame);

private:
  std::mutex mutex_;
  /** - angular speed (in rad/s) */
  double angular_speed_x_;
  double angular_speed_y_;
  double angular_speed_z_;

  /** - acceleration (in m.s-2) */
  double acceleration_x_;
  double acceleration_y_;
  double acceleration_z_;

  /** - motor current (in A) */
  float left_motor_current_rear_;
  float left_motor_current_front_;
  float right_motor_current_rear_;
  float right_motor_current_front_;

  /** - wheel speed (in m/s) */
  float left_wheel_speed_rear_;
  float left_wheel_speed_front_;
  float right_wheel_speed_rear_;
  float right_wheel_speed_front_;

  /** - linear speed (in m/s) */
  double linear_speed_;

  /** - temperature (in degCelsius) */
  double temperature_;


  rclcpp::Logger logger_;
  std::string buffer_;
  // rclcpp::Time data_time_;

  // Variables to manage incomplete frames (if we read serial port before that the robot has finished to write the frame)
  std::string incomplete_frame_str_;
  // rclcpp::Time date_incomplete_frame_;
  bool is_incomplete_frame_;   // 'false', or 'true' if there is an incomplete frame waiting to be filled.

  int registered_data_counter_; // is in range [0,number_of_interesting_data_type_]
                                // Must be incremented in functions that decode a frame (e.g. receiveAccelerationsReplyMessage_, ...)

  int number_of_interesting_data_type_; // Number of frames that get relevant data


};

} // namespace romea
} // namespace ros2


#endif
