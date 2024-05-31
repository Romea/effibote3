#ifndef EFFIBOTE3_CONTROL_EFFIBOTE3_COMMUNICATOR_SENDER_H
#define EFFIBOTE3_CONTROL_EFFIBOTE3_COMMUNICATOR_SENDER_H


#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

// local
#include "effibote3_hardware/effibote3_param.hpp"
#include "effibote3_hardware/effibote3_display.hpp"

namespace romea
{
namespace ros2
{

class Effibote3CommunicatorSender
{

public:
  Effibote3CommunicatorSender();
  ~Effibote3CommunicatorSender() = default;

  void sendSetSpeedCommand(
    double linear_speed,
    double angular_speed,
    serial::Serial & serial_port);

  void sendSetMotorCommand(
    double rear_right_motor,
    double front_right_motor,
    double rear_left_motor,
    double front_left_motor,
    serial::Serial & serial_port);

  void sendRebootCommand(serial::Serial & serial_port);

  /**
   * @brief Puts the vehicle in active mode. This releases the breaks and
   * enables power. The vehicle starts to apply speed commands after 2 seconds.
   */
  void sendSetActiveCommand(serial::Serial & serial_port);

  /**
   * @brief Puts the vehicle in inactive mode. This disables power and applies
   * the breaks.
   */
  void sendSetInactiveCommand(serial::Serial & serial_port);

  /**
   * @brief sendHeartbeat
   */
  void sendHeartbeat(serial::Serial & serial_port);

private:
  void incrementSequenceNumber_();

  /**
   * @brief createMessage_ : Generates a message with the given id and data.
   * @param mess        The message that will be filled.
   * @param id          The id of the frame.
   * @param value1      The linear speed in m/s     OR  the rear_right_motor  current value.
   * @param value2      The angular speed in rad/s  OR  the front_right_motor current value.
   * @param value3                                      The rear_left_motor   current value.
   * @param value4                                      The front_left_motor  current value.
   */
  void createMessage_(
    std::vector<unsigned char> & mess,
    SentMessageId id,
    double value1,
    double value2,
    double value3,
    double value4);

  /**
   * @brief createMessage_ : Generates a message with the given id and no data.
   * @param mess    The message that will be filled
   * @param id      The id of the frame
   */
  void createMessage_(
    std::vector<unsigned char> & mess,
    SentMessageId id);

  /**
   * @brief sendMessage_ : Send a message with the given id and speeds/motor values to the robot via the given serial port
   * @param id            The id of the frame
   * @param value1        The linear speed in m/s     OR  the rear_right_motor  value
   * @param value2        The angular speed in rad/s  OR  the front_right_motor value
   * @param value3                                        The rear_left_motor   value
   * @param value4                                        The front_left_motor  value
   * @param serial_port   The serial port where the message is sent
   */
  void sendMessage_(
    SentMessageId id,
    double value1,
    double value2,
    double value3,
    double value4,
    serial::Serial & serial_port);

  /**
  * @brief sendMessage_ : Send a message with the given id to the robot via the given serial port (facultative)
  * @param id            The id of the frame
  * @param serial_port   The serial port where the message is sent
  */
  void sendMessage_(
    SentMessageId id,
    serial::Serial & serial_port);

private:
  unsigned char sequence_number_;
  rclcpp::Logger logger_;

};

}
}

#endif
