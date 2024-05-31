#ifndef EFFIBOTE3_CONTROL_EFFIBOTE3_PARAM_H
#define EFFIBOTE3_CONTROL_EFFIBOTE3_PARAM_H

#include <math.h>  // PI


namespace romea
{
namespace ros2
{


/**
 * @brief The ids of the messages sent to the E3.
 */
enum SentMessageId
{
  OUT_SET_MOTOR_COMMAND         = 0x08, // To send a motor command
  OUT_SET_SPEED_COMMAND         = 0x10, // To send a speed command
  OUT_SET_ACTIVE_MODE           = 0x12, // To go to active mode (enable power and release breaks)
  OUT_SET_INACTIVE_MODE         = 0x14, // To go to inactive mode
  OUT_HEARTBEAT                 = 0x16, // To tell the robot that we are alive
  OUT_REBOOT                    = 0x18, // To tell the robot to reboot
  OUT_SET_OUTPUTS               = 0x1A, // To set output states
};

/**
 * @brief The ids of the messages received from the E3.
 */
enum ReceivedMessageId
{
  IN_SET_SPEED_COMMAND         = 0x11, // Answer to OUT_SET_SPEED_COMMAND
  IN_SET_ACTIVE_MODE           = 0x13, // Answer to OUT_SET_ACTIVE_MODE
  IN_SET_INACTIVE_MODE         = 0x15, // Answer to OUT_SET_INACTIVE_MODE
  IN_REBOOT                    = 0x19, // Answer to OUT_REBOOT
  IN_SET_OUTPUTS               = 0x1B, // Answer to OUT_SET_OUTPUTS

  IN_RIGHT_WHEEL_SPEED         = 0x82, // The speed of right wheels when sent periodically
  IN_LEFT_WHEEL_SPEED          = 0x84, // The speed of left wheels when sent periodically
  IN_RIGHT_MOTOR_CURRENT       = 0x86, // The current of right motors when sent periodically
  IN_LEFT_MOTOR_CURRENT        = 0x88, // The current of left motors when sent periodically
  IN_STATUS                    = 0x8A, // The status of the vehicle when sent periodically
  IN_ANGULAR_SPEEDS            = 0x8C, // The angular speeds of the vehicle when sent periodically
  IN_ACCELERATIONS             = 0x8E, // The accelerations of the vehicle when sent periodically

  IN_INVALID_MESSAGE_ERROR     = 0xFF, // Sent when the vehicle recevied an invalid message from us
};


//===========================================================================


/**
 * @brief The values of constants used in messages generation
 */
static const unsigned char EFFIBOT_E3_MESSAGE_HEADER = 0xAA;
static const unsigned int EFFIBOT_E3_MESSAGE_SIZE = 13;
static const unsigned int EFFIBOT_E3_CRC_FIRST_BYTE = 1;
static const unsigned int EFFIBOT_E3_CRC_LAST_BYTE = 2;
static const unsigned int EFFIBOT_E3_POSITION_OF_FIRST_DATA_TO_COMPUTE_CRC = 3;
static const unsigned int EFFIBOT_E3_POSITION_OF_LAST_DATA_TO_COMPUTE_CRC = EFFIBOT_E3_MESSAGE_SIZE;
static const unsigned int EFFIBOT_E3_LENGTH_OF_DATA_TO_COMPUTE_CRC =
  EFFIBOT_E3_POSITION_OF_LAST_DATA_TO_COMPUTE_CRC -
  EFFIBOT_E3_POSITION_OF_FIRST_DATA_TO_COMPUTE_CRC;
static const unsigned int EFFIBOT_E3_ID_OFFSET = 4;
static const unsigned int EFFIBOT_E3_DATA_BEGIN = 5;
static const unsigned int EFFIBOT_E3_DATA_END = EFFIBOT_E3_MESSAGE_SIZE;
static const double EFFIBOT_E3_WHEEL_SPEED_RESOLUTION = 1e-3; // the values comming from the vehicle are in mm.s-1
static const double EFFIBOT_E3_MOTOR_CURRENT_RESOLUTION = 1e-3; // the values comming from the vehicle are in milliampere
static const double EFFIBOT_E3_MIN_BATTERY_VOLTAGE = 20.0;
static const double EFFIBOT_E3_MAX_BATTERY_VOLTAGE = 40.0;
static const double EFFIBOT_E3_DIFFERENCE_BATTERY_VOLTAGE = EFFIBOT_E3_MAX_BATTERY_VOLTAGE -
  EFFIBOT_E3_MIN_BATTERY_VOLTAGE;

/**
 * @brief Some parameters to control the E3
 */
static const double LINEAR_SPEED_FACTOR = 1.0 / EFFIBOT_E3_WHEEL_SPEED_RESOLUTION; // m.s-1 to mm.s-1)
static const double ANGULAR_SPEED_FACTOR = 1000. * 180. / M_PI; // radian to millidegrees (= about 57297)

static const double MAX_LINEAR_SPEED_M_S = 3; // maximal linear speed value in m/s  // 2.25 m.s-1 wheel up
static const double MAX_ANGULAR_SPEED_DEG_S = 286.0; // maximal linear speed value in degrees/s _ 286 deg/s = 5 rad/s
static const double MAX_ANGULAR_SPEED_RAD_S = MAX_ANGULAR_SPEED_DEG_S * M_PI / 180; // maximal linear speed value in rad/s

} // namespace ros2
} // namespace romea

#endif
