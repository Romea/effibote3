#include "effibote3_hardware/effibote3_communicator_sender.hpp"

namespace crc
{
/**
 *  Compute the CRC
 */
template<class Iterator>
static uint16_t computeCrc(Iterator begin, Iterator end)
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

static uint16_t computeCrc2(const std::vector<unsigned char> & data)
{
  return computeCrc(
    data.begin() +
    static_cast<std::ptrdiff_t>(romea::ros2::EFFIBOT_E3_POSITION_OF_FIRST_DATA_TO_COMPUTE_CRC),
    data.begin() +
    static_cast<std::ptrdiff_t>(romea::ros2::EFFIBOT_E3_POSITION_OF_LAST_DATA_TO_COMPUTE_CRC));
}

/**
 *  Set the CRC in the given message
 */
static void setCrcInMessage(uint16_t crc, std::vector<unsigned char> & message)
{
  assert(message.size() == romea::ros2::EFFIBOT_E3_MESSAGE_SIZE);
  for (std::size_t i = romea::ros2::EFFIBOT_E3_CRC_FIRST_BYTE;
    i <= romea::ros2::EFFIBOT_E3_CRC_LAST_BYTE; ++i)
  {
    message[i] = static_cast<uint8_t>(crc);
    crc >>= 8;
  }
}

} // end namespace crc

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
Effibote3CommunicatorSender::Effibote3CommunicatorSender()
: sequence_number_(0),
  logger_(rclcpp::get_logger("effibote3_hardware"))
{
}

//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::incrementSequenceNumber_()
{
  ++sequence_number_;
  if (sequence_number_ <= 0) {
    ++sequence_number_;
  }
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::createMessage_(
  std::vector<unsigned char> & mess,
  SentMessageId id,
  double value1,
  double value2,
  double value3,
  double value4)
{
  uint16_t crc;

  mess[0] = EFFIBOT_E3_MESSAGE_HEADER;
  mess[3] = sequence_number_;
  mess[4] = id;

  if (id == OUT_SET_SPEED_COMMAND) {
    double lin_speed = value1, ang_speed = value2;
    int speed_tmp = 0;

    //==============Linear speed:
    if (lin_speed == 0) {
      mess[5] = 0x00;
      mess[6] = 0x00;
      mess[7] = 0x00;
      mess[8] = 0x00;
    } else {
      //check if speed is in good range and modify its value if necessary
      if (lin_speed > MAX_LINEAR_SPEED_M_S) {
        lin_speed = MAX_LINEAR_SPEED_M_S;
      } else if (lin_speed < -MAX_LINEAR_SPEED_M_S) {lin_speed = -MAX_LINEAR_SPEED_M_S;}
      //insert the value in the 4 bytes mess[5..8] of the message
      speed_tmp = int32_t(lin_speed * LINEAR_SPEED_FACTOR);
      mess[5] = (speed_tmp >> 24) & 0xFF;
      mess[6] = (speed_tmp >> 16) & 0xFF;
      mess[7] = (speed_tmp >> 8) & 0xFF;
      mess[8] = speed_tmp & 0xFF;
    }

    //==============Angular speed:
    if (ang_speed == 0) {
      mess[9] = 0x00;
      mess[10] = 0x00;
      mess[11] = 0x00;
      mess[12] = 0x00;
    } else {
      //check if speed is in good range and modify its value if necessary
      if (ang_speed > MAX_ANGULAR_SPEED_RAD_S) {
        ang_speed = MAX_ANGULAR_SPEED_RAD_S;
      } else if (ang_speed < -MAX_ANGULAR_SPEED_RAD_S) {ang_speed = -MAX_ANGULAR_SPEED_RAD_S;}
      //insert the value in the 4 bytes mess[9..12] of the message
      speed_tmp = int32_t(ang_speed * ANGULAR_SPEED_FACTOR);
      mess[9] = (speed_tmp >> 24) & 0xFF;
      mess[10] = (speed_tmp >> 16) & 0xFF;
      mess[11] = (speed_tmp >> 8) & 0xFF;
      mess[12] = speed_tmp & 0xFF;
    }

// #ifndef NDEBUG
//     RCLCPP_INFO_STREAM(
//       logger_,
//       "[ CommunicatorSender::create_message() ]\t Linear speed (m/s) \t= " << lin_speed);
//     RCLCPP_INFO_STREAM(
//       logger_,
//       "[ CommunicatorSender::create_message() ]\t Angular speed (rad/s) \t= " << ang_speed);
// #endif

  } else if (id == OUT_SET_MOTOR_COMMAND) {
    double rear_right_motor = value1, front_right_motor = value2, rear_left_motor = value3,
      front_left_motor = value4;
    int value_motor_tmp = 0;

    value_motor_tmp = int16_t(front_left_motor * 1000);
    mess[5] = (value_motor_tmp >> 8) & 0xFF;
    mess[6] = value_motor_tmp & 0xFF;
    value_motor_tmp = int16_t(rear_left_motor * 1000);
    mess[7] = (value_motor_tmp >> 8) & 0xFF;
    mess[8] = value_motor_tmp & 0xFF;
    value_motor_tmp = int16_t(front_right_motor * 1000);
    mess[9] = (value_motor_tmp >> 8) & 0xFF;
    mess[10] = value_motor_tmp & 0xFF;
    value_motor_tmp = int16_t(rear_right_motor * 1000);
    mess[11] = (value_motor_tmp >> 8) & 0xFF;
    mess[12] = value_motor_tmp & 0xFF;

// #ifndef NDEBUG
//     RCLCPP_INFO_STREAM(
//       logger_,
//       "[ CommunicatorSender::create_message() ]\t Rear_right_motor \t= " << rear_right_motor);
//     RCLCPP_INFO_STREAM(
//       logger_,
//       "[ CommunicatorSender::create_message() ]\t Front_right_motor \t= " << front_right_motor);
//     RCLCPP_INFO_STREAM(
//       logger_,
//       "[ CommunicatorSender::create_message() ]\t Rear_left_motor \t= " << rear_left_motor);
//     RCLCPP_INFO_STREAM(
//       logger_,
//       "[ CommunicatorSender::create_message() ]\t Front_left_motor \t= " << front_left_motor);
// #endif
  } else {
    mess[5] = 0x00;
    mess[6] = 0x00;
    mess[7] = 0x00;
    mess[8] = 0x00;
    mess[9] = 0x00;
    mess[10] = 0x00;
    mess[11] = 0x00;
    mess[12] = 0x00;
  }

  // Crc (Cyclic Redundancy Check) generation
  crc = crc::computeCrc2(mess);
  crc::setCrcInMessage(crc, mess); // mess[1], mess[2]

  incrementSequenceNumber_();
}

//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::createMessage_(
  std::vector<unsigned char> & mess,
  SentMessageId id)
{
  createMessage_(mess, id, 0, 0, 0, 0);
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::sendMessage_(
  SentMessageId id,
  double value1,
  double value2,
  double value3,
  double value4,
  serial::Serial & serial_port)
{

  std::vector<unsigned char> message(EFFIBOT_E3_MESSAGE_SIZE);

  // Create the message
  createMessage_(message, id, value1, value2, value3, value4);

  // Send the message
  serial_port.write(message);

// #ifndef NDEBUG
//   RCLCPP_INFO(logger_, "\n=====================================================================\n");
//   RCLCPP_INFO_STREAM(logger_, " -------------- Message sent: -------------- ");
//   RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(message));
// #endif
}

//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::sendMessage_(
  SentMessageId id,
  serial::Serial & serial_port)
{
  sendMessage_(id, 0, 0, 0, 0, serial_port);
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::sendSetSpeedCommand(
  double linear_speed,
  double angular_speed,
  serial::Serial & serial_port)
{
  // std::cout << ros::Time::now().toNSec() << " e3  " << linear_speed << " " << angular_speed <<
  //   " " << std::endl;
  sendMessage_(OUT_SET_SPEED_COMMAND, linear_speed, angular_speed, 0, 0, serial_port);
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::sendSetMotorCommand(
  double rear_right_motor,
  double front_right_motor,
  double rear_left_motor,
  double front_left_motor,
  serial::Serial & serial_port)
{
  // WARN : don't send this type of frame : can be harmful for the robot if data sent are not coherent!!
  sendMessage_(
    OUT_SET_MOTOR_COMMAND, rear_right_motor, front_right_motor, rear_left_motor,
    front_left_motor, serial_port);
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::sendRebootCommand(serial::Serial & serial_port)
{
  sendMessage_(OUT_REBOOT, serial_port);
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::sendSetActiveCommand(serial::Serial & serial_port)
{
  sendMessage_(OUT_SET_ACTIVE_MODE, serial_port);
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::sendSetInactiveCommand(serial::Serial & serial_port)
{
  sendMessage_(OUT_SET_INACTIVE_MODE, serial_port);
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorSender::sendHeartbeat(serial::Serial & serial_port)
{
  sendMessage_(OUT_HEARTBEAT, serial_port);
}

}
}
