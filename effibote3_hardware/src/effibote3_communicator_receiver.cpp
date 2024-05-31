#include "effibote3_hardware/effibote3_communicator_receiver.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
Effibote3CommunicatorReceiver::Effibote3CommunicatorReceiver()
: logger_(rclcpp::get_logger("effibote3_hardware"))
{
  registered_data_counter_ = 0;
  number_of_interesting_data_type_ = 7;
  is_incomplete_frame_ = false;

}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::reInitialize()
{
  // Re-initialize variables
  registered_data_counter_ = 0;
  is_incomplete_frame_ = false;
  incomplete_frame_str_.clear();
}


//-----------------------------------------------------------------------------
bool Effibote3CommunicatorReceiver::isIncompleteFrame()
{
  return is_incomplete_frame_;
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::processMessage(
  std::string & serial_port_str)
{

  buffer_ += serial_port_str;
  // RCLCPP_ERROR_STREAM(logger_, "\t\t" << convertMessageToHexa(buffer_));
  for (;; ) {
    auto pos = buffer_.find_first_of(EFFIBOT_E3_MESSAGE_HEADER);
    if (pos != std::string::npos) {
      buffer_.erase(0, pos);
    } else {
      break;
    }

    pos = buffer_.find_first_of(EFFIBOT_E3_MESSAGE_HEADER, 1);
    if (pos != std::string::npos) {
      if (pos == EFFIBOT_E3_MESSAGE_SIZE) {
        auto data = std::vector<unsigned char>(
          buffer_.begin(),
          buffer_.begin() + EFFIBOT_E3_MESSAGE_SIZE);

        // RCLCPP_ERROR_STREAM(
        //   logger_, "\t\t" << convertMessageToHexa(buffer_.substr(0, pos)));

        processFrame_(data);
      }
      buffer_.erase(0, pos);
    } else {
      break;
    }
  }

//   ///=================== if incomplete frame ========================================
//   if (is_incomplete_frame_) {
//     // Fill the incomplete frame
//     incomplete_frame_str_ += serial_port_str;

//     // If we now have a pack of data (at least number_of_interesting_data_type_ frames which length is EFFIBOT_E3_MESSAGE_SIZE)
//     if (incomplete_frame_str_.size() >=
//       number_of_interesting_data_type_ * EFFIBOT_E3_MESSAGE_SIZE)
//     {
//       // Decode the data and publish them
//       decodeFullPackOfData_(incomplete_frame_str_);

//       // Re-initialize variables
//       is_incomplete_frame_ = false;
//       incomplete_frame_str_.clear();
//     }
//     // Else nothing : go to read the serial port to complete the incomplete_frame_str_
//   }
//   // Else if there is at least one frame IN_SET_... at the beginning
//   else {
//     ///=================== if there are IN_SET_... frame at the beginning =============
//     unsigned char id; // id of the first frame
//     id = (unsigned char)serial_port_str[EFFIBOT_E3_ID_OFFSET];

//     // While there is another frame than IN_ANGULAR_SPEED frame at the beginning
//     while ( (((unsigned char)serial_port_str[0]) == EFFIBOT_E3_MESSAGE_HEADER) &&
//       (id == IN_SET_ACTIVE_MODE || id == IN_SET_INACTIVE_MODE || id == IN_SET_OUTPUTS ||
//       id == IN_SET_SPEED_COMMAND) )
//     {
//       // Delete the first frame of the string (or part of frame if we reach the last frame and if this one is incomplete)
//       serial_port_str.erase(0, std::min((int)EFFIBOT_E3_MESSAGE_SIZE, (int)serial_port_str.size()));
//       // Re-compute id
//       if (serial_port_str.size() > EFFIBOT_E3_ID_OFFSET) {
//         id = (unsigned char)serial_port_str[EFFIBOT_E3_ID_OFFSET];
//       }
//     }

//     // If there are still some data in the message
//     if (!(serial_port_str.empty()) ) {
//       ///=================== if the first frame is an angular speed reply ===============
//       // If the first frame is well an Angular_speed_reply
//       // (because the robot sends a pack of data at 50Hz,
//       //  and the first frame of this pack is an Angular_speed_reply frame)
//       if ( (((unsigned char)serial_port_str[0]) == EFFIBOT_E3_MESSAGE_HEADER) &&
//         (((unsigned char)serial_port_str[EFFIBOT_E3_ID_OFFSET]) == IN_ANGULAR_SPEEDS) )
//       {
//         // If the serial port contains a full pack of data (at least number_of_interesting_data_type_ frames)
//         if (serial_port_str.size() >= number_of_interesting_data_type_ * EFFIBOT_E3_MESSAGE_SIZE) {
//           // Decode the data and pulish them
//           decodeFullPackOfData_(serial_port_str);
//         } else { // if the serial port contains not a full pack of data (but just the beginning)
//           // Save the message in the variable incomplete_frame_str_
//           incomplete_frame_str_.clear(); // (just for secure, normally it's done above)
//           incomplete_frame_str_ += serial_port_str;

//           is_incomplete_frame_ = true;
//           RCLCPP_INFO(
//             logger_,
//             "[Effibote3CommunicatorReceiver::processMessage] There is an incomplete frame!");
//         }
//       } else {
//         RCLCPP_WARN(
//           logger_,
//           "[Effibote3CommunicatorReceiver::processMessage] There is no incomplete frame and the first frame is not an angular speed frame!");
//         reInitialize();
//       }
//     }
//     // Else : the serial port did not get relevant data (only reply frames.)
//     else {
// // #ifndef NDEBUG
// //       RCLCPP_WARN(
// //         logger_,
// //         "[Effibote3CommunicatorReceiver::processMessage] The serial port did not get relevant data :");
// //       RCLCPP_WARN_STREAM(logger_, "\t\t" << convertMessageToHexa(serial_port_str));
// // #endif
//     }
//   }
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::decodeFullPackOfData_(std::string & message_to_decode)
{
  // // Check that the first frame id is IN_ANGULAR_SPEEDS
  // if (((unsigned char)message_to_decode[EFFIBOT_E3_ID_OFFSET]) == IN_ANGULAR_SPEEDS) {

  //   // While all data are not registered in member variables, process message frame by frame
  //   int i = 0;
  //   while (registered_data_counter_ < number_of_interesting_data_type_) {
  //     std::vector<unsigned char> frame_to_decode(message_to_decode.begin() + i,
  //       message_to_decode.begin() + i + EFFIBOT_E3_MESSAGE_SIZE);
  //     processFrame_(frame_to_decode);
  //     // Move index to the next frame
  //     i += EFFIBOT_E3_MESSAGE_SIZE;
  //   }
  // } else {
  //   RCLCPP_WARN(
  //     logger_,
  //     "[e3CommunicatorReceiver::decodeFullPackOfData_] First frame id is not IN_ANGULAR_SPEEDS");
  //   RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(message_to_decode));
  // }
}


//-----------------------------------------------------------------------------
unsigned char Effibote3CommunicatorReceiver::processFrame_(std::vector<unsigned char> & frame)
{


  // if (frame[0] == EFFIBOT_E3_MESSAGE_HEADER) {
  // ----Check the crc
  // --TODO--

  // Get the id of the frame
  unsigned char id = frame[EFFIBOT_E3_ID_OFFSET];
  // RCLCPP_WARN_STREAM(logger_, "\t\t id" << id);

  // Process message
  switch (id) {
    // case IN_SET_SPEED_COMMAND:
    //   receiveSetSpeedCommandReplyMessage_(frame);
    //   break;

    case IN_SET_ACTIVE_MODE:
      receiveActiveModeReplyMessage_(frame);
      break;

    case IN_SET_INACTIVE_MODE:
      receiveInactiveModeReplyMessage_(frame);
      break;

    case IN_RIGHT_WHEEL_SPEED:
      receiveRightWheelSpeedMessage_(frame);
      break;

    case IN_LEFT_WHEEL_SPEED:
      receiveLeftWheelSpeedMessage_(frame);
      break;

    case IN_RIGHT_MOTOR_CURRENT:
      receiveRightMotorCurrentMessage_(frame);
      break;

    case IN_LEFT_MOTOR_CURRENT:
      receiveLeftMotorCurrentMessage_(frame);
      break;

    case IN_STATUS:
      receiveStatusMessage_(frame);
      break;

    case IN_SET_OUTPUTS:
      receiveSetupOutputsReplyMessage_(frame);
      break;

    case IN_INVALID_MESSAGE_ERROR:
      receiveInvalidMessageErrorMessage_(frame);
      break;

    case IN_ANGULAR_SPEEDS:
      receiveAngularSpeedsReplyMessage_(frame);
      break;

    case IN_ACCELERATIONS:
      receiveAccelerationsReplyMessage_(frame);
      break;

    default:
      // RCLCPP_WARN(logger_, "[From EffibotE3] Received frame with unknown id: ");
      // RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
      break;
  }
  return id;
  // } else {
  //   RCLCPP_WARN(
  //     logger_,
  //     "[From EffibotE3] Received frame has a wrong Message Header (frame[0]): ");
  //   RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
  //   return 0;
  // }
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveSetSpeedCommandReplyMessage_(
  std::vector<unsigned char> & frame)
{
  #ifndef NDEBUG
  RCLCPP_INFO_STREAM(rclcpp::get_logger("debug"), "[From EffibotE3] Speed Command Reply :");
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("debug"), "\t\t" << convertMessageToHexa(frame));
  #endif

  // do not process ack of speed command because there is no valuable
  // content
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveActiveModeReplyMessage_(
  std::vector<unsigned char> & frame)
{
  #ifndef NDEBUG
  RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Active Mode Reply :");
  // RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
  #endif
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveInactiveModeReplyMessage_(
  std::vector<unsigned char> & frame)
{
  #ifndef NDEBUG
  RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Inactive Mode Reply :");
  // RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
  #endif
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveRightWheelSpeedMessage_(
  std::vector<unsigned char> & frame)
{
  // Conversion FROM std::vector<unsigned char> TO int32_t
  // The ..._raw values are in mm.s-1
  int32_t rearRightSpeed_raw =
    (int32_t) ((frame[5] << 24) | (frame[6] << 16) | (frame[7] << 8) | frame[8]);
  int32_t frontRightSpeed_raw =
    (int32_t) ((frame[9] << 24) | (frame[10] << 16) | (frame[11] << 8) | frame[12]);

  // Conversion FROM mm.s-1 TO m.s-1
  right_wheel_speed_rear_ = ((float)(rearRightSpeed_raw)) * EFFIBOT_E3_WHEEL_SPEED_RESOLUTION;
  right_wheel_speed_front_ = ((float)(frontRightSpeed_raw)) * EFFIBOT_E3_WHEEL_SPEED_RESOLUTION;

  registered_data_counter_ += 1;

#ifndef NDEBUG
  RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Right Wheel Speed :");
  RCLCPP_INFO_STREAM(logger_, "\t\t\t rearRightSpeed  (m/s) = " << right_wheel_speed_rear_);
  RCLCPP_INFO_STREAM(logger_, "\t\t\t frontRightSpeed (m/s) = " << right_wheel_speed_front_);
#endif
}

//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveLeftWheelSpeedMessage_(
  std::vector<unsigned char> & frame)
{
  // Conversion FROM std::vector<unsigned char> TO int32_t
  // The ..._raw values are in mm.s-1
  int32_t rearLeftSpeed_raw =
    (int32_t) ((frame[5] << 24) | (frame[6] << 16) | (frame[7] << 8) | frame[8]);
  int32_t frontLeftSpeed_raw =
    (int32_t) ((frame[9] << 24) | (frame[10] << 16) | (frame[11] << 8) | frame[12]);

  // Conversion FROM mm.s-1 TO m.s-1
  left_wheel_speed_rear_ = ((float)(rearLeftSpeed_raw)) * EFFIBOT_E3_WHEEL_SPEED_RESOLUTION;
  left_wheel_speed_front_ = ((float)(frontLeftSpeed_raw)) * EFFIBOT_E3_WHEEL_SPEED_RESOLUTION;

  registered_data_counter_ += 1;

  #ifndef NDEBUG
  RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Left Wheel Speed :");
  RCLCPP_INFO_STREAM(logger_, "\t\t\t rearLeftSpeed  (m/s) = " << left_wheel_speed_rear_);
  RCLCPP_INFO_STREAM(logger_, "\t\t\t frontLeftSpeed (m/s) = " << left_wheel_speed_front_);
  #endif
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveRightMotorCurrentMessage_(
  std::vector<unsigned char> & frame)
{
  // Conversion FROM std::vector<unsigned char> TO u_int32_t
  // The ..._raw values are in milliampere
  u_int32_t rearRightMotorCurrent_raw =
    (u_int32_t) ((frame[5] << 24) | (frame[6] << 16) | (frame[7] << 8) | frame[8]);
  u_int32_t frontRightMotorCurrent_raw =
    (u_int32_t) ((frame[9] << 24) | (frame[10] << 16) | (frame[11] << 8) | frame[12]);

  // Conversion FROM milliampere TO ampere
  right_motor_current_rear_ = ((float)(rearRightMotorCurrent_raw)) *
    EFFIBOT_E3_MOTOR_CURRENT_RESOLUTION;
  right_motor_current_front_ = ((float)(frontRightMotorCurrent_raw)) *
    EFFIBOT_E3_MOTOR_CURRENT_RESOLUTION;

  registered_data_counter_ += 1;

  // #ifndef NDEBUG
  // RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Right Motor Current :");
  // // RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
  // RCLCPP_INFO_STREAM(
  //   logger_,
  //   "\t\t\t rearRightMotorCurrent  (A) = " << right_motor_current_rear_);
  // RCLCPP_INFO_STREAM(
  //   logger_,
  //   "\t\t\t frontRightMotorCurrent (A) = " << right_motor_current_front_);
  // #endif
}

//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveLeftMotorCurrentMessage_(
  std::vector<unsigned char> & frame)
{
  // Conversion FROM std::vector<unsigned char> TO u_int32_t
  // The ..._raw values are in milliampere
  u_int32_t rearLeftMotorCurrent_raw =
    (u_int32_t) ((frame[5] << 24) | (frame[6] << 16) | (frame[7] << 8) | frame[8]);
  u_int32_t frontLeftMotorCurrent_raw =
    (u_int32_t) ((frame[9] << 24) | (frame[10] << 16) | (frame[11] << 8) | frame[12]);

  // Conversion FROM milliampere TO ampere
  left_motor_current_rear_ = ((float)(rearLeftMotorCurrent_raw)) *
    EFFIBOT_E3_MOTOR_CURRENT_RESOLUTION;
  left_motor_current_front_ = ((float)(frontLeftMotorCurrent_raw)) *
    EFFIBOT_E3_MOTOR_CURRENT_RESOLUTION;

  registered_data_counter_ += 1;

  //  #ifndef NDEBUG
  // RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Left Motor Current :");
  // RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t rearLeftMotorCurrent  (A) = " << left_motor_current_rear_);
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t frontLeftMotorCurrent (A) = " << left_motor_current_front_);
  // #endif
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveStatusMessage_(std::vector<unsigned char> & frame)
{
  // (NOTE : frame[11] and frame[12] are unused)

  //===== Temperature =========
  temperature_ = (double) (frame[5]);

  registered_data_counter_ += 1;

  //===== Battery voltage =====
  // Linear interpolation between min and max battery voltage
  double batteryVoltage = ((u_int8_t) frame[6]) * EFFIBOT_E3_DIFFERENCE_BATTERY_VOLTAGE / 255.0 +
    EFFIBOT_E3_MIN_BATTERY_VOLTAGE;

  //===== Input states ========
  typedef struct InputStates
  {
    bool cn12 : 1;
    bool cn13 : 1;
    bool cn14 : 1;
    bool cn15 : 1;
    bool cn16 : 1;
  } inputStates;
  inputStates input_states;
  const std::bitset<8> inputStates_raw(((u_int8_t) frame[7]));
  input_states.cn12 = inputStates_raw[0];
  input_states.cn13 = inputStates_raw[1];
  input_states.cn14 = inputStates_raw[2];
  input_states.cn15 = inputStates_raw[3];
  input_states.cn16 = inputStates_raw[4];

  //===== Error ===============
  // (from effibox): enum ErrorFlag  {    ERROR_FLAG_IMU = 0x01,    ERROR_FLAG_SLAVE = 0x02,    ERROR_FLAG_HARDWARE_FAULT = 0x04,    ERROR_FLAG_COMMUNICATION_TIMEOUT = 0x08,    ERROR_FLAG_COMMUNICATION_SEND_OVERFLOW = 0x010  };
  u_int8_t error = ((u_int8_t) frame[8]);

  //===== State ===============
  // (from effibox): enum State{    STATE_EMERGENCY = 0,    STATE_ACTIVE = 1,    STATE_UNKNOWN  };
  int state = (u_int8_t) frame[9];

  //===== Output states =======
  typedef struct OutputStates
  {
    bool cn7 : 1;
    bool cn8 : 1;
    bool cn9 : 1;
    bool cn10 : 1;
    bool cn11 : 1;
  } outputStates;
  outputStates output_states;
  const std::bitset<8> outputStates_raw(((u_int8_t) frame[10]));
  output_states.cn7 = outputStates_raw[0];
  output_states.cn8 = outputStates_raw[1];
  output_states.cn9 = outputStates_raw[2];
  output_states.cn10 = outputStates_raw[3];
  output_states.cn11 = outputStates_raw[4];


  // #ifndef NDEBUG
  // RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Status Message :");
  // RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t Temperature (degCelcius) = " << temperature_);
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t Battery voltage (Volts) = " << batteryVoltage);
  // RCLCPP_INFO_STREAM(
  //   logger_,
  //   "\t\t\t {Input states} cn12 = " <<
  //     input_states.cn12 << "; cn13 = " <<
  //     input_states.cn13 << "; cn14 = " <<
  //     input_states.cn14 << "; cn15 = " <<
  //     input_states.cn15 << "; cn16 = " <<
  //     input_states.cn16);
  // switch (error) {
  //   case 0x01:
  //     RCLCPP_INFO_STREAM(logger_, "\t\t\t ErrorFlag = ERROR_FLAG_IMU");
  //     break;
  //   case 0x02:
  //     RCLCPP_INFO_STREAM(logger_, "\t\t\t ErrorFlag = ERROR_FLAG_SLAVE");
  //     break;
  //   case 0x04:
  //     RCLCPP_INFO_STREAM(logger_, "\t\t\t ErrorFlag = ERROR_FLAG_HARDWARE_FAULT");
  //     break;
  //   case 0x08:
  //     RCLCPP_INFO_STREAM(logger_, "\t\t\t ErrorFlag = ERROR_FLAG_COMMUNICATION_TIMEOUT");
  //     break;
  //   case 0x010:
  //     RCLCPP_INFO_STREAM(logger_, "\t\t\t ErrorFlag = ERROR_FLAG_COMMUNICATION_SEND_OVERFLOW");
  //     break;
  //   default:
  //     RCLCPP_INFO_STREAM(logger_, "\t\t\t WARN : unknown ErrorFlag");
  //     break;
  // }
  // switch (state) {
  //   case 0:
  //     RCLCPP_INFO_STREAM(logger_, "\t\t\t State = STATE_EMERGENCY");
  //     break;
  //   case 1:
  //     RCLCPP_INFO_STREAM(logger_, "\t\t\t State = STATE_ACTIVE");
  //     break;
  //   default:
  //     RCLCPP_INFO_STREAM(logger_, "\t\t\t State = STATE_UNKNOWN");
  //     break;
  // }
  // RCLCPP_INFO_STREAM(
  //   logger_,
  //   "\t\t\t {Output states} cn7 = " <<
  //     output_states.cn7 << "; cn8 = " <<
  //     output_states.cn8 << "; cn9 = " <<
  //     output_states.cn9 << "; cn10 = " <<
  //     output_states.cn10 << "; cn11 = " <<
  //     output_states.cn11);
  // #endif
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveSetupOutputsReplyMessage_(
  std::vector<unsigned char> & frame)
{
  // #ifndef NDEBUG
  // RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Setup Outputs Reply :");
  // RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
  // #endif
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveInvalidMessageErrorMessage_(
  std::vector<unsigned char> & frame)
{
  // RCLCPP_WARN_STREAM(logger_, "[From EffibotE3] Invalid Message Error :");
  // RCLCPP_WARN_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
  // RCLCPP_WARN_STREAM(
  //   logger_,
  //   "[From EffibotE3] The vehicle says it received an invalid frame with id " <<
  //     " (0x" << std::hex << (int)(frame[EFFIBOT_E3_ID_OFFSET]) << ")");
}


//===========================================================================
//************* Angular Speeds reply ****************************************
//===========================================================================
void Effibote3CommunicatorReceiver::receiveAngularSpeedsReplyMessage_(
  std::vector<unsigned char> & frame)
{
  // (NOTE: frame[11] and frame[12] are unused)

  // Conversion FROM std::vector<unsigned char> TO int16_t
  // The ..._raw values are in millidegrees/s
  int16_t const ang_speed_x_raw = (int16_t) ((frame[5] << 8) | frame[6]);   // Roll
  int16_t const ang_speed_y_raw = (int16_t) ((frame[7] << 8) | frame[8]);   // Yaw
  int16_t const ang_speed_z_raw = (int16_t) ((frame[9] << 8) | frame[10]);   // Pitch

  // Conversion TO dps (=degrees/s) (scale is 8.75 mdps/digit)
  float const ang_speed_x_dps = float(ang_speed_x_raw) * 0.00875f;
  float const ang_speed_y_dps = float(ang_speed_y_raw) * 0.00875f;
  float const ang_speed_z_dps = float(ang_speed_z_raw) * 0.00875f;

  // Conversion FROM degrees/s TO radian/s
  float const angular_speed_x_rad = ang_speed_x_dps * (M_PI / 180.);
  float const angular_speed_y_rad = ang_speed_y_dps * (M_PI / 180.);
  float const angular_speed_z_rad = ang_speed_z_dps * (M_PI / 180.);

  // Axes conversion : depending on the front side of the robot
  // front side = buttons (emergency stop...) side => front_size_= +1
  // In Gazebo :  .x = Front = Roll  ;  .y = Left = Pitch  ;  .z = Up = Yaw
  angular_speed_x_ = static_cast<double>(angular_speed_x_rad);
  angular_speed_y_ = static_cast<double>(angular_speed_z_rad);
  angular_speed_z_ = static_cast<double>( (-1.0) * angular_speed_y_rad );

  registered_data_counter_ += 1;

  // #ifndef NDEBUG
  // RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Angular Speeds :");
  // // RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t ang_speed_x (=Roll) (rad/s) = " << angular_speed_x_);
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t ang_speed_y (=Pitch)(rad/s) = " << angular_speed_y_);
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t ang_speed_z (=Yaw)  (rad/s) = " << angular_speed_z_);
  // #endif
}


//-----------------------------------------------------------------------------
void Effibote3CommunicatorReceiver::receiveAccelerationsReplyMessage_(
  std::vector<unsigned char> & frame)
{
  // (NOTE: frame[11] and frame[12] are unused)

  // Conversion FROM std::vector<unsigned char> TO int16_t
  // The ..._raw values are in mm.s-2 (probably because we there is a factor  8.192 that I don't know why.
  int16_t accel_x_raw = (int16_t) (((int)frame[5] << 8) | frame[6]);
  int16_t accel_y_raw = (int16_t) (((int)frame[7] << 8) | frame[8]);
  int16_t accel_z_raw = (int16_t) (((int)frame[9] << 8) | frame[10]);

  // Conversion TO m.s-2  (conversion found in effibox code)
  float const acceleration_x_ms = float(accel_x_raw) / 8192.0f;
  float const acceleration_y_ms = float(accel_y_raw) / 8192.0f;
  float const acceleration_z_ms = float(accel_z_raw) / 8192.0f;

  // Axes conversion : depending on the front side of the robot
  // In Gazebo :  .x = Front  ;  .y = Left  ;  .z = Up
  acceleration_x_ = static_cast<double>( acceleration_x_ms );
  acceleration_y_ = static_cast<double>( acceleration_y_ms );
  acceleration_z_ = static_cast<double>( acceleration_z_ms );

  registered_data_counter_ += 1;

  // #ifndef NDEBUG
  // RCLCPP_INFO_STREAM(logger_, "[From EffibotE3] Acceleration :");
  // // RCLCPP_INFO_STREAM(logger_, "\t\t" << convertMessageToHexa(frame));
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t accel_x (m.s-2) = " << acceleration_x_);
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t accel_y (m.s-2) = " << acceleration_y_);
  // RCLCPP_INFO_STREAM(logger_, "\t\t\t accel_z (m.s-2) = " << acceleration_z_);
  // #endif
}

}
}
