/*
 * Contains the definitions of the functions of the class Effibote3Control
 *
 * Written by Luc Pain. June 2017.
 */


#include "effibote3_control.h"


//===========================================================================
//************* Effibote3Control Constructor ********************************
//===========================================================================
Effibote3Control::Effibote3Control(serial::Serial & serial)
: nh_(),
  private_nh_("~"),
  serial_port_(serial)
{
  // Publisher & Subscriber & Timer
  joy_sub_ = nh_.subscribe("joy_teleop/joy", 1, &Effibote3Control::joyCallback, this);
  cmd_vel_sub_ = nh_.subscribe(
    "vehicle_controller/cmd_vel", 1, &Effibote3Control::joyVelCallback,
    this);
  robot_is_active_sub_ = nh_.subscribe(
    "vehicle_controller/is_active", 1,
    &Effibote3Control::isActiveCallback, this);
  radar_pub_ = nh_.advertise<std_msgs::Bool>("radar/is_active", 1);
  reset_odom_pub_ = nh_.advertise<std_msgs::Bool>("vehicle_controller/reset_odometry", 1);
  data_sent_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("vehicle_controller/sent", 1);
  controller_timer_ = nh_.createTimer(
    ros::Duration(
      0.1), &Effibote3Control::controllerTimerCallback, this);                                                 // The first argument is the period (in seconds) of the Timer.


  front_side_ = 1; // initially, front side is buttons side

  is_speeds_command_available_ = false;
  is_send_null_speed_needed_ = false;
  last_command_sent_ = 0;

  is_robot_active_ = false;
  is_robot_active_became_true_ = false;
  is_robot_active_became_false_ = true;
  is_received_message_published_ = true;
  is_received_message_published_became_true_ = false;
  is_reinitialize_odometry_needed_ = true;

  communicator_sender_.displayRosInfo(false);

  if (front_side_ == 1) {
    ROS_INFO(
      "[Effibote3Control()] Front of the robot is the side where there are buttons (emergency stop...)!");
  } else if (front_side_ == -1) {
    ROS_INFO(
      "[Effibote3Control()] Front of the robot is the side where there are NO buttons (emergency stop...)!");
  } else {
    ROS_WARN(
      "[Effibote3Control()] The variable front_side_ is set to a different value than 1 or -1. Please check the code");
  }
}

//======================================

Effibote3Control::~Effibote3Control()
{
}


//===========================================================================
//************* Others getter/setter ****************************************
//===========================================================================
bool Effibote3Control::getIsReinitializeOdometryNeeded()
{
  return is_reinitialize_odometry_needed_;
}
//======================
void Effibote3Control::setIsReinitializeOdometryNeeded(bool value)
{
  is_reinitialize_odometry_needed_ = value;
}
//======================
bool Effibote3Control::getIsReceivedMessagePublished()
{
  return is_received_message_published_;
}
//======================
bool Effibote3Control::getIsReceivedMessagePublishedBecameTrue()
{
  return is_received_message_published_became_true_;
}
//======================

void Effibote3Control::setIsReceivedMessagePublishedBecameTrue(bool value)
{
  is_received_message_published_became_true_ = value;
}

//===========================================================================
//************* isActiveCallback ********************************************
//===========================================================================
/**
 * @brief isActiveCallback : send a SET_ACTIVE or SET_INACTIVE command whether a boolean true or false is published
 *                            on the /effibote3/is_active topic
 * @param is_active : boolean collected on the /effibote3/is_active topic
 */
void Effibote3Control::isActiveCallback(const std_msgs::Bool::ConstPtr & is_active)
{
  /// ======= Set robot to Active or Inactive =======

  // If 'false' published on topic /effibote3/is_active
  if (!is_active->data) {
    // If the robot is active
    if (is_robot_active_) {
      ROS_WARN(
        "Deactivation of the robot (Boolean 'false' published on topic /effibote3/is_active) !");
      // Prohibit control of the robot
      is_robot_active_ = false;
      // Send a null linear speed command to be sure wheels are stopped
      communicator_sender_.sendSetSpeedCommand(0.0, 0.0, serial_port_);
      // Set the robot inactive
      communicator_sender_.sendSetInactiveCommand(serial_port_);
      is_robot_active_became_false_ = true;
    } else {
      ROS_WARN(
        "[Boolean 'false' published on topic /effibote3/is_active but the robot is already inactive]");
    }
  }

  // If 'true' published on topic /effibote3/is_active
  if (is_active->data) {
    // If the robot is not active
    if (!is_robot_active_) {
      // Activate the robot
      ROS_INFO("Activation of the robot (Boolean 'true' published on topic /effibote3/is_active) !");
      communicator_sender_.sendSetActiveCommand(serial_port_);
      // Allow control of the robot
      is_robot_active_ = true;
      is_robot_active_became_true_ = true;
    } else {
      ROS_WARN(
        "[Boolean 'true' published on topic /effibote3/is_active but the robot is already active]");
    }
  }
}


//===========================================================================
//************* joyCallback *************************************************
//===========================================================================
/**
 * @brief joyCallback : modify the global variable is_robot_active_ and send a SetActive (or SetInactive) command
 *                      whether some specific buttons of the joystick are pressed
 */
void Effibote3Control::joyCallback(const sensor_msgs::Joy::ConstPtr & joy)
{
  /// ======= Reset odometry =======

  //=================================================
  // RESET ODOMETRY
  //=================================================
  // If 'RB' and 'LB' buttons and 'LT' and 'RT' axes are pressed
  if ((joy->buttons[4]) && (joy->buttons[5]) && (joy->axes[2] < -0.5) && (joy->axes[5] < -0.5)) {
    // Publish the value 'true' to the topic /effibote3/reset_odometry
    std_msgs::Bool reset_odom;
    reset_odom.data = true;
    reset_odom_pub_.publish(reset_odom);
    ROS_INFO("[ Odometry reset ! (LB + RB + LT + RT)]");
  }
  /// ======= Set robot to Active or Inactive =======
  // If 'A' button and 'LT' axe are pressed      OR 'B' button pressed
  else if ( (joy->buttons[0] && (joy->axes[2] < -0.5)) || joy->buttons[1]) {
    // If the robot is active
    if (is_robot_active_) {
      ROS_WARN("Deactivation of the robot ('A'+'LT' or 'B' pressed) !");

      // Prohibit control of the robot
      is_robot_active_ = false;
      // Set the robot inactive
      communicator_sender_.sendSetInactiveCommand(serial_port_);
      is_robot_active_became_false_ = true;

      ROS_WARN_STREAM(
        "The robot is not active!" << \
          "\n\t\t\t  - Press 'A'+'RT' to activate the robot -" << \
          "\n\t\t\t  - Press 'A'+'LT' or 'B' to DEactivate the robot -");
    } else {
      ROS_WARN("['A'+'LT' or 'B' pressed but the robot is already inactive]");
    }
  }
  // If 'A' button and 'RT' axe are pressed
  else if (joy->buttons[0] && (joy->axes[5] < -0.5)) {
    // If the robot is not active
    if (!is_robot_active_) {
      // Activate the robot
      ROS_INFO("Activation of the robot (buttons 'A' and 'RT' pressed) !");
      communicator_sender_.sendSetActiveCommand(serial_port_);
      // Allow control of the robot
      is_robot_active_ = true;
      is_robot_active_became_true_ = true;

      // Need to re initialize odometry to re-initialize the bias that has
      // probably changed since last time robot was active.
      is_reinitialize_odometry_needed_ = true;
    } else {
      ROS_WARN("[Buttons 'A' and 'RT' pressed but the robot is already active]");
    }
  }
  /// ======= Set the radar active or inactive =======
  // If 'X' button and 'LT' axe are pressed
  else if (joy->buttons[2] && (joy->axes[2] < -0.5)) {
    // Publish the value false to the topic /radar/is_active
    std_msgs::Bool radar_active;
    radar_active.data = false;
    radar_pub_.publish(radar_active);
    ROS_INFO("[ Radar deactivated ]");
  }
  // If 'X' button and 'RT' axe are pressed
  else if (joy->buttons[2] && (joy->axes[5] < -0.5)) {
    // Publish the value true to the topic /radar/is_active
    std_msgs::Bool radar_active;
    radar_active.data = true;
    radar_pub_.publish(radar_active);
    ROS_INFO("[ Radar activated ]");
  }


  /// ======= Set received messages pulished or not =======

  // If 'Y' button and 'LT' axe are pressed
  if (joy->buttons[3] && (joy->axes[2] < -0.5)) {
    if (is_received_message_published_) {
      // Set the global variable is_received_message_published_ to FALSE
      ROS_INFO("Messages are not published anymore (buttons 'Y' and 'LT' pressed) !");
      ROS_INFO("-> Received data are not published (press RT+Y to publish them).");
      is_received_message_published_ = false;
    } else {
      ROS_WARN("[Button 'Y' and 'LT' pressed but the messages are already NOT published]");
    }
  }

  // If 'Y' button and 'RT' axe are pressed
  if (joy->buttons[3] && (joy->axes[5] < -0.5)) {
    if (!is_received_message_published_) {
      // Set the global variable is_received_message_published_ to TRUE
      ROS_INFO("Messages are now published (buttons 'Y' and 'RT' pressed) !");
      is_received_message_published_ = true;
      is_received_message_published_became_true_ = true; // used for reInitialize communicator_receiver

      // Flush Input of serial port (because a lot of undatable data has been written
      // on serial port while is_received_message_published_ was false
      serial_port_.flushInput(); // Flush serial port one time
      // Re-flush it after a pack of frames to be written, in case we flushed while robot was writting a pack of frames
      serial_port_.waitReadable();
      serial_port_.waitByteTimes(EFFIBOT_E3_MESSAGE_SIZE * 13);
      serial_port_.flush();
    } else {
      ROS_WARN("[Button 'Y' and 'RT' pressed but the messages are already published]");
    }
  }

}


//===========================================================================
//************* joyVelCallback **********************************************
//===========================================================================
/**
 * @brief joyVelCallback : send a OUT_SET_SPEED_COMMAND if joystick sends messages
 * @param twist_joy : speeds collected on the /cmd_vel topic
 */
void Effibote3Control::joyVelCallback(const geometry_msgs::Twist::ConstPtr & twist_joy)
{
  if (is_robot_active_) {
    if (is_speeds_command_available_ == false) {
      // Define speeds to send (that will be sent by the controllerTimerCallback)
      speeds_.twist.linear.x = (-front_side_) * (twist_joy->linear.x);
      speeds_.twist.angular.z = twist_joy->angular.z; // (Rem: angular.z is Yaw axis)

      // Check if speed is in good range and modify its value if necessary
      // It is also check in the effibote3_communicator_sender.cpp file but I do it here in order
      // to publish on the /effibote3/control/sent topic the true speeds sent (in case there is a out of range value)
//      if      (speeds_.twist.linear.x >  MAX_LINEAR_SPEED_M_S)       speeds_.twist.linear.x =  MAX_LINEAR_SPEED_M_S;
//      else if (speeds_.twist.linear.x < -MAX_LINEAR_SPEED_M_S)       speeds_.twist.linear.x = -MAX_LINEAR_SPEED_M_S;
//      if      (speeds_.twist.angular.z >  MAX_ANGULAR_SPEED_RAD_S)   speeds_.twist.angular.z =  MAX_ANGULAR_SPEED_RAD_S;
//      else if (speeds_.twist.angular.z < -MAX_ANGULAR_SPEED_RAD_S)   speeds_.twist.angular.z = -MAX_ANGULAR_SPEED_RAD_S;

      // Tells to the timer that speeds are available
      is_speeds_command_available_ = true;
    }
  }
}


//===========================================================================
//************* controllerTimerCallback *************************************
//===========================================================================
void Effibote3Control::controllerTimerCallback(const ros::TimerEvent & /*event*/)
{
  int command_sent = 0;

  if (is_robot_active_) {
    // If the robot was not active and if we just activate it
    if (is_robot_active_became_true_) {
      communicator_sender_.sendSetActiveCommand(serial_port_);
      ROS_INFO("The robot is active!");
      is_robot_active_became_true_ = false;
    } else {
      // If last command sent is a Heartbeat, and before last command sent is a Set_Speed
      // Otherwise, the robot may keep a no null speed in memory and continue to move at this speed while we are sending heartbeats.
      if (is_send_null_speed_needed_) {
        // Send a null Speed_command message
        communicator_sender_.sendSetSpeedCommand(0, 0, serial_port_);

        // Publish data on the effibote3/control/sent topic
        speeds_.header.stamp = ros::Time::now();
        speeds_.twist.linear.x = 0;
        speeds_.twist.angular.z = 0;
        data_sent_pub_.publish(speeds_);

        is_send_null_speed_needed_ = false;
      } else if (is_speeds_command_available_) {
        // Send a Speed_command message
        communicator_sender_.sendSetSpeedCommand(
          speeds_.twist.linear.x, speeds_.twist.angular.z,
          serial_port_);
        speeds_.header.stamp = ros::Time::now();
        command_sent = 1;

        // Publish data on the effibote3/control/sent topic
        // Need to change sign of linear speed if front_side_=1
        speeds_.twist.linear.x = (-front_side_) * speeds_.twist.linear.x;
        data_sent_pub_.publish(speeds_);

        // Tell to joyVelCallback to fill speeds_ with new values
        is_speeds_command_available_ = false;
      } else {
        // Send a Heartbeat message
        communicator_sender_.sendHeartbeat(serial_port_);

        // If the previous sent command is a Speed command
        if (last_command_sent_ == 1) {
          // The next call of the timer will send a null speed
          is_send_null_speed_needed_ = true;
        }
      }
    }

    last_command_sent_ = command_sent;
  }
}
//====================================================================================================
//******************************************************************************** end Effibote3Control class
//====================================================================================================
