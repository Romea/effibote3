#ifndef EFFIBOTE3_CONTROL_EFFIBOTE3_CONTROL_H
#define EFFIBOTE3_CONTROL_EFFIBOTE3_CONTROL_H

/*
 * Contains the declaration of the functions of the class Effibote3Control
 *
 * Written by Luc Pain. June 2017.
 */


#include <ros/ros.h>
#include <serial/serial.h>              // serial_port
#include <std_msgs/Bool.h>              // radar/is_active
#include <geometry_msgs/Twist.h>        // joyVelCallback
#include <sensor_msgs/Joy.h>            // joyCallback
#include <geometry_msgs/TwistStamped.h> // speeds_

//local
#include "effibote3_communicator_sender.h"
#include "effibote3_param.h"            // EFFIBOTE3_MESSAGE_SIZE
#include "effibote3_display.h"


//====================================================================================================
//************* E3Control class declaration **********************************************************
//====================================================================================================
class Effibote3Control
{

public:
  Effibote3Control(serial::Serial & serial);
  ~Effibote3Control();


  bool getIsReinitializeOdometryNeeded();
  void setIsReinitializeOdometryNeeded(bool value);
  bool getIsReceivedMessagePublished();
  bool getIsReceivedMessagePublishedBecameTrue();
  void setIsReceivedMessagePublishedBecameTrue(bool value);

private:
  /**
   * @brief joyCallback : modify the global variable g_is_robot_active and send a SetActive (or SetInactive) command
   *                      whether some specific buttons of the joystick are pressed
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr & joy);
  /**
   * @brief isActiveCallback : send a SET_ACTIVE or SET_INACTIVE command whether a boolean true or false is published
   *                            on the /effibote3/is_active topic
   * @param is_active : boolean collected on the /effibote3/is_active topic
   */
  void isActiveCallback(const std_msgs::Bool::ConstPtr & is_active);
  /**
   * @brief joyVelCallback : define speeds to send from /cmd_vel topic values
   * @param twist_joy : speeds collected on the /cmd_vel topic
   */
  void joyVelCallback(const geometry_msgs::Twist::ConstPtr & twist_joy);
  /**
   * @brief Timer that sends control commands to the robot
   */
  void controllerTimerCallback(const ros::TimerEvent & event);


  //==================================================================
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber robot_is_active_sub_;
  ros::Publisher radar_pub_;
  ros::Publisher reset_odom_pub_;
  ros::Publisher data_sent_pub_;
  ros::Timer controller_timer_;

  Effibote3CommunicatorSender communicator_sender_;

  //==================================================================
  int front_side_;    /*  Set the side (front, back) of the robot;
                       *  if buttons (emergency stop ...) side = front then front_side_=(+1)
                       *  otherwise front_size_=(-1).
                       *  WARN : You must change this in this class via setFrontSide()
                       *           AND also in the Effibote3CommunicatorReceiver class
                       *           from (effibote3_communicator_receiver.cpp) via Effibote3CommunicatorReceiver::setFrontSide()
                       *      You must do changes in the 'main' function thanks to .setFrontSide() methods
                       */

  // Member variables for managing the sending of commands
  geometry_msgs::TwistStamped speeds_;
  bool is_speeds_command_available_;
  bool is_send_null_speed_needed_;
  int last_command_sent_;   // = 1 (if command sent is Speeds command), else = 0

  // Boolean
  bool is_robot_active_;
  bool is_robot_active_became_true_;
  bool is_robot_active_became_false_;
  bool is_received_message_published_;
  bool is_received_message_published_became_true_;
  bool is_reinitialize_odometry_needed_;

  serial::Serial & serial_port_;
};


#endif
