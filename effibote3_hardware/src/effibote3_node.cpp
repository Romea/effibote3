/*
 * File that receive messages (odometry, send speed reply...) from the robot via serial port
 * and that call classes to send commands to the robot (via serial port).
 *
 * Written by Luc Pain. June 2017.
 */


#include <ros/ros.h>
#include <serial/serial.h>        // serial_port
#include <std_msgs/Bool.h>        // radar/is_active
#include <geometry_msgs/Twist.h>  // joyVelCallback
#include <sensor_msgs/Joy.h>      // joyCallback
#include <geometry_msgs/TwistStamped.h> //data_sent_pub_
//std
#include <vector>
#include <string>
#include <stdint.h>
#include <iostream>
#include <ostream>
#include <sstream>
//local
#include "effibote3_control.h"
#include "effibote3_communicator_sender.h"
#include "effibote3_communicator_receiver.h"
#include "effibote3_param.h"
#include "effibote3_display.h"

#define SLEEP_TIME_RECEIVING_NO_MESSAGE 100000 // in micro seconds, must be lower than 100000 micro seconds.
                                               // = Time to sleep at the end of the loop in the main function, if the receiving of messages
                                               // from the robot is deactivated.


//===========================================================================
//************* main ********************************************************
//===========================================================================
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "effibote3_node");

  ROS_WARN_STREAM(
    "The robot is not active!" << \
      "\n\t\t\t  - Press 'A'+'RT' to activate the robot -" << \
      "\n\t\t\t  - Press 'A'+'LT' or 'B' to DEactivate the robot -");


  /// ===================== Open the serial port =====================
  //WARN : if there are problems, maybe the timeout should be shorter than the frequency
  //    of the controllerTimerCallback from Effibote3Control class.
  //    Because joyVelCallback needs spinOnce() to set speeds_ sent by the timer
  //    and  ser.waitReadable() blocks the programm until data are received or until the timeout elapse.
  serial::Serial ser;
  try {
    ser.setPort("/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // Timeout = 1s
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException & e) {
    ROS_ERROR_STREAM("Unable to open port ");
    std::cerr << e.what() << std::endl;
    return -1;
  }
  if (ser.isOpen()) {
    ROS_INFO_STREAM("Serial Port initialized");
  } else {
    return -1;
  }


  /// ================= Call classes =================================

  // Class that manages joystick and timer to send command.
  // Need to be instanciated after ros::init because it contains a publisher.
  Effibote3Control e3_control(ser);

  // Class to receive messages from e3.
  // Need to be instanciated after ros::init because it contains a publisher.
  Effibote3CommunicatorReceiver communicator_receiver;

  ///~~~~~~~~~~~~ Values you can modify ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~///
  // Display or not the ROS messages for debugging.
  communicator_receiver.displayRosInfo(false); // value : false or true - to display ROS messages from communicator_receiver.
  e3_control.displayRosInfo(false); // value : false or true - to display ROS messages from communicator_sender used by Effibote3Control class.
  bool display_ros_info_main = false; // value : false or true - to display ROS messages from this function (main()).


  // Set the front side. Initially, the front side is the buttons side.
  int front_side = 1; // = 1 if front is buttons side; else = (-1)
  ///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~///
  e3_control.setFrontSide(front_side);
  communicator_receiver.setFrontSide(front_side);


  /// ================= Receive messages and spinOnce ================

  // Get the date of beginning of the program (in seconds).
  // (Use it if you want to substract this to the date of data recovery.)
  double beginning_date = ros::Time::now().toSec();

  while (ros::ok()) {
    // If joystick posted a message on /joy topic, execute joyCallback
    // i.e. check for incoming messages
    ros::spinOnce();

    // If odometry values need to be re-initialized
    if (e3_control.getIsReinitializeOdometryNeeded()) {
      // Re-initialize odometry values
      communicator_receiver.reInitializeOdometry();
      e3_control.setIsReinitializeOdometryNeeded(false);
    }

    // If we start again receiving message from the robot (after having stopped or at the beginning of the programm)
    if (e3_control.getIsReceivedMessagePublishedBecameTrue()) {
      // Re-initialize member variables of communicator_receiver (for example, in case there was an incomplete frame
      communicator_receiver.reInitialize();

      e3_control.setIsReceivedMessagePublishedBecameTrue(false);
    }

    // If we authorize the reading of messages coming from E3 (via serial port)
    if (e3_control.getIsReceivedMessagePublished()) {
      if (display_ros_info_main) {
        ROS_INFO("\n=====================================================================\n");
      }

      // Wait for a byte to be read on serial port or for the timeout to be elapsed :
      // If there is a byte to read on serial port
      if (ser.waitReadable()) {
        // Get the date
        ros::Time date = ros::Time::now();

        // Wait that the robot write a pack of frames (13 maxi) on serial port
        // Hazardous function!
        ser.waitByteTimes(EFFIBOT_E3_MESSAGE_SIZE * 13); // 13 because we can receive a pack containing maxi 13 frames (with id = IN_...).

        // Read the serial port
        std::string serial_port_string;
        serial_port_string = ser.read(ser.available());
        int serial_port_size = serial_port_string.size();

        // If there is no incomplete frame and the first byte read is not a message header : do not process message
        if (!(communicator_receiver.isIncompleteFrame()) &&
          (((unsigned char)serial_port_string[0]) != EFFIBOT_E3_MESSAGE_HEADER) )
        {
          ROS_WARN(
            "[E3Receiver] Serial port read : There is no incomplete frame and the first byte read is not a message header");
          ROS_WARN_STREAM("\t\t" << convertMessageToHexa(serial_port_string));

          // ReInitialize incomplete frame variables...
          communicator_receiver.reInitialize();
        } else { // process message
          if (display_ros_info_main) {
            ROS_INFO_STREAM(
              " -------------- Messages received: " << "(size :" << serial_port_size << ") (date :" << date.toSec() - beginning_date <<
              ") --------------");
            ROS_INFO_STREAM("\t\t" << convertMessageToHexa(serial_port_string));
          }
          // Process the message read on serial port
          communicator_receiver.processMessage(serial_port_string, date);
        }
      } else { // If timeout has elapsed
        ROS_WARN("Timeout elapsed! No data available on serial port...");
      }
    } else {
      // Sleep a time in order to use less CPU.
      // Do NOT sleep more than 100ms because spinOnce() must be executed
      // to send new speeds (via joyVelCallback then controllerTimerCallback)
      usleep(SLEEP_TIME_RECEIVING_NO_MESSAGE); // sleep 100ms
    }
  }
  return 0;
}
