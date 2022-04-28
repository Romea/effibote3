#include "effibote3_hardware/effibote3_hardware.hpp"
#include <rclcpp/rclcpp.hpp>

namespace romea {

//-----------------------------------------------------------------------------
EffibotE3Hardware::EffibotE3Hardware():
  HardwareSystemInterface4WD()
{

}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("EffibotE3Hardware"), "Read data from robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("EffibotE3Hardware"), "Send command to robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::connect()
{
  RCLCPP_INFO(rclcpp::get_logger("EffibotE3Hardware"), "Init communication with robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type EffibotE3Hardware::disconnect()
{
  RCLCPP_INFO(rclcpp::get_logger("Adap2eHardware"), "Close communication with robot");
  return hardware_interface::return_type::OK;
}

};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::EffibotE3Hardware, hardware_interface::SystemInterface)
