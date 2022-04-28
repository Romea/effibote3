#ifndef EFFIBOTE3_HARDWARE_HPP_
#define EFFIBOTE3_HARDWARE_HPP_

#include "romea_mobile_base_hardware/hardware_system_interface.hpp"
#include <rclcpp/macros.hpp>

namespace romea
{

class EffibotE3Hardware : public HardwareSystemInterface4WD
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(EffibotE3Hardware);

  EffibotE3Hardware();

  virtual hardware_interface::return_type read() override;

  virtual hardware_interface::return_type write() override;

  virtual hardware_interface::return_type connect() override;

  virtual hardware_interface::return_type disconnect() override;

};

}

#endif
