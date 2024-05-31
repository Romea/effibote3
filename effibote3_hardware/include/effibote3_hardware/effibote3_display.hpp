#ifndef EFFIBOTE3_CONTROL_EFFIBOTE3_DISPLAY_H
#define EFFIBOTE3_CONTROL_EFFIBOTE3_DISPLAY_H


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <bitset>

namespace romea
{
namespace ros2
{

/**
 * @brief Methods that transform a unsigned char into its binary value
 *        and put it in the ostringstream 'ch'
 */
void charToBin_(std::ostringstream & ch, unsigned char A);


/**
 * @brief Methods that return a string after having converted each char in binary/hex representation
 *        from the given list of char (i.e. std::vector<unsigned char> OR std_msgs::String OR std::string).
 * @param mess : message to be converted.
 * @return a string
 */
std::string convertMessageToBinary(const std::vector<unsigned char> & mess);
std::string convertMessageToBinary(const std_msgs::msg::String & mess);

std::string convertMessageToHexa(const std_msgs::msg::String & mess);
std::string convertMessageToHexa(const std::vector<unsigned char> & mess);
std::string convertMessageToHexa(const std::string & mess);

}  // namespace ros2
}  // namespace romea

#endif
