/*
 * Contains the definitions of functions used to display messages on screen (in binary or hex)
 */


#include "effibote3_hardware/effibote3_display.hpp"


namespace romea
{
namespace ros2
{


//===========================================================================

void charToBin_(std::ostringstream & ch, unsigned char A)
{
  unsigned short int B = 0;
  // Convert, one by one, the 8 bits of the unsigned char into a '0' or '1' and put it in the string stream
  for (int i = sizeof(A) * 7; i >= 0; i--) {
    B = A & (1 << i);
    ch << "" << (B == 0 ? '0' : '1');
  }
}

//===========================================================================

std::string convertMessageToBinary(const std::vector<unsigned char> & mess)
{
  std::ostringstream strsbin;

  // Conversion std::vector<unsigned char> to std::ostringstream
  for (size_t i = 0; i < mess.size(); i++) {
    charToBin_(strsbin, mess[i]);
    strsbin << " ";
  }

  return (std::string) (strsbin.str());
}

//===============================

std::string convertMessageToBinary(const std_msgs::msg::String & mess)
{
  std::ostringstream strsbin;

  // Conversion std_msgs::String to std::ostringstream
  for (size_t i = 0; i < mess.data.size(); i++) {
    charToBin_(strsbin, mess.data[i]);
    strsbin << " ";
  }

  return (std::string) (strsbin.str());
}

//===========================================================================

std::string convertMessageToHexa(const std_msgs::msg::String & mess)
{
  std::ostringstream strshex;

  // Conversion std_msgs::String to std::ostringstream
  for (size_t i = 0; i < mess.data.size(); i++) {
    strshex << "  " << std::hex << (int) ((unsigned char) (mess.data[i]));
  }

  return (std::string) (strshex.str());
}

//===============================

std::string convertMessageToHexa(const std::vector<unsigned char> & mess)
{
  std::ostringstream strshex;

  // Conversion std::vector<unsigned char> to std::ostringstream
  for (size_t i = 0; i < mess.size(); i++) {
    strshex << "  " << std::hex << (int) (mess[i]);
  }

  return (std::string) (strshex.str());
  // or
  //return (""<<strshex.str());
}

//===============================

std::string convertMessageToHexa(const std::string & mess)
{
  std::ostringstream strshex;

  // Conversion std::string to std::ostringstream
  for (size_t i = 0; i < mess.size(); i++) {
    strshex << "  " << std::hex << (int) ((unsigned char) (mess[i]));
  }

  return (std::string) (strshex.str());
}

}
}
