#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>


struct Config
{
  std::string F_left_wheel_name = "F_left_wheel";
  std::string F_right_wheel_name = "F_right_wheel";
  std::string B_left_wheel_name = "B_left_wheel";
  std::string B_right_wheel_name = "B_right_wheel";
  float loop_rate = 30;
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int timeout = 1000;
  int enc_counts_per_rev = 1920;
};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H