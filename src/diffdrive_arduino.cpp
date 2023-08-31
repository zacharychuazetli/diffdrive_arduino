#include "diffdrive_arduino/diffdrive_arduino.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"




DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}





return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.F_left_wheel_name = info_.hardware_parameters["F_left_wheel_name"];
  cfg_.F_right_wheel_name = info_.hardware_parameters["F_right_wheel_name"];
  cfg_.B_left_wheel_name = info_.hardware_parameters["B_left_wheel_name"];
  cfg_.B_right_wheel_name = info_.hardware_parameters["B_right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Set up the wheels
  F_l_wheel_.setup(cfg_.F_left_wheel_name, cfg_.enc_counts_per_rev);
  F_r_wheel_.setup(cfg_.F_right_wheel_name, cfg_.enc_counts_per_rev);
  B_l_wheel_.setup(cfg_.B_left_wheel_name, cfg_.enc_counts_per_rev);
  B_r_wheel_.setup(cfg_.B_right_wheel_name, cfg_.enc_counts_per_rev);

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(F_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &F_l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(F_l_wheel_.name, hardware_interface::HW_IF_POSITION, &F_l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(F_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &F_r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(F_r_wheel_.name, hardware_interface::HW_IF_POSITION, &F_r_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(B_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &B_l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(B_l_wheel_.name, hardware_interface::HW_IF_POSITION, &B_l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(B_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &B_r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(B_r_wheel_.name, hardware_interface::HW_IF_POSITION, &B_r_wheel_.pos));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(F_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &F_l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(F_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &F_r_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(B_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &B_l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(B_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &B_r_wheel_.cmd));

  return command_interfaces;
}


return_type DiffDriveArduino::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  arduino_.sendEmptyMsg();
  // arduino.setPidValues(9,7,0,100);
  // arduino.setPidValues(14,7,0,100);
  arduino_.setPidValues(30, 20, 0, 100);
  //arduino_.setPidValues(20, 12, 0, 50);

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type DiffDriveArduino::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::read()
{

  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.readEncoderValues(F_l_wheel_.enc, F_r_wheel_.enc, B_l_wheel_.enc, B_r_wheel_.enc);

  double pos_prev = F_l_wheel_.pos;
  F_l_wheel_.pos = F_l_wheel_.calcEncAngle();
  F_l_wheel_.vel = (F_l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = F_r_wheel_.pos;
  F_r_wheel_.pos = F_r_wheel_.calcEncAngle();
  F_r_wheel_.vel = (F_r_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = B_l_wheel_.pos;
  B_l_wheel_.pos = B_l_wheel_.calcEncAngle();
  B_l_wheel_.vel = (B_l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = B_r_wheel_.pos;
  B_r_wheel_.pos = B_r_wheel_.calcEncAngle();
  B_r_wheel_.vel = (B_r_wheel_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;

  
}

hardware_interface::return_type DiffDriveArduino::write()
{

  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.setMotorValues(F_l_wheel_.cmd / F_l_wheel_.rads_per_count / cfg_.loop_rate, F_r_wheel_.cmd / F_r_wheel_.rads_per_count / cfg_.loop_rate,
                          B_l_wheel_.cmd / B_l_wheel_.rads_per_count / cfg_.loop_rate, B_r_wheel_.cmd / B_r_wheel_.rads_per_count / cfg_.loop_rate);




  return return_type::OK;


  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)