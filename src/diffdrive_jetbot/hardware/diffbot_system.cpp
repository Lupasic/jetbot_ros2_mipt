// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffdrive_jetbot/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_jetbot
{
hardware_interface::CallbackReturn DiffDriveJetbotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  
  // Motor configuration parameters from xacro
  cfg_.mtype = std::stoi(info_.hardware_parameters["mtype"]);
  cfg_.deadzone = std::stoi(info_.hardware_parameters["deadzone"]);
  cfg_.mline = std::stoi(info_.hardware_parameters["mline"]);
  cfg_.mphase = std::stoi(info_.hardware_parameters["mphase"]);
  cfg_.wdiameter = std::stoi(info_.hardware_parameters["wdiameter"]);
  
  // Calculate encoder counts per revolution: mline * 4 * mphase
  cfg_.enc_counts_per_rev = cfg_.mline * 4 * cfg_.mphase;
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveJetbotHardware"), 
              "Calculated enc_counts_per_rev: %d (mline: %d, mphase: %d)", 
              cfg_.enc_counts_per_rev, cfg_.mline, cfg_.mphase);
  
  cfg_.pid_p = std::stod(info_.hardware_parameters["pid_p"]);
  cfg_.pid_d = std::stod(info_.hardware_parameters["pid_d"]);
  cfg_.pid_i = std::stod(info_.hardware_parameters["pid_i"]);
  
  cfg_.max_motor_rpm = std::stod(info_.hardware_parameters["max_motor_rpm"]);
  
  // Convert RPM to rad/s: RPM * 2π / 60
  cfg_.max_motor_rads = cfg_.max_motor_rpm * 2.0 * M_PI / 60.0;
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveJetbotHardware"), 
              "Max motor speed: %.2f RPM = %.2f rad/s", 
              cfg_.max_motor_rpm, cfg_.max_motor_rads);
  

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveJetbotHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveJetbotHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveJetbotHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveJetbotHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveJetbotHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveJetbotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveJetbotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveJetbotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveJetbotHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  
  // Setup all motor parameters
  comms_.setup_motor_parameters(cfg_.mtype, cfg_.deadzone, cfg_.mline, cfg_.mphase, cfg_.wdiameter, cfg_.pid_p, cfg_.pid_i, cfg_.pid_d);
  comms_.read_flash_settings();
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveJetbotHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveJetbotHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveJetbotHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveJetbotHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveJetbotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveJetbotHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveJetbotHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveJetbotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveJetbotHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveJetbotHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveJetbotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // Read total encoder counts
  double left_enc_total, right_enc_total;
  comms_.read_encoder_data(left_enc_total, right_enc_total, 1);
  
  // Update wheel encoder values
  wheel_l_.enc = left_enc_total;
  wheel_r_.enc = right_enc_total;
  
  // Update wheel positions based on encoder counts
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  
  // Read current motor speeds directly from hardware
  double left_speed_rads, right_speed_rads;
  comms_.read_encoder_data(left_speed_rads, right_speed_rads, 3);
  
  // Update wheel velocities with direct readings
  wheel_l_.vel = left_speed_rads / 1000.0 * cfg_.max_motor_rads;
  wheel_r_.vel = right_speed_rads / 1000.0 * cfg_.max_motor_rads;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_jetbot ::DiffDriveJetbotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_l_ticks = wheel_l_.cmd * 1000.0 / cfg_.max_motor_rads;
  int motor_r_ticks = wheel_r_.cmd * 1000.0 / cfg_.max_motor_rads;
  comms_.set_motor_speed_values(motor_l_ticks, motor_r_ticks);
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_jetbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_jetbot::DiffDriveJetbotHardware, hardware_interface::SystemInterface)
