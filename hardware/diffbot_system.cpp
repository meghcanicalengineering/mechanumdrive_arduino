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

#include "mechanumdrive_arduino/MechanumDriveArduinoHardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mechanumdrive_arduino
{
hardware_interface::CallbackReturn MechanumDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.front_left_wheel_name = info_.hardware_parameters["drivewh2_l"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["drivewh2_r"];
  cfg_.rear_left_wheel_name = info_.hardware_parameters["drivewh1_l"];
  cfg_.rear_right_wheel_name = info_.hardware_parameters["drivewh1_r"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("MechanumDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }

  wheel_fl_.setup(cfg_.drivewh2_l, cfg_.enc_counts_per_rev);
  wheel_fr_.setup(cfg_.drivewh2_r, cfg_.enc_counts_per_rev);
  wheel_rl_.setup(cfg_.drivewh1_l, cfg_.enc_counts_per_rev);
  wheel_rr_.setup(cfg_.drivewh1_r, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // MechanumDriveSystem expects exactly two state interfaces and one command interface per joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MechanumDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MechanumDriveArduinoHardware"),
        "Joint '%s' has an incorrect command interface '%s'. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MechanumDriveArduinoHardware"),
        "Joint '%s' has %zu state interfaces. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MechanumDriveArduinoHardware"),
        "Joint '%s' has incorrect state interfaces. Expected '%s' and '%s'.", joint.name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MechanumDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // Front left wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fl_.name, hardware_interface::HW_IF_POSITION, &wheel_fl_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.vel));
  // Front right wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fr_.name, hardware_interface::HW_IF_POSITION, &wheel_fr_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.vel));
  // Rear left wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rl_.name, hardware_interface::HW_IF_POSITION, &wheel_rl_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rl_.vel));
  // Rear right wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rr_.name, hardware_interface::HW_IF_POSITION, &wheel_rr_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MechanumDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // Front left wheel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.cmd));
  // Front right wheel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.cmd));
  // Rear left wheel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_rl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rl_.cmd));
  // Rear right wheel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.cmd));

  return command_interfaces;
}

// Additional methods (on_configure, on_cleanup, on_activate, on_deactivate, read, write)
// should be updated similarly to manage four wheels and their unique control requirements for a mecanum drive.

}  // namespace mechanumdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mechanumdrive_arduino::MechanumDriveArduinoHardware, hardware_interface::SystemInterface)
