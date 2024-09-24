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

#include "BMC_2024/aug_2024_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace BMC_2024
{
hardware_interface::CallbackReturn AUG2024Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  //cfg_.front_left_motor_name = info_.hardware_parameters["front_left_wheel_name"];
  //cfg_.front_right_motor_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.rear_left_motor_name = info_.hardware_parameters["rear_left_wheel_name"];
  cfg_.rear_right_motor_name = info_.hardware_parameters["rear_right_wheel_name"];
  cfg_.can_interface = info_.hardware_parameters["can_interface"];
 try {
  //cfg_.can_fl_id = std::stoi(info_.hardware_parameters["can_fl_id"]); //this comes from the urdf file can_fl_id is a parameter in the urdf file
  //cfg_.can_fr_id = std::stoi(info_.hardware_parameters["can_fr_id"]); //gets turned into a info.hardware_parameters type
  cfg_.can_rl_id = std::stoi(info_.hardware_parameters["can_rl_id"]);
  cfg_.can_rr_id = std::stoi(info_.hardware_parameters["can_rr_id"]);
} catch (const std::invalid_argument& e) {
  RCLCPP_ERROR(rclcpp::get_logger("AUG2024Hardware"), "Invalid CAN ID parameter: %s", e.what());
  return hardware_interface::CallbackReturn::ERROR;
} catch (const std::out_of_range& e) {
  RCLCPP_ERROR(rclcpp::get_logger("AUG2024Hardware"), "CAN ID out of range: %s", e.what());
  return hardware_interface::CallbackReturn::ERROR;
}
  //cfg_.fl_mult = std::stof(info_.hardware_parameters["fl_mult"]);
  //cfg_.fr_mult = std::stof(info_.hardware_parameters["fr_mult"]);
  cfg_.rl_mult = std::stof(info_.hardware_parameters["rl_mult"]);
  cfg_.rr_mult = std::stof(info_.hardware_parameters["rr_mult"]);
  cfg_.pid_p = std::stof(info_.hardware_parameters["pid_p"]);
  cfg_.pid_i = std::stof(info_.hardware_parameters["pid_i"]);
  cfg_.pid_d = std::stof(info_.hardware_parameters["pid_d"]);

  //motor_fl_.name = cfg_.front_left_motor_name; //why change from cfg_ to motor_fl_? 
  //motor_fr_.name = cfg_.front_right_motor_name;
  motor_rl_.name = cfg_.rear_left_motor_name;
  motor_rr_.name = cfg_.rear_right_motor_name;

  //motor_fl_.mult = cfg_.fl_mult;
  //motor_fr_.mult = cfg_.fr_mult;
  motor_rl_.mult = cfg_.rl_mult;
  motor_rr_.mult = cfg_.rr_mult;
  
  //motor_fl_.can_id = (uint32_t)cfg_.can_fl_id;
  //motor_fr_.can_id = (uint32_t)cfg_.can_fr_id;
  motor_rl_.can_id = (uint32_t)cfg_.can_rl_id;
  motor_rr_.can_id = (uint32_t)cfg_.can_rr_id;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RowbotSystem has exactly three states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AUG2024Hardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AUG2024Hardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AUG2024Hardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AUG2024Hardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AUG2024Hardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    /*
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_CURRENT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AUG2024Hardware"),
        "Joint '%s' have '%s' as third state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_CURRENT);
      return hardware_interface::CallbackReturn::ERROR;
    }
    */
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AUG2024Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // front left motor
 // state_interfaces.emplace_back(hardware_interface::StateInterface(
   // motor_fl_.name, hardware_interface::HW_IF_POSITION, &motor_fl_.position));
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
    //motor_fl_.name, hardware_interface::HW_IF_VELOCITY, &motor_fl_.velocity));
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
  //  motor_fl_.name, hardware_interface::HW_IF_CURRENT, &motor_fl_.current));

  // front right motor
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
    //motor_fr_.name, hardware_interface::HW_IF_POSITION, &motor_fr_.position));
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
    //motor_fr_.name, hardware_interface::HW_IF_VELOCITY, &motor_fr_.velocity));
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
  //  motor_fr_.name, hardware_interface::HW_IF_CURRENT, &motor_fr_.current));

  // rear left motor
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    motor_rl_.name, hardware_interface::HW_IF_POSITION, &motor_rl_.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    motor_rl_.name, hardware_interface::HW_IF_VELOCITY, &motor_rl_.velocity));
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
  //  motor_rl_.name, hardware_interface::HW_IF_CURRENT, &motor_rl_.current));
  
  // rear right motor
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    motor_rr_.name, hardware_interface::HW_IF_POSITION, &motor_rr_.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    motor_rr_.name, hardware_interface::HW_IF_VELOCITY, &motor_rr_.velocity));
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
  //  motor_rr_.name, hardware_interface::HW_IF_CURRENT, &motor_rr_.current));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AUG2024Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  //command_interfaces.emplace_back(hardware_interface::CommandInterface(
   // motor_fl_.name, hardware_interface::HW_IF_VELOCITY, &motor_fl_.cmd));
  //command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //motor_fr_.name, hardware_interface::HW_IF_VELOCITY, &motor_fr_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    motor_rl_.name, hardware_interface::HW_IF_VELOCITY, &motor_rl_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    motor_rr_.name, hardware_interface::HW_IF_VELOCITY, &motor_rr_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn AUG2024Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "Configuring ...please wait...");

  // open can sockets
  //comms_fl_.open(cfg_.can_interface.c_str(), motor_fl_.can_id);
  //comms_fr_.open(cfg_.can_interface.c_str(), motor_fr_.can_id);
  comms_rl_.open(cfg_.can_interface.c_str(), motor_rl_.can_id);
  comms_rr_.open(cfg_.can_interface.c_str(), motor_rr_.can_id);
  // set pids
  //comms_fl_.setPIDs(cfg_.pid_p, cfg_.pid_i, cfg_.pid_d);
  //comms_fr_.setPIDs(cfg_.pid_p, cfg_.pid_i, cfg_.pid_d);
  comms_rl_.setPIDs(cfg_.pid_p, cfg_.pid_i, cfg_.pid_d);
  comms_rr_.setPIDs(cfg_.pid_p, cfg_.pid_i, cfg_.pid_d);

  RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AUG2024Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "Activating ...please wait...");

  // enable the bmcs
  //comms_fl_.enable();
  //comms_fr_.enable();
  comms_rl_.enable();
  comms_rr_.enable();

  RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AUG2024Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "Deactivating ...please wait...");

  //comms_fl_.disable();
  //comms_fr_.disable();
  comms_rl_.disable();
  comms_rr_.disable();
  
  RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AUG2024Hardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "Cleaning up...please wait...");

  //comms_fl_.close();
  //comms_fr_.close();
  comms_rl_.close();
  comms_rr_.close();
  
  RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AUG2024Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{ 
  // update states
  //comms_fl_.update(&motor_fl_);
  //comms_fr_.update(&motor_fr_);
  comms_rl_.update(&motor_rl_);
  comms_rr_.update(&motor_rr_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BMC_2024 ::AUG2024Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
 
  //comms_fl_.setVelocity(motor_fl_.cmd * motor_fl_.mult); //motor_fl_.cmd is the velocity command coming from the controller
  //comms_fr_.setVelocity(motor_fr_.cmd * motor_fr_.mult);
  comms_rl_.setVelocity(motor_rl_.cmd * motor_rl_.mult);
  comms_rr_.setVelocity(motor_rr_.cmd * motor_rr_.mult);
  //RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "Front Left Motor Command: %.3f", (motor_fl_.cmd * motor_rr_.mult));
  //RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "Front right Motor Command: %.3f", (motor_fr_.cmd * motor_rr_.mult));
  //RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "rear Left Motor Command: %.3f", (motor_rl_.cmd * motor_rr_.mult));
  //RCLCPP_INFO(rclcpp::get_logger("AUG2024Hardware"), "rear right Motor Command: %.3f", (motor_rr_.cmd * motor_rr_.mult));
  return hardware_interface::return_type::OK;
}

}  // namespace rowbot_bmc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  BMC_2024::AUG2024Hardware, hardware_interface::SystemInterface)
