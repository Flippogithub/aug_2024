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

#ifndef BMC_2024_AUG_2024_SYSTEM_HPP_
#define BMC_2024_AUG_2024_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "BMC_2024/visibility_control.h"

#include "BMC_2024/bmc_comms.hpp"

namespace BMC_2024
{
class AUG2024Hardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string front_left_motor_name = "";
  std::string front_right_motor_name = "";
  std::string rear_left_motor_name = "";
  std::string rear_right_motor_name = "";
  std::string can_interface = "can0";
  int can_fl_id; // = 0xB7; 
  int can_fr_id; // = 0xB1;
  int can_rl_id; // = 0xB2;
  int can_rr_id; // = 0xB3;
  float fl_mult = 1.0;
  float fr_mult = -1.0;
  float rl_mult = 1.0;
  float rr_mult = -1.0;
  float pid_p = 0.0;
  float pid_i = 0.0;
  float pid_d = 0.0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AUG2024Hardware)

  ROWBOT_BMC_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ROWBOT_BMC_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROWBOT_BMC_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROWBOT_BMC_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROWBOT_BMC_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  ROWBOT_BMC_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROWBOT_BMC_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROWBOT_BMC_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROWBOT_BMC_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  //BMCComms comms_fl_;
  //BMCComms comms_fr_;
  BMCComms comms_rl_;
  BMCComms comms_rr_;
  Config cfg_;
  //Motor motor_fl_;
  //Motor motor_fr_;
  Motor motor_rl_;
  Motor motor_rr_;
};

}  // namespace rowbot_bmc

#endif  // ROWBOT_BMC__ROWBOT_SYSTEM_HPP_
