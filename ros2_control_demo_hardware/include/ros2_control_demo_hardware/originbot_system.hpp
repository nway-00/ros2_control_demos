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

#ifndef ROS2_CONTROL_DEMO_HARDWARE__ORIGINBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__ORIGINBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "ros2_control_demo_hardware/visibility_control.h"

#include "originbot_comms.hpp"
#include "wheel.hpp"


namespace ros2_control_demo_hardware
{

class OriginBotSystemHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OriginBotSystemHardware);

  OriginBotSystemHardware();

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Parameters for the OriginBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  std::string hw_serial_dev_;

  // Store the command for the simulated robot
  //std::vector<double> hw_commands_;
  //std::vector<double> hw_positions_;   // ???不知道是什么
  //std::vector<double> hw_velocities_;  // 每个轮子的速度
  Wheel hw_wheel_l_;  // 左轮
  Wheel hw_wheel_r_;  // 右轮

  // Store the wheeled robot position
  double base_x_, base_y_, base_theta_;

  rclcpp::Logger logger_;

  std::chrono::time_point<std::chrono::system_clock> time_;

  OriginBotComms originbot_;
};

}  // namespace ros2_control_demo_hardware

#endif  // ROS2_CONTROL_DEMO_HARDWARE__ORIGINBOT_SYSTEM_HPP_
