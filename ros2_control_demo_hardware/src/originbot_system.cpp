
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

#include "ros2_control_demo_hardware/originbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ros2_control_demo_hardware
{
OriginBotSystemHardware::OriginBotSystemHardware()
  : logger_(rclcpp::get_logger("OriginBotSystemHardware"))
{
  
}

hardware_interface::return_type OriginBotSystemHardware::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  time_ = std::chrono::system_clock::now(); // 上次时间

  // 参数 & 状态，命令
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);  // 延迟启动secs
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);    // 延迟停止secs
  hw_serial_dev_ = info_.hardware_parameters["example_param_hw_device"];                   // 串口设备名称
  //hw_positions_.resize( info_.joints.size(), std::numeric_limits<double>::quiet_NaN());  // 位置数组
  //hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());  // 速度数组
  //hw_commands_.resize(  info_.joints.size(), std::numeric_limits<double>::quiet_NaN());  // 命令数组

  // 检查 joints 个数
  if(info_.joints.size() != 2){
      RCLCPP_FATAL(logger_, "More than 2 joints %d  found. 2 expected.", info_.joints.size());
      return hardware_interface::return_type::ERROR;
  }
  // 检查每个 joint 中 command_interfaces 和 state_interface
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint: 
    // {command_interfaces:{HW_IF_VELOCITY}, state_interfaces:{HW_IF_POSITION,HW_IF_VELOCITY}}
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OriginBotSystemHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }
    // 检查 command_interfaces[0].name
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OriginBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
    // 检查 state_interface
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OriginBotSystemHardware"),
        "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OriginBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OriginBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  //set up the wheels
  hw_wheel_l_.setup(info_.joints[0].name);
  hw_wheel_r_.setup(info_.joints[1].name);

  // 串口设置
  originbot_.setup(hw_serial_dev_, 115200, 2000);

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> OriginBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(hw_wheel_l_.name, hardware_interface::HW_IF_POSITION, &hw_wheel_l_.pos));  // 左轮-pos
  state_interfaces.emplace_back(hardware_interface::StateInterface(hw_wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &hw_wheel_l_.vel));  // 左轮-速度
  state_interfaces.emplace_back(hardware_interface::StateInterface(hw_wheel_r_.name, hardware_interface::HW_IF_POSITION, &hw_wheel_r_.pos));  // 右轮-pos
  state_interfaces.emplace_back(hardware_interface::StateInterface(hw_wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &hw_wheel_r_.vel));  // 右轮-速度

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OriginBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(hw_wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &hw_wheel_l_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(hw_wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &hw_wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::return_type OriginBotSystemHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("OriginBotSystemHardware"), "Starting ...please wait...");

  // 延迟 hw_start_sec_ 秒
  for (auto i = 0; i <= hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("OriginBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // 停车
  originbot_.volocity_control(0.0f, 0.0f);

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("OriginBotSystemHardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OriginBotSystemHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("OriginBotSystemHardware"), "Stopping ...please wait...");

  // 延迟 hw_stop_sec_ 秒停止
  for (auto i = 0; i <= hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("OriginBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }

   // 停车
  originbot_.volocity_control(0.0f, 0.0f);

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("OriginBotSystemHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OriginBotSystemHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("OriginBotSystemHardware"), "Reading...");

  //Caculate time delta
  auto new_time = std::chrono::system_clock::now();
  auto delt = new_time - time_;
  double deltaSeconds = delt.count();
  time_ = new_time;

  // 检查连接
  if(! originbot_.connected()){
    return hardware_interface::return_type::ERROR;
  }

  // 获取左轮速度，右轮速度
  float left_vel = 0.0f, right_vel = 0.0f; // 左轮速度m/s 右轮速度m/s
  originbot_.get_volocity(left_vel, right_vel);
  
  double pos_prv = hw_wheel_l_.pos;
  hw_wheel_l_.vel = left_vel;
  hw_wheel_l_.pos = (left_vel * deltaSeconds) + pos_prv;
  
  pos_prv = hw_wheel_r_.pos;
  hw_wheel_r_.vel = right_vel;
  hw_wheel_r_.pos = (right_vel * deltaSeconds) + pos_prv;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_hardware::OriginBotSystemHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("OriginBotSystemHardware"), "Writing...");

  originbot_.volocity_control(hw_wheel_l_.cmd, hw_wheel_r_.cmd);

  // for (auto i = 0u; i < hw_commands_.size(); i++)
  // {
  //   // Simulate sending commands to the hardware
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("OriginBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
  //     info_.joints[i].name.c_str());
  // }
  RCLCPP_INFO(rclcpp::get_logger("OriginBotSystemHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::OriginBotSystemHardware, hardware_interface::SystemInterface)
