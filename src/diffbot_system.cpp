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

#include "minimal_driver/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace minimal_driver
{
hardware_interface::CallbackReturn DiffDriveSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = (info_.hardware_parameters["left_wheel_name"]);
  cfg_.right_wheel_name = (info_.hardware_parameters["right_wheel_name"]);

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  node_ = rclcpp::Node::make_shared("diff_drive_controller");


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  left_encoder_sub_ = node_ -> create_subscription<std_msgs::msg::Float32>(
    "leftWheelSpeed", 10, std::bind(&DiffDriveSystemHardware::left_encoder_callback, this, std::placeholders::_1));
  right_encoder_sub_ = node_ -> create_subscription<std_msgs::msg::Float32>(
    "rightWheelSpeed", 10, std::bind(&DiffDriveSystemHardware::right_encoder_callback, this, std::placeholders::_1));

  // Initialize publishers to send velocity commands to the microcontroller
  left_vel_pub_ = node_->create_publisher<std_msgs::msg::Float32>("leftWheelCommand", 10);
  right_vel_pub_ = node_->create_publisher<std_msgs::msg::Float32>("rightWheelCommand", 10);


  RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  for (std::size_t i = 0; i < hw_velocities_.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

  }


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type minimal_driver ::DiffDriveSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  left_wheel_command_ = hw_commands_[0];
  right_wheel_command_ = hw_commands_[1];

  // Create a message to send to the motor controller
  std_msgs::msg::Float32 left_wheel_msg;
  std_msgs::msg::Float32 right_wheel_msg;

  left_wheel_msg.data = left_wheel_command_;
  right_wheel_msg.data = right_wheel_command_;
  
  left_vel_pub_ -> publish(left_wheel_msg);
  right_vel_pub_ -> publish(right_wheel_msg);

  return hardware_interface::return_type::OK;
}

void DiffDriveSystemHardware::left_encoder_callback(const std_msgs::msg::Float32::SharedPtr msg){
  left_wheel_velocity_ = msg->data;
  hw_velocities_[0] = left_wheel_velocity_;
}
void DiffDriveSystemHardware::right_encoder_callback(const std_msgs::msg::Float32::SharedPtr msg){
  left_wheel_velocity_ = msg->data;
  hw_velocities_[1] = right_wheel_velocity_;
}

}  // namespace minimal_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  minimal_driver::DiffDriveSystemHardware, hardware_interface::SystemInterface)