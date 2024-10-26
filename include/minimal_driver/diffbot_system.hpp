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

#ifndef DIFF_DRIVE_CONTROLLER__DIFFBOT_SYSTEM_HPP_
#define DIFF_DRIVE_CONTROLLER__DIFFBOT_SYSTEM_HPP_

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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


namespace minimal_driver
{
class DiffDriveSystemHardware : public hardware_interface::SystemInterface
{

  struct config{
    std::string left_wheel_name = "";
    std::string right_wheel_name = "";
  };

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  // Wheel names
  config cfg_;
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Subscribers to get actual velocity from encoders
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_encoder_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_encoder_sub_;

  // Publishers to send velocity commands to the microcontroller
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_vel_pub_;

  rclcpp::Node::SharedPtr node_;

  //velocity from the encoders
  double left_wheel_velocity_ = 0.0;
  double right_wheel_velocity_ = 0.0;

  double left_wheel_command_ = 0.0;
  double right_wheel_command_ = 0.0;

  void left_encoder_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void right_encoder_callback(const std_msgs::msg::Float32::SharedPtr msg);

};

}  // namespace minimal_driver

#endif  // DIFF_DRIVE_CONTROLLER__DIFFBOT_SYSTEM_HPP_
