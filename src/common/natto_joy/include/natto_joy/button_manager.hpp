// Copyright 2025 Kazusa Hashimoto
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

#ifndef __BUTTON_MANAGER_HPP__
#define __BUTTON_MANAGER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace button_manager {
class button_manager : public rclcpp::Node {
   public:
    button_manager (const rclcpp::NodeOptions &node_options);

   private:
    size_t                   num_button_;
    std::vector<std::string> button_mode_;
    std::vector<std::string> button_function_;
    std::vector<std::string> joint_name_;
    std::vector<double>      position_on_;
    std::vector<double>      position_off_;
    std::vector<double>      speed_on_;
    std::vector<double>      speed_off_;
    std::vector<bool>        publish_always_;
    std::vector<int>         last_button_state_;

    std::string zr_mode_, zl_mode_;
    std::string zr_function_, zl_function_;
    std::string zr_joint_name_, zl_joint_name_;
    double      zr_position_on_, zr_position_off_;
    double      zl_position_on_, zl_position_off_;
    double      zr_speed_on_, zr_speed_off_;
    double      zl_speed_on_, zl_speed_off_;
    bool        zr_publish_always_, zl_publish_always_;

    std_msgs::msg::Bool          power_msg_, allow_auto_drive_msg_;
    sensor_msgs::msg::JointState command_joint_state_msg_, command_joint_state_always_msg_;

    void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          power_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          allow_auto_drive_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        origin_get_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr     joy_subscriber_;
};
}  // namespace button_manager

#endif  // __BUTTON_MANAGER_HPP__