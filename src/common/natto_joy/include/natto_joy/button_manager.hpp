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

#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

namespace button_manager {
class button_manager : public rclcpp::Node {
   public:
    button_manager (const rclcpp::NodeOptions &node_options);

   private:
    size_t      power_on_button, power_off_button;
    std::string allow_auto_drive_method;
    size_t      allow_auto_drive_on_button, allow_auto_drive_off_button;

    std_msgs::msg::Bool power_msg_, allow_auto_drive_msg_;

    void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      power_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      allow_auto_drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
};
}  // namespace button_manager

#endif  // __BUTTON_MANAGER_HPP__