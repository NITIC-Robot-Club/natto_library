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

#include "natto_joy/button_manager.hpp"

namespace button_manager {

button_manager::button_manager (const rclcpp::NodeOptions &node_options) : Node ("button_manager", node_options) {
    power_publisher_            = this->create_publisher<std_msgs::msg::Bool> ("power", 10);
    allow_auto_drive_publisher_ = this->create_publisher<std_msgs::msg::Bool> ("allow_auto_drive", 10);
    joy_subscriber_             = this->create_subscription<sensor_msgs::msg::Joy> ("joy", 10, std::bind (&button_manager::joy_callback, this, std::placeholders::_1));

    power_on_button             = static_cast<size_t> (this->declare_parameter<int> ("power_on_button", 0));
    power_off_button            = static_cast<size_t> (this->declare_parameter<int> ("power_off_button", 1));
    allow_auto_drive_method     = this->declare_parameter<std::string> ("allow_auto_drive_method", "toggle");
    allow_auto_drive_on_button  = static_cast<size_t> (this->declare_parameter<int> ("allow_auto_drive_on_button", 2));
    allow_auto_drive_off_button = static_cast<size_t> (this->declare_parameter<int> ("allow_auto_drive_off_button", 3));

    RCLCPP_INFO (this->get_logger (), "button_manager node has been started.");
    RCLCPP_INFO (this->get_logger (), " power_on_button: %zu", power_on_button);
    RCLCPP_INFO (this->get_logger (), " power_off_button: %zu", power_off_button);
    RCLCPP_INFO (this->get_logger (), " allow_auto_drive_method: %s", allow_auto_drive_method.c_str ());
    if (allow_auto_drive_method == "toggle") {
        RCLCPP_INFO (this->get_logger (), " Using 'toggle' method for allow_auto_drive.");
        RCLCPP_INFO (this->get_logger (), " allow_auto_drive_on_button: %zu", allow_auto_drive_on_button);
        RCLCPP_INFO (this->get_logger (), " allow_auto_drive_off_button: %zu", allow_auto_drive_off_button);
    } else if (allow_auto_drive_method == "hold") {
        RCLCPP_INFO (this->get_logger (), " Using 'hold' method for allow_auto_drive.");
        RCLCPP_INFO (this->get_logger (), " allow_auto_drive_on_button: %zu", allow_auto_drive_on_button);
    } else {
        RCLCPP_ERROR (this->get_logger (), "Unknown allow_auto_drive_method: %s\n Please set to either 'toggle' or 'hold'.", allow_auto_drive_method.c_str ());
        rclcpp::shutdown ();
    }
}

void button_manager::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.size () > power_on_button && msg->buttons[power_on_button]) {
        power_msg_.data = true;
    }
    if (msg->buttons.size () > power_off_button && msg->buttons[power_off_button]) {
        power_msg_.data = false;
    }

    if (allow_auto_drive_method == "toggle") {
        if (msg->buttons.size () > allow_auto_drive_on_button && msg->buttons[allow_auto_drive_on_button]) {
            allow_auto_drive_msg_.data = true;
        }
        if (msg->buttons.size () > allow_auto_drive_off_button && msg->buttons[allow_auto_drive_off_button]) {
            allow_auto_drive_msg_.data = false;
        }
    } else if (allow_auto_drive_method == "hold") {
        if (msg->buttons.size () > allow_auto_drive_on_button && msg->buttons[allow_auto_drive_on_button]) {
            allow_auto_drive_msg_.data = true;
        } else {
            allow_auto_drive_msg_.data = false;
        }
    } else {
        RCLCPP_ERROR (this->get_logger (), "Unknown allow_auto_drive_method: %s\n Please set to either 'toggle' or 'hold'.", allow_auto_drive_method.c_str ());
    }

    power_publisher_->publish (power_msg_);
    allow_auto_drive_publisher_->publish (allow_auto_drive_msg_);
}

}  // namespace button_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (button_manager::button_manager)