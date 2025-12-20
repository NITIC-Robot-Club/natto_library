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

#include "natto_command_selector/twist_selector.hpp"

namespace twist_selector {

twist_selector::twist_selector (const rclcpp::NodeOptions &node_options) : Node ("twist_selector", node_options) {
    allow_auto_drive_subscriber_ = this->create_subscription<std_msgs::msg::Bool> ("allow_auto_drive", 10, std::bind (&twist_selector::allow_auto_drive_callback, this, std::placeholders::_1));
    manual_subscriber_           = this->create_subscription<geometry_msgs::msg::TwistStamped> ("manual_twist", 10, std::bind (&twist_selector::manual_callback, this, std::placeholders::_1));
    auto_subscriber_             = this->create_subscription<geometry_msgs::msg::TwistStamped> ("auto_twist", 10, std::bind (&twist_selector::auto_callback, this, std::placeholders::_1));
    selected_twist_publisher_    = this->create_publisher<geometry_msgs::msg::TwistStamped> ("selected_twist", 10);
    allow_auto_drive_            = this->declare_parameter<bool> ("initial_allow_auto_drive", false);

    RCLCPP_INFO (this->get_logger (), "twist_selector node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "initial_allow_auto_drive: %s", allow_auto_drive_ ? "true" : "false");
}

void twist_selector::allow_auto_drive_callback (const std_msgs::msg::Bool::SharedPtr msg) {
    allow_auto_drive_ = msg->data;
}

void twist_selector::manual_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (!allow_auto_drive_) {
        selected_twist_publisher_->publish (*msg);
    }
}

void twist_selector::auto_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (allow_auto_drive_) {
        selected_twist_publisher_->publish (*msg);
    }
}

}  // namespace twist_selector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (twist_selector::twist_selector)