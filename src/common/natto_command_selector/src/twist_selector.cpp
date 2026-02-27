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
    controller_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float32> ("controller_angle", rclcpp::SensorDataQoS (), [this] (const std_msgs::msg::Float32::SharedPtr msg) { controller_angle_ = msg->data; });

    current_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped> ("current_pose", 10, [this] (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        robot_angle_ = std::atan2 (
            2.0 * (msg->pose.orientation.w * msg->pose.orientation.z + msg->pose.orientation.x * msg->pose.orientation.y), 1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y + msg->pose.orientation.z * msg->pose.orientation.z));
    });

    allow_auto_drive_ = this->declare_parameter<bool> ("initial_allow_auto_drive", false);

    double frequency        = this->declare_parameter<double> ("frequency", 5.0);
    enable_heading_control_ = this->declare_parameter<bool> ("enable_heading_control", false);
    RCLCPP_INFO (this->get_logger (), "twist_selector node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "initial_allow_auto_drive: %s", allow_auto_drive_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "frequency: %f", frequency);
    received_         = false;
    controller_angle_ = 0.0f;
    robot_angle_      = 0.0f;

    timer_ = this->create_wall_timer (std::chrono::milliseconds (static_cast<int> (1000.0 / frequency)), [this] () {
        if (!received_) {
            geometry_msgs::msg::TwistStamped stop_msg;
            stop_msg.header.stamp    = this->now ();
            stop_msg.header.frame_id = "base_link";
            stop_msg.twist.linear.x  = 0.0;
            stop_msg.twist.linear.y  = 0.0;
            stop_msg.twist.angular.z = 0.0;
            selected_twist_publisher_->publish (stop_msg);
            RCLCPP_WARN (this->get_logger (), "No twist command received yet. Publishing stop command.");
        }
        received_ = false;
    });
}

void twist_selector::allow_auto_drive_callback (const std_msgs::msg::Bool::SharedPtr msg) {
    allow_auto_drive_ = msg->data;
}

void twist_selector::manual_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (!allow_auto_drive_) {
        if (enable_heading_control_) {
            double angle        = std::atan2 (msg->twist.linear.y, msg->twist.linear.x);
            double speed        = std::hypot (msg->twist.linear.x, msg->twist.linear.y);
            msg->twist.linear.x = speed * std::cos (angle + controller_angle_ - robot_angle_);
            msg->twist.linear.y = speed * std::sin (angle + controller_angle_ - robot_angle_);
        }
        selected_twist_publisher_->publish (*msg);
        received_ = true;
    }
}

void twist_selector::auto_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (allow_auto_drive_) {
        selected_twist_publisher_->publish (*msg);
        received_ = true;
    }
}

}  // namespace twist_selector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (twist_selector::twist_selector)