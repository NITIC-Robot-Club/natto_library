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

#include "natto_chassis_calculator/two_wheel_calculator.hpp"

namespace two_wheel_calculator {

two_wheel_calculator::two_wheel_calculator (const rclcpp::NodeOptions &node_options) : Node ("two_wheel_calculator", node_options) {
    two_wheel_command_publisher_ = this->create_publisher<natto_msgs::msg::TwoWheel> ("two_wheel_command", 10);
    twist_command_subscriber_    = this->create_subscription<geometry_msgs::msg::TwistStamped> ("command_velocity", 10, std::bind (&two_wheel_calculator::command_velocity_callback, this, std::placeholders::_1));

    wheel_radius_ = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_base_   = this->declare_parameter<double> ("wheel_base", 0.5);

    RCLCPP_INFO (this->get_logger (), "two_wheel_calculator node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "wheel_base: %.2f m", wheel_base_);
}

void two_wheel_calculator::command_velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    natto_msgs::msg::TwoWheel two_wheel_msg;
    two_wheel_msg.wheel_speed.resize (2, 0.0);

    // Disable lateral movement (ignore msg->twist.linear.y)
    double linear_velocity  = msg->twist.linear.x;
    double angular_velocity = msg->twist.angular.z;

    // Differential drive kinematics
    // v_left = linear_velocity - (wheel_base / 2) * angular_velocity
    // v_right = linear_velocity + (wheel_base / 2) * angular_velocity
    double v_left  = linear_velocity - (wheel_base_ / 2.0) * angular_velocity;
    double v_right = linear_velocity + (wheel_base_ / 2.0) * angular_velocity;

    // Convert linear velocity to wheel speed in rps (revolutions per second)
    two_wheel_msg.wheel_speed[0] = (v_left / wheel_radius_) / (2.0 * M_PI);
    two_wheel_msg.wheel_speed[1] = (v_right / wheel_radius_) / (2.0 * M_PI);

    two_wheel_command_publisher_->publish (two_wheel_msg);
}

}  // namespace two_wheel_calculator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (two_wheel_calculator::two_wheel_calculator)
