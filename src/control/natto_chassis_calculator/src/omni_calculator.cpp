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

#include "natto_chassis_calculator/omni_calculator.hpp"

namespace omni_calculator {

omni_calculator::omni_calculator (const rclcpp::NodeOptions &node_options) : Node ("omni_calculator", node_options) {
    omni_command_publisher_   = this->create_publisher<natto_msgs::msg::Omni> ("omni_command", 10);
    twist_command_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped> ("command_velocity", 10, std::bind (&omni_calculator::command_velocity_callback, this, std::placeholders::_1));

    wheel_radius_    = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_position_x = this->declare_parameter<std::vector<double>> ("wheel_position_x", {0.5, -0.5, -0.5, 0.5});
    wheel_position_y = this->declare_parameter<std::vector<double>> ("wheel_position_y", {0.5, 0.5, -0.5, -0.5});
    wheel_angle      = this->declare_parameter<std::vector<double>> ("wheel_angle", {-45.0, 45.0, 135.0, -135.0});

    num_wheels_ = wheel_position_x.size ();
    if (wheel_position_y.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_position_x and wheel_position_y must have the same size.");
        throw std::runtime_error ("wheel_position_x and wheel_position_y must have the same size.");
    }

    if (wheel_angle.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_angle must have the same size as wheel_position_x and wheel_position_y.");
        throw std::runtime_error ("wheel_angle must have the same size as wheel_position_x and wheel_position_y.");
    }

    RCLCPP_INFO (this->get_logger (), "omni_calculator node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "Wheel radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %d", num_wheels_);
    for (int i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "Wheel %d position: (%.2f, %.2f), angle: %.2f deg", i, wheel_position_x[i], wheel_position_y[i], wheel_angle[i]);
    }
}

void omni_calculator::command_velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    natto_msgs::msg::Omni omni_msg;
    omni_msg.wheel_speed.resize (num_wheels_, 0.0);

    double x = msg->twist.linear.x;
    double y = msg->twist.linear.y;
    double z = msg->twist.angular.z;

    for (int i = 0; i < num_wheels_; i++) {
        double wheel_vx        = x - z * wheel_position_y[i];
        double wheel_vy        = y + z * wheel_position_x[i];
        double wheel_speed     = std::sqrt (wheel_vx * wheel_vx + wheel_vy * wheel_vy);
        double wheel_direction = std::atan2 (wheel_vy, wheel_vx);

        double wheel_angle_rad      = wheel_angle[i] * M_PI / 180.0;
        double adjusted_wheel_speed = wheel_speed * std::cos (wheel_direction - wheel_angle_rad);

        omni_msg.wheel_speed[i] = (adjusted_wheel_speed / wheel_radius_) / (2.0 * M_PI);
    }

    omni_command_publisher_->publish (omni_msg);
}

}  // namespace omni_calculator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (omni_calculator::omni_calculator)