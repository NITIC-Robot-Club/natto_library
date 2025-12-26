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

#include "natto_chassis_calculator/swerve_calculator.hpp"

namespace swerve_calculator {

swerve_calculator::swerve_calculator (const rclcpp::NodeOptions &node_options) : Node ("swerve_calculator", node_options) {
    swerve_command_publisher_ = this->create_publisher<natto_msgs::msg::Swerve> ("swerve_command", 10);
    joint_state_publisher_    = this->create_publisher<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::QoS (10).best_effort ());
    twist_command_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped> ("command_velocity", 10, std::bind (&swerve_calculator::command_velocity_callback, this, std::placeholders::_1));
    swerve_result_subscriber_ = this->create_subscription<natto_msgs::msg::Swerve> ("swerve_result", 10, std::bind (&swerve_calculator::swerve_result_callback, this, std::placeholders::_1));

    infinite_swerve_mode_ = this->declare_parameter<bool> ("infinite_swerve_mode", false);
    publish_joint_state_  = this->declare_parameter<bool> ("publish_joint_state", true);
    wheel_radius_         = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_position_x_     = this->declare_parameter<std::vector<double>> ("wheel_position_x", {0.5, -0.5, -0.5, 0.5});
    wheel_position_y_     = this->declare_parameter<std::vector<double>> ("wheel_position_y", {0.5, 0.5, -0.5, -0.5});
    if (publish_joint_state_) {
        steer_names_ = this->declare_parameter<std::vector<std::string>> ("steer_names", {"swerve_steer_0", "swerve_steer_1", "swerve_steer_2", "swerve_steer_3"});
        wheel_names_ = this->declare_parameter<std::vector<std::string>> ("wheel_names", {"swerve_wheel_0", "swerve_wheel_1", "swerve_wheel_2", "swerve_wheel_3"});
    }

    num_wheels_ = wheel_position_x_.size ();
    if (wheel_position_y_.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_position_x and wheel_position_y must have the same size.");
        throw std::runtime_error ("wheel_position_x and wheel_position_y must have the same size.");
    }

    if (publish_joint_state_) {
        if (steer_names_.size () != num_wheels_) {
            RCLCPP_ERROR (this->get_logger (), "steer_names size must be equal to number of wheels.");
            throw std::runtime_error ("steer_names size must be equal to number of wheels.");
        }
        if (wheel_names_.size () != num_wheels_) {
            RCLCPP_ERROR (this->get_logger (), "wheel_names size must be equal to number of wheels.");
            throw std::runtime_error ("wheel_names size must be equal to number of wheels.");
        }
        joint_state_msg_.name.resize (num_wheels_ * 2);
        joint_state_msg_.position.resize (num_wheels_ * 2);
        joint_state_msg_.velocity.resize (num_wheels_ * 2);
        for (size_t i = 0; i < num_wheels_; i++) {
            joint_state_msg_.name[i * 2 + 0] = steer_names_[i];
            joint_state_msg_.name[i * 2 + 1] = wheel_names_[i];
        }
    }

    RCLCPP_INFO (this->get_logger (), "swerve_calculator node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "infinite_swerve_mode: %s", infinite_swerve_mode_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %zu", num_wheels_);
    for (size_t i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "wheel_position_xy[%zu] : (%.2f, %.2f)", i, wheel_position_x_[i], wheel_position_y_[i]);
        if (publish_joint_state_) {
            RCLCPP_INFO (this->get_logger (), "  steer_name: %s, wheel_name: %s", steer_names_[i].c_str (), wheel_names_[i].c_str ());
        }
    }
}

void swerve_calculator::command_velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    natto_msgs::msg::Swerve swerve_msg;
    swerve_msg.wheel_angle.resize (num_wheels_, 0.0);
    swerve_msg.wheel_speed.resize (num_wheels_, 0.0);

    double x = msg->twist.linear.x;
    double y = msg->twist.linear.y;
    double z = msg->twist.angular.z;

    for (size_t i = 0; i < num_wheels_; i++) {
        if (x == 0.0 && y == 0.0 && z == 0.0) {
            double vx = -wheel_position_y_[i];
            double vy = +wheel_position_x_[i];

            swerve_msg.wheel_speed[i] = 0.0;
            swerve_msg.wheel_angle[i] = std::atan2 (vy, vx);
        } else {
            double vx    = x - z * wheel_position_y_[i];
            double vy    = y + z * wheel_position_x_[i];
            double v     = std::hypot (vx, vy);
            double angle = v / (2.0 * M_PI * wheel_radius_);

            swerve_msg.wheel_speed[i] = angle;
            swerve_msg.wheel_angle[i] = std::atan2 (vy, vx);
        }
    }
    if (infinite_swerve_mode_) {
        for (size_t i = 0; i < num_wheels_; i++) {
            if (swerve_result_.wheel_angle.size () != num_wheels_ || swerve_result_.wheel_speed.size () != num_wheels_) {
                continue;
            }
            while (swerve_msg.wheel_angle[i] - swerve_result_.wheel_angle[i] > M_PI) {
                swerve_msg.wheel_angle[i] -= 2.0 * M_PI;
            }
            while (swerve_msg.wheel_angle[i] - swerve_result_.wheel_angle[i] < -M_PI) {
                swerve_msg.wheel_angle[i] += 2.0 * M_PI;
            }
            if (std::fabs (swerve_msg.wheel_angle[i] - swerve_result_.wheel_angle[i]) > M_PI / 2.0) {
                swerve_msg.wheel_angle[i] += (swerve_msg.wheel_angle[i] > swerve_result_.wheel_angle[i]) ? -M_PI : M_PI;
                swerve_msg.wheel_speed[i] *= -1.0;
            }
        }
    }
    swerve_command_publisher_->publish (swerve_msg);

    if (publish_joint_state_) {
        for (size_t i = 0; i < num_wheels_; i++) {
            joint_state_msg_.position[i * 2 + 0] = swerve_msg.wheel_angle[i];
            joint_state_msg_.velocity[i * 2 + 1] = swerve_msg.wheel_speed[i] * 2.0 * M_PI;
        }
        joint_state_publisher_->publish (joint_state_msg_);
    }
}

void swerve_calculator::swerve_result_callback (const natto_msgs::msg::Swerve::SharedPtr msg) {
    swerve_result_ = *msg;
}

}  // namespace swerve_calculator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (swerve_calculator::swerve_calculator)