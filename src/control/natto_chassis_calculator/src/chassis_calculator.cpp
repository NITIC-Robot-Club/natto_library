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

#include "natto_chassis_calculator/chassis_calculator.hpp"

namespace chassis_calculator {

chassis_calculator::chassis_calculator (const rclcpp::NodeOptions &node_options) : Node ("chassis_calculator", node_options) {
    command_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::QoS (10).best_effort ());
    joint_state_subscriber_        = this->create_subscription<sensor_msgs::msg::JointState> ("joint_states", rclcpp::QoS (10).best_effort (), std::bind (&chassis_calculator::joint_state_callback, this, std::placeholders::_1));
    twist_command_subscriber_      = this->create_subscription<geometry_msgs::msg::TwistStamped> ("command_velocity", 10, std::bind (&chassis_calculator::command_velocity_callback, this, std::placeholders::_1));

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    chassis_type_ = this->declare_parameter<std::string> ("chassis_type", "");
    wheel_radius_ = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_names_  = this->declare_parameter<std::vector<std::string>> ("wheel_names", {""});
    num_wheels_   = wheel_names_.size ();

    wheel_base_names_ = this->declare_parameter<std::vector<std::string>> ("wheel_base_names", {""});
    if (wheel_base_names_.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "Size of wheel_base_names and wheel_names must be the same.");
        throw std::runtime_error ("Invalid parameters: wheel_base_names and wheel_names size mismatch.");
    }

    if (chassis_type_ == "swerve") {
        infinite_swerve_mode_ = this->declare_parameter<bool> ("infinite_swerve_mode", false);

        command_joint_state_.header.frame_id = "base_link";
        command_joint_state_.name.resize (num_wheels_ * 2);
        command_joint_state_.position.resize (num_wheels_ * 2);
        command_joint_state_.velocity.resize (num_wheels_ * 2);
        for (size_t i = 0; i < num_wheels_; i++) {
            command_joint_state_.name[i]               = wheel_names_[i];
            command_joint_state_.name[i + num_wheels_] = wheel_base_names_[i];
        }
    } else if (chassis_type_ == "omni") {
        command_joint_state_.header.frame_id = "base_link";
        command_joint_state_.name.resize (num_wheels_);
        command_joint_state_.position.resize (num_wheels_);
        command_joint_state_.velocity.resize (num_wheels_);
        for (size_t i = 0; i < num_wheels_; i++) {
            command_joint_state_.name[i] = wheel_names_[i];
        }
    } else {
        RCLCPP_ERROR (this->get_logger (), "Unsupported chassis_type: %s", chassis_type_.c_str ());
        throw std::runtime_error ("Invalid parameter: unsupported chassis_type.");
    }

    RCLCPP_INFO (this->get_logger (), "chassis_calculator node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "chassis_type: %s", chassis_type_.c_str ());
    if (chassis_type_ == "swerve") {
        RCLCPP_INFO (this->get_logger (), "infinite_swerve_mode: %s", infinite_swerve_mode_ ? "true" : "false");
    }
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "num_wheels: %zu", num_wheels_);
}

void chassis_calculator::command_velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    command_joint_state_.header.stamp = this->now ();

    double x = msg->twist.linear.x;
    double y = msg->twist.linear.y;
    double z = msg->twist.angular.z;

    if (chassis_type_ == "swerve") {
        for (size_t i = 0; i < num_wheels_; i++) {
            double wheel_position_x;
            double wheel_position_y;

            try {
                geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform ("base_link", wheel_base_names_[i] + "_link", tf2::TimePointZero);
                wheel_position_x                                = tf_stamped.transform.translation.x;
                wheel_position_y                                = tf_stamped.transform.translation.y;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 3000, "Could not get transform from %s to base_link: %s", wheel_base_names_[i].c_str (), ex.what ());
                return;
            }

            if (x == 0.0 && y == 0.0 && z == 0.0) {
                double vx = -wheel_position_y;
                double vy = +wheel_position_x;

                command_joint_state_.velocity[i]               = 0.0;
                command_joint_state_.position[i + num_wheels_] = std::atan2 (vy, vx);
            } else {
                double vx    = x - z * wheel_position_y;
                double vy    = y + z * wheel_position_x;
                double v     = std::hypot (vx, vy);
                double speed = v / wheel_radius_;

                command_joint_state_.velocity[i]               = speed;
                command_joint_state_.position[i + num_wheels_] = std::atan2 (vy, vx);
            }
        }
        if (infinite_swerve_mode_) {
            for (size_t i = 0; i < num_wheels_; i++) {
                for (size_t j = 0; j < joint_state_.name.size (); j++) {
                    if (joint_state_.name[j] == wheel_base_names_[i]) {
                        double current_angle = joint_state_.position[j];
                        while (command_joint_state_.position[i + num_wheels_] - current_angle > M_PI) {
                            command_joint_state_.position[i + num_wheels_] -= 2.0 * M_PI;
                        }
                        while (command_joint_state_.position[i + num_wheels_] - current_angle < -M_PI) {
                            command_joint_state_.position[i + num_wheels_] += 2.0 * M_PI;
                        }
                        if (std::abs (command_joint_state_.position[i + num_wheels_] - current_angle) > M_PI / 2.0) {
                            command_joint_state_.position[i + num_wheels_] += (command_joint_state_.position[i + num_wheels_] > current_angle) ? -M_PI : M_PI;
                            command_joint_state_.velocity[i] *= -1.0;
                        }
                        break;
                    }
                }
            }
        }
        command_joint_state_publisher_->publish (command_joint_state_);
    } else if (chassis_type_ == "omni") {
        for (size_t i = 0; i < num_wheels_; i++) {
            double wheel_position_x;
            double wheel_position_y;
            double wheel_angle;

            try {
                geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform ("base_link", wheel_base_names_[i] + "_link", tf2::TimePointZero);

                wheel_position_x = tf_stamped.transform.translation.x;
                wheel_position_y = tf_stamped.transform.translation.y;
                wheel_angle      = tf2::getYaw (tf_stamped.transform.rotation);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 3000, "Could not get transform from %s to base_link: %s", wheel_base_names_[i].c_str (), ex.what ());
                return;
            }
            double wheel_vx        = x - z * wheel_position_y;
            double wheel_vy        = y + z * wheel_position_x;
            double wheel_speed     = std::sqrt (wheel_vx * wheel_vx + wheel_vy * wheel_vy);
            double wheel_direction = std::atan2 (wheel_vy, wheel_vx);

            double adjusted_wheel_speed = wheel_speed * std::cos (wheel_direction - wheel_angle);

            command_joint_state_.velocity[i] = adjusted_wheel_speed / wheel_radius_;
        }
        command_joint_state_publisher_->publish (command_joint_state_);
    }
}

void chassis_calculator::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_state_ = *msg;
}
}  // namespace chassis_calculator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (chassis_calculator::chassis_calculator)