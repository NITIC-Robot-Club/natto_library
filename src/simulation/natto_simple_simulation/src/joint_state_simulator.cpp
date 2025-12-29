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

#include "natto_simple_simulation/joint_state_simulator.hpp"

namespace joint_state_simulator {

joint_state_simulator::joint_state_simulator (const rclcpp::NodeOptions &node_options) : Node ("joint_state_simulator", node_options) {
    joint_state_publisher_     = this->create_publisher<sensor_msgs::msg::JointState> ("joint_states", rclcpp::QoS (rclcpp::KeepLast (10)).best_effort ());
    simulation_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("simulation_pose", 10);
    command_joint_state_subscriber_ =
        this->create_subscription<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::QoS (rclcpp::KeepLast (10)).best_effort (), std::bind (&joint_state_simulator::command_joint_state_callback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster> (this);

    chassis_type_ = this->declare_parameter<std::string> ("chassis_type", "");
    wheel_names_  = this->declare_parameter<std::vector<std::string>> ("wheel_names", {""});
    wheel_radius_ = this->declare_parameter<double> ("wheel_radius", 0.05);
    frequency_    = this->declare_parameter<double> ("frequency", 1000.0);

    num_wheels_ = wheel_names_.size ();
    if (chassis_type_ == "") {
        RCLCPP_ERROR (this->get_logger (), "chassis_type parameter is required.");
        throw std::runtime_error ("chassis_type parameter is required.");
    }

    if (chassis_type_ == "swerve") {
        infinite_swerve_mode_ = this->declare_parameter<bool> ("infinite_swerve_mode", false);
        steer_names_          = this->declare_parameter<std::vector<std::string>> ("steer_names", {""});
        if (steer_names_.size () != num_wheels_) {
            RCLCPP_ERROR (this->get_logger (), "steer_names size must be equal to number of wheels.");
            throw std::runtime_error ("steer_names size must be equal to number of wheels.");
        }
    } else if (chassis_type_ == "omni" || chassis_type_ == "mecanum") {
        wheel_angle_ = this->declare_parameter<std::vector<double>> ("wheel_angle_deg", {0.0});
        if (wheel_angle_.size () != num_wheels_) {
            RCLCPP_ERROR (this->get_logger (), "wheel_angle_deg size must be equal to number of wheels.");
            throw std::runtime_error ("wheel_angle_deg size must be equal to number of wheels.");
        }
    } else {
        RCLCPP_ERROR (this->get_logger (), "Unsupported chassis_type: %s", chassis_type_.c_str ());
        throw std::runtime_error ("Unsupported chassis_type: " + chassis_type_);
    }

    joint_names_ = this->declare_parameter<std::vector<std::string>> ("joint_names", {""});
    num_joints_  = joint_names_.size ();

    control_modes_ = this->declare_parameter<std::vector<std::string>> ("control_modes", {""});
    if (control_modes_.size () != num_joints_) {
        RCLCPP_ERROR (this->get_logger (), "control_modes size must be equal to number of joints.");
        throw std::runtime_error ("control_modes size must be equal to number of joints.");
    }
    for (size_t i = 0; i < num_joints_; i++) {
        if (control_modes_[i] != "position" && control_modes_[i] != "speed") {
            RCLCPP_ERROR (this->get_logger (), "Unsupported control_mode: %s", control_modes_[i].c_str ());
            throw std::runtime_error ("Unsupported control_mode: " + control_modes_[i]);
        }
    }

    initial_positions_ = this->declare_parameter<std::vector<double>> ("initial_positions", {0.0});
    if (initial_positions_.size () != joint_names_.size ()) {
        RCLCPP_ERROR (this->get_logger (), "initial_positions size must be equal to number of joints.");
        throw std::runtime_error ("initial_positions size must be equal to number of joints.");
    }

    joint_position_tau_ = this->declare_parameter<std::vector<double>> ("joint_position_tau", {0.5});
    if (joint_position_tau_.size () != joint_names_.size ()) {
        RCLCPP_ERROR (this->get_logger (), "joint_position_tau size must be equal to number of joints.");
        throw std::runtime_error ("joint_position_tau size must be equal to number of joints.");
    }

    joint_velocity_tau_ = this->declare_parameter<std::vector<double>> ("joint_velocity_tau", {0.1});
    if (joint_velocity_tau_.size () != joint_names_.size ()) {
        RCLCPP_ERROR (this->get_logger (), "joint_velocity_tau size must be equal to number of joints.");
        throw std::runtime_error ("joint_velocity_tau size must be equal to number of joints.");
    }

    joint_velocity_max_ = this->declare_parameter<std::vector<double>> ("joint_velocity_max", {10.0});
    if (joint_velocity_max_.size () != joint_names_.size ()) {
        RCLCPP_ERROR (this->get_logger (), "joint_velocity_max size must be equal to number of joints.");
        throw std::runtime_error ("joint_velocity_max size must be equal to number of joints.");
    }

    current_.name     = joint_names_;
    current_.position = initial_positions_;
    current_.velocity.resize (joint_names_.size (), 0.0);
    current_.effort.resize (joint_names_.size (), 0.0);

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency_), std::bind (&joint_state_simulator::timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "joint_state_simulator node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "chassis_type: %s", chassis_type_.c_str ());
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency_);
    RCLCPP_INFO (this->get_logger (), "initial pose: (%f, %f, %f)", initial_x_, initial_y_, initial_yaw_);
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %zu", num_wheels_);
    for (size_t i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "wheel_name[%zu]: %s", i, wheel_names_[i].c_str ());
    }
    RCLCPP_INFO (this->get_logger (), "Number of joints: %zu", num_joints_);
    for (size_t i = 0; i < num_joints_; i++) {
        RCLCPP_INFO (this->get_logger (), "joint_name[%zu]: %s control_mode[%zu]: %s", i, joint_names_[i].c_str (), i, control_modes_[i].c_str ());
    }
}

void joint_state_simulator::command_joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    command_ = *msg;
}

void joint_state_simulator::timer_callback () {
    const double dt = 1.0 / frequency_;

    for (size_t i = 0; i < num_joints_; i++) {
        auto it = std::find (command_.name.begin (), command_.name.end (), joint_names_[i]);

        if (it == command_.name.end ()) {
            continue;
        }

        size_t index = static_cast<size_t> (std::distance (command_.name.begin (), it));

        if (control_modes_[i] == "position") {
            const double target_position  = command_.position[index];
            const double error            = target_position - current_.position[i];
            double       command_velocity = error / joint_position_tau_[i];

            command_velocity     = std::clamp (command_velocity, -joint_velocity_max_[i], joint_velocity_max_[i]);
            current_.velocity[i] = command_velocity;
            current_.position[i] += command_velocity * dt;
        } else if (control_modes_[i] == "speed") {
            const double target_velocity = command_.velocity[index];
            const double acceleration    = (target_velocity - current_.velocity[i]) / joint_velocity_tau_[i];

            current_.velocity[i] += acceleration / frequency_;
            current_.velocity[i] = std::clamp (current_.velocity[i], -joint_velocity_max_[i], joint_velocity_max_[i]);
            current_.position[i] += current_.velocity[i] / frequency_;
        }
    }

    current_.header.stamp = this->now ();
    joint_state_publisher_->publish (current_);
}

}  // namespace joint_state_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (joint_state_simulator::joint_state_simulator)