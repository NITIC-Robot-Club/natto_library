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

#include "natto_simple_simulation/two_wheel_simulator.hpp"

namespace two_wheel_simulator {

two_wheel_simulator::two_wheel_simulator (const rclcpp::NodeOptions &node_options) : Node ("two_wheel_simulator", node_options) {
    two_wheel_result_publisher_     = this->create_publisher<natto_msgs::msg::TwoWheel> ("two_wheel_result", 10);
    simulation_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("simulation_pose", 10);
    two_wheel_command_subscriber_   = this->create_subscription<natto_msgs::msg::TwoWheel> ("two_wheel_command", 10, std::bind (&two_wheel_simulator::two_wheel_command_callback, this, std::placeholders::_1));
    tf_broadcaster_            = std::make_shared<tf2_ros::TransformBroadcaster> (this);

    wheel_radius_                 = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_base_                   = this->declare_parameter<double> ("wheel_base", 0.5);
    wheel_speed_gain_p_           = this->declare_parameter<double> ("wheel_speed_gain_p", 300.0);
    wheel_speed_gain_d_           = this->declare_parameter<double> ("wheel_speed_gain_d", 100.0);
    frequency_                    = this->declare_parameter<double> ("frequency", 1000.0);
    current_pose_.pose.position.x = this->declare_parameter<double> ("initial_pose_x", 1.0);
    current_pose_.pose.position.y = this->declare_parameter<double> ("initial_pose_y", 1.0);
    double yaw                    = this->declare_parameter<double> ("initial_pose_yaw_deg", 0.0);

    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, yaw * M_PI / 180.0);
    current_pose_.pose.orientation = tf2::toMsg (q);

    RCLCPP_INFO (this->get_logger (), "two_wheel_simulator node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency_);
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "wheel_base: %.2f m", wheel_base_);
    RCLCPP_INFO (this->get_logger (), "wheel_speed_gain_p: %.2f, wheel_speed_gain_d: %.2f", wheel_speed_gain_p_, wheel_speed_gain_d_);

    command_.wheel_speed.resize (2, 0.0);
    result_.wheel_speed.resize (2, 0.0);

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency_), std::bind (&two_wheel_simulator::timer_callback, this));
}

void two_wheel_simulator::two_wheel_command_callback (const natto_msgs::msg::TwoWheel::SharedPtr msg) {
    received_commands_.push_back (*msg);
}

void two_wheel_simulator::timer_callback () {
    if (received_commands_.empty ()) {
        received_commands_.push_back (result_);
    }
    natto_msgs::msg::TwoWheel command_sum;
    command_sum.wheel_speed.resize (2, 0.0);
    for (size_t i = 0; i < received_commands_.size (); i++) {
        if (received_commands_[i].wheel_speed.size () != 2) {
            RCLCPP_FATAL (this->get_logger (), "Received command size does not match number of wheels (2).");
        }
        for (size_t j = 0; j < 2; j++) {
            command_sum.wheel_speed[j] += received_commands_[i].wheel_speed[j];
        }
    }
    for (size_t j = 0; j < 2; j++) {
        command_.wheel_speed[j] = command_sum.wheel_speed[j] / static_cast<double> (received_commands_.size ());
    }

    for (size_t i = 0; i < 2; i++) {
        double speed_error      = command_.wheel_speed[i] - result_.wheel_speed[i];
        double speed_adjustment = wheel_speed_gain_p_ * speed_error - wheel_speed_gain_d_ * (result_.wheel_speed[i] - command_.wheel_speed[i]);

        result_.wheel_speed[i] += speed_adjustment / frequency_;

        if (abs (received_commands_.back ().wheel_speed[i] - result_.wheel_speed[i]) < 0.01) {
            // +の目標から-0.0を目標にしたときなどの見た目の問題
            // 誤差が小さいときは見た目のために一致させる
            result_.wheel_speed[i] = received_commands_.back ().wheel_speed[i];
        }
    }
    two_wheel_result_publisher_->publish (result_);
    received_commands_.clear ();

    // Calculate linear and angular velocities from wheel speeds
    double v_left = result_.wheel_speed[0] * 2.0 * M_PI * wheel_radius_;
    double v_right = result_.wheel_speed[1] * 2.0 * M_PI * wheel_radius_;

    // Differential drive kinematics
    double linear_velocity = (v_left + v_right) / 2.0;
    double angular_velocity = (v_right - v_left) / wheel_base_;

    double yaw      = tf2::getYaw (current_pose_.pose.orientation);
    double vx_world = linear_velocity * cos (yaw);
    double vy_world = linear_velocity * sin (yaw);
    current_pose_.pose.position.x += vx_world / frequency_;
    current_pose_.pose.position.y += vy_world / frequency_;
    yaw += angular_velocity / frequency_;

    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, yaw);
    current_pose_.pose.orientation.x = q.x ();
    current_pose_.pose.orientation.y = q.y ();
    current_pose_.pose.orientation.z = q.z ();
    current_pose_.pose.orientation.w = q.w ();
    current_pose_.header.stamp       = this->now ();
    current_pose_.header.frame_id    = "map";
    simulation_pose_publisher_->publish (current_pose_);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = this->now ();
    tf_msg.header.frame_id         = "map";
    tf_msg.child_frame_id          = "simulation";
    tf_msg.transform.translation.x = current_pose_.pose.position.x;
    tf_msg.transform.translation.y = current_pose_.pose.position.y;
    tf_msg.transform.translation.z = current_pose_.pose.position.z;
    tf_msg.transform.rotation      = current_pose_.pose.orientation;

    tf_broadcaster_->sendTransform (tf_msg);
}

}  // namespace two_wheel_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (two_wheel_simulator::two_wheel_simulator)
