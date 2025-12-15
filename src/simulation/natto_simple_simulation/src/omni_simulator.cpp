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

#include "natto_simple_simulation/omni_simulator.hpp"

namespace omni_simulator {

omni_simulator::omni_simulator (const rclcpp::NodeOptions &node_options) : Node ("omni_simulator", node_options) {
    omni_result_publisher_     = this->create_publisher<natto_msgs::msg::Omni> ("omni_result", 10);
    simulation_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("simulation_pose", 10);
    omni_command_subscriber_   = this->create_subscription<natto_msgs::msg::Omni> ("omni_command", 10, std::bind (&omni_simulator::omni_command_callback, this, std::placeholders::_1));
    tf_broadcaster_            = std::make_shared<tf2_ros::TransformBroadcaster> (this);

    wheel_radius_                 = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_position_x_             = this->declare_parameter<std::vector<double>> ("wheel_position_x", {0.5, -0.5, -0.5, 0.5});
    wheel_position_y_             = this->declare_parameter<std::vector<double>> ("wheel_position_y", {0.5, 0.5, -0.5, -0.5});
    wheel_angle_                  = this->declare_parameter<std::vector<double>> ("wheel_angle_deg", {-45.0, 45.0, 135.0, -135.0});
    wheel_speed_gain_p_           = this->declare_parameter<double> ("wheel_speed_gain_p", 300.0);
    wheel_speed_gain_d_           = this->declare_parameter<double> ("wheel_speed_gain_d", 100.0);
    frequency_                    = this->declare_parameter<double> ("frequency", 1000.0);
    current_pose_.pose.position.x = this->declare_parameter<double> ("initial_pose_x", 1.0);
    current_pose_.pose.position.y = this->declare_parameter<double> ("initial_pose_y", 1.0);
    double yaw                    = this->declare_parameter<double> ("initial_pose_yaw_deg", 0.0);

    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, yaw * M_PI / 180.0);
    current_pose_.pose.orientation = tf2::toMsg (q);

    num_wheels_ = wheel_position_x_.size ();
    if (wheel_position_y_.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_position_x and wheel_position_y must have the same size.");
        throw std::runtime_error ("wheel_position_x and wheel_position_y must have the same size.");
    }

    RCLCPP_INFO (this->get_logger (), "omni_simulator node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency_);
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %d", num_wheels_);
    for (int i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "wheel_position_xy[%d]: (%.2f, %.2f), wheel_angle_deg[%d]: %.2f", i, wheel_position_x_[i], wheel_position_y_[i], i, wheel_angle_[i]);
    }
    RCLCPP_INFO (this->get_logger (), "wheel_speed_gain_p: %.2f, wheel_speed_gain_d: %.2f", wheel_speed_gain_p_, wheel_speed_gain_d_);

    command_.wheel_speed.resize (num_wheels_, 0.0);
    result_.wheel_speed.resize (num_wheels_, 0.0);

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency_), std::bind (&omni_simulator::timer_callback, this));
}

void omni_simulator::omni_command_callback (const natto_msgs::msg::Omni::SharedPtr msg) {
    received_commands_.push_back (*msg);
}

void omni_simulator::timer_callback () {
    if (received_commands_.empty ()) {
        received_commands_.push_back (result_);
    }
    natto_msgs::msg::Omni command_sum;
    command_sum.wheel_speed.resize (num_wheels_, 0.0);
    for (int i = 0; i < received_commands_.size (); i++) {
        for (int j = 0; j < num_wheels_; j++) {
            if (received_commands_[i].wheel_speed.size () != num_wheels_) {
                RCLCPP_FATAL (this->get_logger (), "Received command size does not match number of wheels.");
            }
            command_sum.wheel_speed[j] += received_commands_[i].wheel_speed[j];
        }
    }
    for (int j = 0; j < num_wheels_; j++) {
        command_.wheel_speed[j] = command_sum.wheel_speed[j] / received_commands_.size ();
    }

    for (int i = 0; i < num_wheels_; i++) {
        double speed_error = command_.wheel_speed[i] - result_.wheel_speed[i];  
        double speed_adjustment = wheel_speed_gain_p_ * speed_error - wheel_speed_gain_d_ * (result_.wheel_speed[i] - command_.wheel_speed[i]);

        result_.wheel_speed[i] += speed_adjustment / frequency_;

        if (abs (received_commands_.back ().wheel_speed[i] - result_.wheel_speed[i]) < 0.01) {
            // +の目標から-0.0を目標にしたときなどの見た目の問題
            // 誤差が小さいときは見た目のために一致させる
            result_.wheel_speed[i] = received_commands_.back ().wheel_speed[i];
        }
    }
    omni_result_publisher_->publish (result_);
    received_commands_.clear ();

    double ATA[3][3] = {};  // A^T * A
    double ATb[3]    = {};  // A^T * b

    for (int i = 0; i < num_wheels_; i++) {
        double angle = wheel_angle_[i] * M_PI / 180.0;
        double speed = result_.wheel_speed[i] * 2.0 * M_PI * wheel_radius_;

        double c     = std::cos (angle);
        double s     = std::sin (angle);
        double ax[3] = {c, s, (-wheel_position_y_[i] * c + wheel_position_x_[i] * s)};
        double b     = speed;

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                ATA[row][col] += ax[row] * ax[col];
            }
            ATb[row] += ax[row] * b;
        }
    }

    double A[3][4] = {
        {ATA[0][0], ATA[0][1], ATA[0][2], ATb[0]},
        {ATA[1][0], ATA[1][1], ATA[1][2], ATb[1]},
        {ATA[2][0], ATA[2][1], ATA[2][2], ATb[2]}
    };

    for (int i = 0; i < 3; ++i) {
        double pivot = A[i][i];
        if (std::fabs (pivot) < 1e-9) return;
        for (int j = i; j < 4; ++j) A[i][j] /= pivot;
        for (int k = 0; k < 3; ++k) {
            if (k == i) continue;
            double factor = A[k][i];
            for (int j = i; j < 4; ++j) A[k][j] -= factor * A[i][j];
        }
    }
    double vx = A[0][3];
    double vy = A[1][3];
    double vz = A[2][3];

    double yaw      = tf2::getYaw (current_pose_.pose.orientation);
    double vx_world = vx * cos (yaw) - vy * sin (yaw);
    double vy_world = vx * sin (yaw) + vy * cos (yaw);
    current_pose_.pose.position.x += vx_world / frequency_;
    current_pose_.pose.position.y += vy_world / frequency_;
    yaw += vz / frequency_;

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

}  // namespace omni_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (omni_simulator::omni_simulator)