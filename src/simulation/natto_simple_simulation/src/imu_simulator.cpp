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

#include "natto_simple_simulation/imu_simulator.hpp"

namespace imu_simulator {

imu_simulator::imu_simulator (const rclcpp::NodeOptions &node_options) : Node ("imu_simulator", node_options) {
    this->declare_parameter<double> ("frequency", 100.0);
    this->declare_parameter<double> ("noise_stddev_angular_velocity", 0.01);
    this->declare_parameter<double> ("noise_stddev_linear_acceleration", 0.1);

    frequency_                        = this->get_parameter ("frequency").as_double ();
    noise_stddev_angular_velocity_    = this->get_parameter ("noise_stddev_angular_velocity").as_double ();
    noise_stddev_linear_acceleration_ = this->get_parameter ("noise_stddev_linear_acceleration").as_double ();

    imu_publisher_              = this->create_publisher<sensor_msgs::msg::Imu> ("imu", 10);
    simulation_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped> ("simulation_pose", 10, std::bind (&imu_simulator::simulation_pose_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency_), std::bind (&imu_simulator::timer_callback, this));
}

void imu_simulator::simulation_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (is_first_pose_) {
        is_first_pose_ = false;
        last_x_        = msg->pose.position.x;
        last_y_        = msg->pose.position.y;
        last_yaw_      = tf2::getYaw (msg->pose.orientation);
    }
    current_x_   = msg->pose.position.x;
    current_y_   = msg->pose.position.y;
    current_yaw_ = tf2::getYaw (msg->pose.orientation);
}

void imu_simulator::timer_callback () {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp    = this->now ();
    imu_msg.header.frame_id = "base_link";

    double current_vx = (current_x_ - last_x_) * frequency_;
    double current_vy = (current_y_ - last_y_) * frequency_;

    vyaw_ = (current_yaw_ - last_yaw_) * frequency_;

    imu_msg.angular_velocity.z    = vyaw_ + noise_stddev_angular_velocity_ * ((double)rand () / RAND_MAX * 2.0 - 1.0);
    imu_msg.linear_acceleration.x = (vx_ - current_vx) * frequency_ + noise_stddev_linear_acceleration_ * ((double)rand () / RAND_MAX * 2.0 - 1.0);
    imu_msg.linear_acceleration.y = (vy_ - current_vy) * frequency_ + noise_stddev_linear_acceleration_ * ((double)rand () / RAND_MAX * 2.0 - 1.0);
    imu_msg.linear_acceleration.z = GRAVITY_ + noise_stddev_linear_acceleration_ * ((double)rand () / RAND_MAX * 2.0 - 1.0);

    imu_publisher_->publish (imu_msg);

    last_x_   = current_x_;
    last_y_   = current_y_;
    last_yaw_ = current_yaw_;
    vx_       = current_vx;
    vy_       = current_vy;
}

}  // namespace imu_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (imu_simulator::imu_simulator)