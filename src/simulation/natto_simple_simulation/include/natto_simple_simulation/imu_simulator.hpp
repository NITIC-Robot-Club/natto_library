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

#ifndef __IMU_SIMULATOR_HPP__
#define __IMU_SIMULATOR_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace imu_simulator {
class imu_simulator : public rclcpp::Node {
   public:
    imu_simulator (const rclcpp::NodeOptions &node_options);

   private:
    double       frequency_;
    double       noise_stddev_angular_velocity_;
    double       noise_stddev_linear_acceleration_;
    const double GRAVITY_ = 9.80665;

    bool  is_first_pose_ = true;

    double vx_, vy_, vyaw_;
    double last_x_, last_y_, last_yaw_;
    double current_x_, current_y_, current_yaw_;

    void simulation_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timer_callback ();

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              imu_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr simulation_pose_subscriber_;
    rclcpp::TimerBase::SharedPtr                                     timer_;
};
}  // namespace imu_simulator

#endif  // __IMU_SIMULATOR_HPP__