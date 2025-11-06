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

#ifndef __SWERVE_SIMULATOR_HPP__
#define __SWERVE_SIMULATOR_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "natto_msgs/msg/swerve.hpp"
#include "natto_msgs/msg/map.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <array>
#include <vector>

namespace swerve_simulator {
class swerve_simulator : public rclcpp::Node {
   public:
    swerve_simulator (const rclcpp::NodeOptions &node_options);

   private:
    double wheel_radius_;
    int    num_wheels_, period_ms;
    double angle_gain_p_, angle_gain_d_;
    double speed_gain_p_, speed_gain_d_;
    rclcpp::Time last_time;
    natto_msgs::msg::Map map;

    std::vector<double> wheel_position_x;
    std::vector<double> wheel_position_y;

    std::vector<natto_msgs::msg::Swerve> received_commands;
    natto_msgs::msg::Swerve              result;
    geometry_msgs::msg::PoseStamped      current_pose;
    natto_msgs::msg::Swerve              compute_average_command (const std::vector<natto_msgs::msg::Swerve> &commands) const;
    natto_msgs::msg::Swerve              apply_wheel_response (double dt, const natto_msgs::msg::Swerve &reference_command, const natto_msgs::msg::Swerve &latest_command, const natto_msgs::msg::Swerve &current_state) const;
    void                                 publish_swerve_result (const natto_msgs::msg::Swerve &swerve_state);
    std::array<double, 3>                estimate_body_velocity (const natto_msgs::msg::Swerve &wheel_state) const;
    geometry_msgs::msg::Pose             integrate_pose (const double vx, const double vy, const double vz, const double dt);
    void                                 publish_pose (const geometry_msgs::msg::Pose &new_pose);
    void                                 broadcast_transform (const geometry_msgs::msg::Pose &new_pose);

    void swerve_command_callback (const natto_msgs::msg::Swerve::SharedPtr msg);
    void map_callback (const natto_msgs::msg::Map::SharedPtr msg);
    void timer_callback ();

    rclcpp::Publisher<natto_msgs::msg::Swerve>::SharedPtr         swerve_result_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr simulation_pose_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Swerve>::SharedPtr      swerve_command_subscriber_;
    rclcpp::Subscription<natto_msgs::msg::Map>::SharedPtr         map_subscriber_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
};
}  // namespace swerve_simulator

#endif  // __SWERVE_SIMULATOR_HPP__
