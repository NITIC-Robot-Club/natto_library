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

#ifndef __MECANUM_SIMULATOR_HPP__
#define __MECANUM_SIMULATOR_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "natto_msgs/msg/mecanum.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <cmath>

namespace mecanum_simulator {
class mecanum_simulator : public rclcpp::Node {
   public:
    mecanum_simulator (const rclcpp::NodeOptions &node_options);

   private:
    double wheel_radius_;
    size_t num_wheels_;
    double frequency_;
    double wheel_speed_gain_p_, wheel_speed_gain_d_;

    std::vector<double> wheel_position_x_;
    std::vector<double> wheel_position_y_;
    std::vector<double> wheel_angle_;

    std::vector<natto_msgs::msg::Mecanum> received_commands_;
    natto_msgs::msg::Mecanum              command_;
    natto_msgs::msg::Mecanum              result_;
    geometry_msgs::msg::PoseStamped       current_pose_;

    void mecanum_command_callback (const natto_msgs::msg::Mecanum::SharedPtr msg);
    void timer_callback ();

    rclcpp::Publisher<natto_msgs::msg::Mecanum>::SharedPtr        mecanum_result_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr simulation_pose_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Mecanum>::SharedPtr     mecanum_command_subscriber_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
};
}  // namespace mecanum_simulator

#endif  // __MECANUM_SIMULATOR_HPP__
