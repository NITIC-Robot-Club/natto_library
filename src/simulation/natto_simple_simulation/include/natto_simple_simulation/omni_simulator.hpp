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
#include "natto_msgs/msg/omni.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/transform_broadcaster.h>

namespace omni_simulator {
class omni_simulator : public rclcpp::Node {
   public:
    omni_simulator (const rclcpp::NodeOptions &node_options);

   private:
    double wheel_radius_;
    int    num_wheels_;
    double frequency_;
    double wheel_speed_gain_p_, wheel_speed_gain_d_;

    std::vector<double> wheel_position_x;
    std::vector<double> wheel_position_y;
    std::vector<double> wheel_angle;

    std::vector<natto_msgs::msg::Omni> received_commands;
    natto_msgs::msg::Omni              command;
    natto_msgs::msg::Omni              result;
    geometry_msgs::msg::PoseStamped    current_pose;

    void omni_command_callback (const natto_msgs::msg::Omni::SharedPtr msg);
    void timer_callback ();

    rclcpp::Publisher<natto_msgs::msg::Omni>::SharedPtr           omni_result_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr simulation_pose_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Omni>::SharedPtr        omni_command_subscriber_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
};
}  // namespace omni_simulator

#endif  // __OMNI_SIMULATOR_HPP__