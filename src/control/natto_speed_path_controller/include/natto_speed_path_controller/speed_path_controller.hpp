// Copyright 2026 Kazusa Hashimoto
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

#ifndef __SPEED_PATH_CONTROLLER_HPP__
#define __SPEED_PATH_CONTROLLER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "natto_msgs/msg/speed_path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace speed_path_controller {
class speed_path_controller : public rclcpp::Node {
   public:
    speed_path_controller (const rclcpp::NodeOptions &node_options);

   private:
    double position_error_p_, angle_error_p_;
    double position_error_allowance_m_, angle_error_allowance_rad_;

    double goal_position_tolerance_;
    double goal_yaw_tolerance_deg_;
    double goal_speed_tolerance_xy_m_s_;
    double goal_speed_tolerance_yaw_deg_s_;

    natto_msgs::msg::SpeedPath      speed_path_;
    geometry_msgs::msg::PoseStamped current_pose_;

    void speed_path_callback (const natto_msgs::msg::SpeedPath::SharedPtr msg);
    void current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timer_callback ();

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   twist_publisher_;
    rclcpp::Subscription<natto_msgs::msg::SpeedPath>::SharedPtr      speed_path_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                goal_reached_publisher_;
    rclcpp::TimerBase::SharedPtr                                     timer_;
};
}  // namespace speed_path_controller

#endif  // __SPEED_PATH_CONTROLLER_HPP__