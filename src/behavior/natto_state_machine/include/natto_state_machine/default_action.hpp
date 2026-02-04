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

#ifndef __DEFAULT_ACTION_HPP__
#define __DEFAULT_ACTION_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "natto_msgs/msg/state_action.hpp"
#include "natto_msgs/msg/state_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace default_action {
class default_action : public rclcpp::Node {
   public:
    default_action (const rclcpp::NodeOptions &node_options);

   private:
    double                        frequency_;
    bool                          allow_auto_drive_;
    std::map<std::string, double> joint_tolerances_;

    uint64_t                 set_pose_state_id_;
    bool                     set_pose_goal_sent_;
    double                   xy_tolerance_m_, yaw_tolerance_deg_;
    geometry_msgs::msg::Pose current_pose_, goal_pose_;

    uint64_t     wait_state_id_;
    bool         wait_started_;
    rclcpp::Time wait_start_time_;
    double       wait_duration_sec_;

    sensor_msgs::msg::JointState command_joint_state_, joint_state_;

    uint64_t joint_state_id_;
    bool     joint_state_sent_;

    void state_action_callback (const natto_msgs::msg::StateAction::SharedPtr msg);
    void goal_result_callback (const std_msgs::msg::Bool::SharedPtr msg);
    void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);
    void timer_callback ();

    rclcpp::Publisher<natto_msgs::msg::StateResult>::SharedPtr       state_result_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    goal_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       joint_state_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    joint_state_subscriber_;
    rclcpp::Subscription<natto_msgs::msg::StateAction>::SharedPtr    state_action_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             goal_result_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             allow_auto_drive_subscriber_;
    rclcpp::TimerBase::SharedPtr                                     timer_;
};
}  // namespace default_action

#endif  // __DEFAULT_ACTION_HPP__