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

#ifndef __HOLONOMIC_PURE_PURSUIT_HPP__
#define __HOLONOMIC_PURE_PURSUIT_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace holonomic_pure_pursuit {

class holonomic_pure_pursuit : public rclcpp::Node {
   public:
    holonomic_pure_pursuit (const rclcpp::NodeOptions &options);

   private:
    double lookahead_time_;                  // 速度スケーリング用の時間 [s]
    double min_lookahead_distance_;          // 最小lookahead距離 [m]
    double max_lookahead_distance_;          // 最大lookahead距離 [m]
    double yaw_speed_p_;                     // ヨー速度比例ゲイン
    double curvature_decceleration_p_;       // 曲率減速用の比例ゲイン
    double min_curvature_speed_m_s_;         // 曲率減速用の最低速度 [m/s]
    double yaw_decceleration_p_;             // ヨー減速用の比例ゲイン
    double max_speed_xy_m_s_;                // 最大並進速度
    double min_speed_xy_m_s_;                // 最小並進速度
    double max_speed_yaw_deg_s_;             // 最大回転速度
    double min_speed_yaw_deg_s_;             // 最小回転速度
    double max_acceleration_xy_m_s2_;        // 最大加速度
    double max_acceleration_yaw_deg_s2_;     // 最大角加速度 [deg/s^2]
    double goal_deceleration_m_s2_;          // ゴール減速用の減速度 [m/s^2]
    double goal_deceleration_distance_p_;    // ゴール減速用の距離比例ゲイン
    double goal_position_tolerance_;         // ゴール位置許容誤差 [m]
    double goal_yaw_tolerance_deg_;          // ゴールヨー許容誤差 [deg]
    double goal_speed_tolerance_xy_m_s_;     // ゴール速度許容誤差 [m/s]
    double goal_speed_tolerance_yaw_deg_s_;  // ゴール速度許容誤差 [deg/s]

    double lookahead_distance_;
    double delta_t_s_;

    geometry_msgs::msg::PoseStamped  current_pose_;
    geometry_msgs::msg::TwistStamped last_cmd_vel_;
    nav_msgs::msg::Path              path_;

    void timer_callback ();
    void pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void path_callback (const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr                                     timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    lookahead_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                goal_reached_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             path_subscriber_;
};

}  // namespace holonomic_pure_pursuit

#endif  // __holonomic_pure_pursuit_hpp__