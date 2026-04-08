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

#ifndef __TWIST_SELECTOR_HPP__
#define __TWIST_SELECTOR_HPP__

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace twist_selector {
class twist_selector : public rclcpp::Node {
   public:
    twist_selector (const rclcpp::NodeOptions &node_options);

   private:
    bool  allow_auto_drive_;
    bool  received_;
    bool  enable_heading_control_;
    float controller_angle_;
    float robot_angle_;

    void allow_auto_drive_callback (const std_msgs::msg::Bool::SharedPtr msg);
    void manual_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void auto_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    selected_twist_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              allow_auto_drive_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr manual_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr auto_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           controller_angle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  current_pose_subscriber_;
    rclcpp::TimerBase::SharedPtr                                      timer_;
};
}  // namespace twist_selector

#endif  // __TWIST_SELECTOR_HPP__