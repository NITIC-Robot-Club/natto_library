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

#ifndef __JOINT_STATES_MERGER_HPP__
#define __JOINT_STATES_MERGER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

namespace joint_state_merger {
class joint_state_merger : public rclcpp::Node {
   public:
    joint_state_merger (const rclcpp::NodeOptions &node_options);

   private:
    sensor_msgs::msg::JointState merged_joint_states_;

    void joint_states_callback (const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr                 merged_joint_state_publisher_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_state_subscribers_;
};
}  // namespace joint_state_merger

#endif  // __JOINT_STATES_MERGER_HPP__