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
#ifndef __JOINT_STATES_RULE_HPP__
#define __JOINT_STATES_RULE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

namespace joint_states_rule {

struct JointRange {
    std::string joint_name;
    double      min;
    double      max;
};

struct Rule {
    std::vector<JointRange> if_conditions;
    JointRange              then_condition;
};

using RuleMap = std::map<std::string, Rule>;

class joint_states_rule : public rclcpp::Node {
   public:
    joint_states_rule (const rclcpp::NodeOptions &node_options);

   private:
    sensor_msgs::msg::JointState::SharedPtr current_joint_states_;

    RuleMap    rules_;
    void       parse_rules ();
    JointRange parse_joint_range (const std::string &base_key) const;

    void joint_states_callback (const sensor_msgs::msg::JointState::SharedPtr msg);
    void command_joint_states_callback (const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    fixed_command_joint_states_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_joint_states_subscriber_;
};
}  // namespace joint_states_rule

#endif  // __JOINT_STATES_RULE_HPP__