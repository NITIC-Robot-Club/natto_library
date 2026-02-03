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

#include "natto_joint_state/joint_state_rule.hpp"

namespace joint_state_rule {

joint_state_rule::joint_state_rule (const rclcpp::NodeOptions &node_options) : Node ("joint_state_rule", rclcpp::NodeOptions (node_options).allow_undeclared_parameters (true).automatically_declare_parameters_from_overrides (true)) {
    fixed_command_joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState> ("fixed_command_joint_states", rclcpp::SensorDataQoS ());
    joint_states_subscriber_              = this->create_subscription<sensor_msgs::msg::JointState> ("joint_states", rclcpp::SensorDataQoS (), std::bind (&joint_state_rule::joint_states_callback, this, std::placeholders::_1));
    command_joint_states_subscriber_      = this->create_subscription<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::SensorDataQoS (), std::bind (&joint_state_rule::command_joint_states_callback, this, std::placeholders::_1));

    parse_rules ();
}

JointRange joint_state_rule::parse_joint_range (const std::string &base_key) const {
    JointRange jr;
    jr.joint_name = this->get_parameter (base_key + ".joint_name").as_string ();
    auto range    = this->get_parameter (base_key + ".range").as_double_array ();
    if (range.size () != 2) {
        throw std::runtime_error ("range must have exactly 2 elements: " + base_key);
    }
    jr.min = range[0];
    jr.max = range[1];
    return jr;
}

void joint_state_rule::parse_rules () {
    std::map<std::string, rclcpp::Parameter> params;
    this->get_parameters ("joint_state_rule", params);
    rules_.clear ();

    if (params.empty ()) {
        RCLCPP_INFO (this->get_logger (), "No joint_state_rule parameters found");
        return;
    }

    std::set<std::string> rule_names;
    for (const auto &kv : params) {
        auto pos = kv.first.find ('.');
        if (pos != std::string::npos) {
            rule_names.insert (kv.first.substr (0, pos));
        }
    }

    for (const auto &rule_name : rule_names) {
        Rule                  rule;
        std::set<std::string> if_names;
        for (const auto &kv : params) {
            const std::string prefix = rule_name + ".if_";
            if (kv.first.rfind (prefix, 0) == 0) {
                auto rest = kv.first.substr (prefix.size ());
                auto pos  = rest.find ('.');
                if (pos != std::string::npos) {
                    if_names.insert (rest.substr (0, pos));
                }
            }
        }
        for (const auto &if_name : if_names) {
            rule.if_conditions.push_back (parse_joint_range ("joint_state_rule." + rule_name + ".if_" + if_name));
        }
        rule.then_condition = parse_joint_range ("joint_state_rule." + rule_name + ".then");
        rules_.emplace (rule_name, rule);
    }
    RCLCPP_INFO (this->get_logger (), "Loaded %zu joint_state_rule rules", rules_.size ());
}

void joint_state_rule::joint_states_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    current_joint_states_ = msg;
}

void joint_state_rule::command_joint_states_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    sensor_msgs::msg::JointState fixed_command = *msg;

    if (rules_.empty ()) {
        fixed_command_joint_states_publisher_->publish (fixed_command);
        return;
    }

    for (const auto &kv : rules_) {
        const Rule &rule       = kv.second;
        bool        need_clamp = true;
        for (const auto &if_condition : rule.if_conditions) {
            bool need_clamp_inner = false;
            auto it_command       = std::find (msg->name.begin (), msg->name.end (), if_condition.joint_name);
            if (it_command != msg->name.end ()) {
                size_t index = static_cast<size_t> (std::distance (msg->name.begin (), it_command));
                if (if_condition.min <= msg->position[index] && msg->position[index] <= if_condition.max) {
                    need_clamp_inner = true;
                    break;
                }
            }

            if (current_joint_states_) {
                auto it_current = std::find (current_joint_states_->name.begin (), current_joint_states_->name.end (), if_condition.joint_name);
                if (it_current != current_joint_states_->name.end ()) {
                    size_t index = static_cast<size_t> (std::distance (current_joint_states_->name.begin (), it_current));
                    if (if_condition.min <= current_joint_states_->position[index] && current_joint_states_->position[index] <= if_condition.max) {
                        need_clamp_inner = true;
                        break;
                    }
                }
            }
            if (!need_clamp_inner) {
                need_clamp = false;
                break;
            }
        }
        if (need_clamp) {
            auto it_fixed = std::find (fixed_command.name.begin (), fixed_command.name.end (), rule.then_condition.joint_name);
            if (it_fixed != fixed_command.name.end ()) {
                size_t index                  = static_cast<size_t> (std::distance (fixed_command.name.begin (), it_fixed));
                double value                  = fixed_command.position[index];
                fixed_command.position[index] = std::clamp (value, rule.then_condition.min, rule.then_condition.max);
            }
        }
    }
    fixed_command_joint_states_publisher_->publish (fixed_command);
}

}  // namespace joint_state_rule

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (joint_state_rule::joint_state_rule)