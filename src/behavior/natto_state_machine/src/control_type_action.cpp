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

#include "natto_state_machine/default_action.hpp"

#include <algorithm>

namespace default_action {
namespace detail {

std::string remove_quotes (const std::string &value) {
    if (value.size () >= 2 && value.front () == '"' && value.back () == '"') {
        return value.substr (1, value.size () - 2);
    }
    return value;
}

std::optional<uint8_t> control_type_from_string (const std::string &value) {
    if (value == "current") return natto_msgs::msg::JointControlType::CURRENT;
    if (value == "speed") return natto_msgs::msg::JointControlType::SPEED;
    if (value == "position") return natto_msgs::msg::JointControlType::POSITION;
    if (value == "duty") return natto_msgs::msg::JointControlType::DUTY;
    return std::nullopt;
}

}  // namespace detail

void default_action::handle_set_control_type (const natto_msgs::msg::StateAction::SharedPtr msg) {
    if (msg->arguments_names.size () != msg->arguments_values.size () || msg->arguments_names.empty ()) {
        RCLCPP_WARN (this->get_logger (), "Invalid set_control_type arguments: names and values must be non-empty and have the same size.");
        publish_set_control_type_result (msg->state_id, false);
        return;
    }

    std::vector<control_type_change_t> changes;
    changes.reserve (msg->arguments_names.size ());

    for (size_t i = 0; i < msg->arguments_names.size (); ++i) {
        const std::string joint_name = msg->arguments_names[i];
        const std::string value      = detail::remove_quotes (msg->arguments_values[i]);

        if (joint_name.empty () || std::find_if (changes.begin (), changes.end (), [&joint_name] (const auto &change) { return change.joint_name == joint_name; }) != changes.end ()) {
            RCLCPP_WARN (this->get_logger (), "Invalid or duplicated joint name in set_control_type: '%s'", joint_name.c_str ());
            publish_set_control_type_result (msg->state_id, false);
            return;
        }

        const auto control_type = detail::control_type_from_string (value);
        if (!control_type) {
            RCLCPP_WARN (this->get_logger (), "Unknown control type '%s' for joint '%s'", value.c_str (), joint_name.c_str ());
            publish_set_control_type_result (msg->state_id, false);
            return;
        }

        changes.push_back ({joint_name, *control_type});
    }

    for (const auto &change : changes) {
        natto_msgs::msg::JointControlType control_type;
        control_type.joint_name   = change.joint_name;
        control_type.control_type = change.control_type;
        joint_control_type_publisher_->publish (control_type);
    }

    publish_set_control_type_result (msg->state_id, true);
    RCLCPP_INFO (this->get_logger (), "Published control type changes for %zu joints.", changes.size ());
}

void default_action::publish_set_control_type_result (uint64_t state_id, bool success) {
    natto_msgs::msg::StateResult result;
    result.state_id    = state_id;
    result.action_name = "set_control_type";
    result.success     = success;
    state_result_publisher_->publish (result);
}

}  // namespace default_action
