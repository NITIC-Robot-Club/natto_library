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
#include <set>
#include <utility>

namespace default_action::detail {

static std::string trim (std::string value) {
    const auto first = value.find_first_not_of (" \t\n\r");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of (" \t\n\r");
    return value.substr (first, last - first + 1);
}

static std::string unquote (std::string value) {
    value = trim (std::move (value));
    if (value.size () >= 2 && ((value.front () == '\'' && value.back () == '\'') || (value.front () == '"' && value.back () == '"'))) {
        return value.substr (1, value.size () - 2);
    }
    return value;
}

}  // namespace default_action::detail

namespace default_action {

void default_action::handle_get_origin (const natto_msgs::msg::StateAction::SharedPtr msg) {
    if (msg->arguments_names.size () != msg->arguments_values.size ()) {
        RCLCPP_WARN (this->get_logger (), "get_origin received mismatched argument names and values.");
        natto_msgs::msg::StateResult result;
        result.state_id    = msg->state_id;
        result.action_name = "get_origin";
        result.success     = false;
        state_result_publisher_->publish (result);
        return;
    }

    if (origin_action_active_) {
        if (origin_state_id_ == msg->state_id) {
            RCLCPP_DEBUG (this->get_logger (), "Ignoring duplicate get_origin request for state ID %lu while it is still active.", msg->state_id);
            return;
        }
        RCLCPP_WARN (this->get_logger (), "Cannot start get_origin state ID %lu while state ID %lu is still active.", msg->state_id, origin_state_id_);
        natto_msgs::msg::StateResult result;
        result.state_id    = msg->state_id;
        result.action_name = "get_origin";
        result.success     = false;
        state_result_publisher_->publish (result);
        return;
    }

    std::vector<origin_request_t> requests;
    std::set<std::string>         requested_joints;
    for (size_t i = 0; i < msg->arguments_names.size (); ++i) {
        const std::string argument_name  = detail::unquote (msg->arguments_names[i]);
        const std::string argument_value = detail::unquote (msg->arguments_values[i]);

        origin_request_t request;
        if (argument_name == "joint_name" || argument_name == "joint") {
            // Keep the old joint_name=<name> syntax. Waiting for SUCCEEDED is the safe legacy default.
            request.joint_name      = argument_value;
            request.required_status = natto_msgs::msg::OriginStatus::SUCCEEDED;
        } else {
            // New syntax: get_origin(joint_name=started|succeeded).
            request.joint_name = argument_name;
            if (argument_value == "started" || argument_value == "start" || argument_value == "need_start") {
                request.required_status = natto_msgs::msg::OriginStatus::STARTED;
            } else if (argument_value == "succeeded" || argument_value == "success" || argument_value == "need_success") {
                request.required_status = natto_msgs::msg::OriginStatus::SUCCEEDED;
            } else {
                RCLCPP_WARN (this->get_logger (), "Unsupported get_origin completion condition '%s' for joint '%s'. Use started or succeeded.", argument_value.c_str (), argument_name.c_str ());
                requests.clear ();
                break;
            }
        }

        if (request.joint_name.empty ()) {
            RCLCPP_WARN (this->get_logger (), "get_origin received an empty joint name.");
            requests.clear ();
            break;
        }
        if (!requested_joints.insert (request.joint_name).second) {
            RCLCPP_WARN (this->get_logger (), "get_origin received duplicate joint '%s'.", request.joint_name.c_str ());
            requests.clear ();
            break;
        }
        requests.push_back (request);
    }

    if (requests.empty ()) {
        RCLCPP_WARN (this->get_logger (), "get_origin requires at least one joint=started or joint=succeeded argument.");
        natto_msgs::msg::StateResult result;
        result.state_id    = msg->state_id;
        result.action_name = "get_origin";
        result.success     = false;
        state_result_publisher_->publish (result);
        return;
    }

    origin_state_id_      = msg->state_id;
    origin_requests_      = requests;
    origin_action_active_ = true;
    for (const auto &request : origin_requests_) {
        std_msgs::msg::String origin_get_msg;
        origin_get_msg.data = request.joint_name;
        origin_get_publisher_->publish (origin_get_msg);
    }
    RCLCPP_INFO (this->get_logger (), "Requested origin for %zu joints and waiting for the configured completion status.", origin_requests_.size ());
}

void default_action::publish_get_origin_result (bool success) {
    natto_msgs::msg::StateResult result;
    result.state_id    = origin_state_id_;
    result.action_name = "get_origin";
    result.success     = success;
    state_result_publisher_->publish (result);
    RCLCPP_INFO (this->get_logger (), "get_origin for state ID %lu %s.", origin_state_id_, success ? "succeeded" : "failed");
    origin_action_active_ = false;
    origin_requests_.clear ();
}

void default_action::origin_status_callback (const natto_msgs::msg::OriginStatus::SharedPtr msg) {
    if (!origin_action_active_) {
        return;
    }

    const auto request_it = std::find_if (origin_requests_.begin (), origin_requests_.end (), [&msg] (const auto &request) { return request.joint_name == msg->joint_name; });
    if (request_it == origin_requests_.end () || request_it->completed) {
        return;
    }

    if (msg->status == natto_msgs::msg::OriginStatus::FAILED) {
        RCLCPP_WARN (this->get_logger (), "Origin failed for joint '%s' (reason=%u).", msg->joint_name.c_str (), msg->reason);
        publish_get_origin_result (false);
        return;
    }

    if (msg->status >= request_it->required_status) {
        request_it->completed = true;
        RCLCPP_INFO (this->get_logger (), "Origin completion condition met for joint '%s' (status=%u).", msg->joint_name.c_str (), msg->status);
    }

    if (std::all_of (origin_requests_.begin (), origin_requests_.end (), [] (const auto &request) { return request.completed; })) {
        publish_get_origin_result (true);
    }
}

}  // namespace default_action
