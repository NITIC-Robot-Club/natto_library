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
#include <functional>

namespace {

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

}  // namespace

namespace default_action {

default_action::default_action (const rclcpp::NodeOptions &node_options) : Node ("default_action", rclcpp::NodeOptions (node_options).allow_undeclared_parameters (true).automatically_declare_parameters_from_overrides (true)) {
    state_result_publisher_       = this->create_publisher<natto_msgs::msg::StateResult> ("state_result", 10);
    joint_control_type_publisher_ = this->create_publisher<natto_msgs::msg::JointControlType> ("/set_joint_control_type", 10);
    goal_publisher_               = this->create_publisher<geometry_msgs::msg::PoseStamped> ("goal_pose", 10);
    joint_state_publisher_        = this->create_publisher<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::SensorDataQoS ());
    origin_get_publisher_         = this->create_publisher<std_msgs::msg::String> ("get_origin_joint_name", 10);
    state_action_subscriber_      = this->create_subscription<natto_msgs::msg::StateAction> ("state_action", 10, std::bind (&default_action::state_action_callback, this, std::placeholders::_1));
    goal_result_subscriber_       = this->create_subscription<std_msgs::msg::Bool> ("goal_reached", 10, std::bind (&default_action::goal_result_callback, this, std::placeholders::_1));
    current_pose_subscriber_      = this->create_subscription<geometry_msgs::msg::PoseStamped> ("current_pose", 10, [this] (const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_pose_ = msg->pose; });
    joint_state_subscriber_       = this->create_subscription<sensor_msgs::msg::JointState> ("joint_states", rclcpp::SensorDataQoS (), std::bind (&default_action::joint_state_callback, this, std::placeholders::_1));
    allow_auto_drive_subscriber_  = this->create_subscription<std_msgs::msg::Bool> ("allow_auto_drive", 10, [this] (const std_msgs::msg::Bool::SharedPtr msg) { allow_auto_drive_ = msg->data; });

    xy_tolerance_m_    = this->declare_parameter<double> ("xy_tolerance_m", 0.2);
    yaw_tolerance_deg_ = this->declare_parameter<double> ("yaw_tolerance_deg", 10.0);
    if (this->has_parameter ("initial_allow_auto_drive")) {
        allow_auto_drive_ = this->get_parameter ("initial_allow_auto_drive").as_bool ();
    } else {
        allow_auto_drive_ = this->declare_parameter<bool> ("initial_allow_auto_drive", false);
    }

    frequency_ = this->declare_parameter<double> ("frequency", 10.0);
    timer_     = this->create_wall_timer (std::chrono::duration (std::chrono::duration<double> (1.0 / frequency_)), std::bind (&default_action::timer_callback, this));

    std::map<std::string, rclcpp::Parameter> params;
    this->get_parameters ("tolerances", params);
    joint_tolerances_.clear ();
    for (const auto &kv : params) {
        joint_tolerances_[kv.first] = kv.second.as_double ();
    }

    if (this->has_parameter ("reverse_y")) {
        reverse_y_ = this->get_parameter ("reverse_y").as_bool ();
    } else {
        reverse_y_ = this->declare_parameter<bool> ("reverse_y", false);
    }

    if (this->has_parameter ("reverse_y_offset")) {
        reverse_y_offset_ = this->get_parameter ("reverse_y_offset").as_double ();
    } else {
        reverse_y_offset_ = this->declare_parameter<double> ("reverse_y_offset", 0.0);
    }

    RCLCPP_INFO (this->get_logger (), "default_action node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "xy_tolerance_m: %.3f", xy_tolerance_m_);
    RCLCPP_INFO (this->get_logger (), "yaw_tolerance_deg: %.3f", yaw_tolerance_deg_);
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency_);
    RCLCPP_INFO (this->get_logger (), "initial_allow_auto_drive: %s", allow_auto_drive_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "tolerances:");
    for (const auto &kv : joint_tolerances_) {
        RCLCPP_INFO (this->get_logger (), "  %s: %.4f", kv.first.c_str (), kv.second);
    }
    RCLCPP_INFO (this->get_logger (), "reverse_y: %s", reverse_y_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "reverse_y_offset: %.3f", reverse_y_offset_);

    set_pose_goal_sent_ = false;
    joint_state_sent_   = false;
    wait_started_       = false;
}

void default_action::state_action_callback (const natto_msgs::msg::StateAction::SharedPtr msg) {
    if (msg->action_name == "set_pose") {
        for (size_t i = 0; i < msg->arguments_names.size (); i++) {
            if (msg->arguments_names[i] == "x") {
                goal_pose_.position.x = std::stod (msg->arguments_values[i]);
            } else if (msg->arguments_names[i] == "y") {
                goal_pose_.position.y = std::stod (msg->arguments_values[i]);
                if (reverse_y_) {
                    goal_pose_.position.y = -goal_pose_.position.y + reverse_y_offset_;
                }
            } else if (msg->arguments_names[i] == "yaw") {
                double yaw = std::stod (msg->arguments_values[i]) * M_PI / 180.0;
                if (reverse_y_) {
                    yaw *= -1;
                }
                tf2::Quaternion q;
                q.setRPY (0.0, 0.0, yaw);
                goal_pose_.orientation = tf2::toMsg (q);
            }
        }
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp    = this->now ();
        goal.header.frame_id = "map";
        goal.pose            = goal_pose_;
        goal_publisher_->publish (goal);
        set_pose_state_id_  = msg->state_id;
        set_pose_goal_sent_ = true;
    } else if (msg->action_name == "wait") {
        wait_state_id_ = msg->state_id;
        if (!wait_started_) {
            wait_start_time_ = this->now ();
        }
        wait_started_ = true;
        for (size_t i = 0; i < msg->arguments_names.size (); i++) {
            if (msg->arguments_names[i] == "duration_sec") {
                wait_duration_sec_ = std::stod (msg->arguments_values[i]);
            }
        }
    } else if (msg->action_name == "set_joint_position") {
        joint_state_id_   = msg->state_id;
        joint_state_sent_ = true;
        command_joint_state_.name.clear ();
        command_joint_state_.position.clear ();
        command_joint_state_.velocity.clear ();
        for (size_t i = 0; i < msg->arguments_names.size (); i++) {
            size_t j;
            for (j = 0; j < command_joint_state_.name.size (); j++) {
                if (msg->arguments_names[i] == command_joint_state_.name[j]) {
                    command_joint_state_.position[j] = std::stod (msg->arguments_values[i]);
                }
            }
            if (j == command_joint_state_.name.size ()) {
                command_joint_state_.name.push_back (msg->arguments_names[i]);
                command_joint_state_.position.push_back (std::stod (msg->arguments_values[i]));
                command_joint_state_.velocity.push_back (0.0);
            }
        }
    } else if (msg->action_name == "set_joint_velocity") {
        joint_state_id_   = msg->state_id;
        joint_state_sent_ = false;
        command_joint_state_.name.clear ();
        command_joint_state_.position.clear ();
        command_joint_state_.velocity.clear ();
        for (size_t i = 0; i < msg->arguments_names.size (); i++) {
            size_t j;
            for (j = 0; j < command_joint_state_.name.size (); j++) {
                if (msg->arguments_names[i] == command_joint_state_.name[j]) {
                    command_joint_state_.velocity[j] = std::stod (msg->arguments_values[i]);
                }
            }
            if (j == command_joint_state_.name.size ()) {
                command_joint_state_.name.push_back (msg->arguments_names[i]);
                command_joint_state_.position.push_back (0.0);
                command_joint_state_.velocity.push_back (std::stod (msg->arguments_values[i]));
            }
        }

        natto_msgs::msg::StateResult result;
        result.state_id    = joint_state_id_;
        result.success     = true;
        result.action_name = "set_joint_velocity";
        state_result_publisher_->publish (result);
    } else if (msg->action_name == "set_control_type") {
        handle_set_control_type (msg);
    } else if (msg->action_name == "get_origin") {
        std::string joint_name;
        for (size_t i = 0; i < msg->arguments_names.size () && i < msg->arguments_values.size (); i++) {
            if (msg->arguments_names[i] == "joint_name" || msg->arguments_names[i] == "joint") {
                joint_name = msg->arguments_values[i];
                if (joint_name.size () >= 2 && ((joint_name.front () == '\'' && joint_name.back () == '\'') || (joint_name.front () == '"' && joint_name.back () == '"'))) {
                    joint_name = joint_name.substr (1, joint_name.size () - 2);
                }
                break;
            }
        }

        natto_msgs::msg::StateResult result;
        result.state_id    = msg->state_id;
        result.action_name = "get_origin";
        if (joint_name.empty ()) {
            RCLCPP_WARN (this->get_logger (), "get_origin requires a joint_name argument.");
            result.success = false;
        } else {
            std_msgs::msg::String origin_get_msg;
            origin_get_msg.data = joint_name;
            origin_get_publisher_->publish (origin_get_msg);
            result.success = true;
            RCLCPP_INFO (this->get_logger (), "Requested origin for joint '%s'.", joint_name.c_str ());
        }
        state_result_publisher_->publish (result);
    }
}

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
        const std::string value      = remove_quotes (msg->arguments_values[i]);

        if (joint_name.empty () || std::find_if (changes.begin (), changes.end (), [&joint_name] (const auto &change) { return change.joint_name == joint_name; }) != changes.end ()) {
            RCLCPP_WARN (this->get_logger (), "Invalid or duplicated joint name in set_control_type: '%s'", joint_name.c_str ());
            publish_set_control_type_result (msg->state_id, false);
            return;
        }

        const auto control_type = control_type_from_string (value);
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

void default_action::goal_result_callback (const std_msgs::msg::Bool::SharedPtr msg) {
    if (!set_pose_goal_sent_) {
        return;
    }
    bool position_reached = false;
    bool yaw_reached      = false;

    double dx       = goal_pose_.position.x - current_pose_.position.x;
    double dy       = goal_pose_.position.y - current_pose_.position.y;
    double distance = sqrt (dx * dx + dy * dy);
    if (distance <= xy_tolerance_m_) {
        position_reached = true;
    }

    double goal_yaw    = tf2::getYaw (goal_pose_.orientation);
    double current_yaw = tf2::getYaw (current_pose_.orientation);
    double yaw_diff    = fabs (goal_yaw - current_yaw);
    if (yaw_diff > M_PI) {
        yaw_diff = 2.0 * M_PI - yaw_diff;
    }
    if (yaw_diff <= yaw_tolerance_deg_ * M_PI / 180.0) {
        yaw_reached = true;
    }

    natto_msgs::msg::StateResult result;
    result.state_id    = set_pose_state_id_;
    result.success     = msg->data && position_reached && yaw_reached;
    result.action_name = "set_pose";
    state_result_publisher_->publish (result);

    if (result.success) {
        set_pose_goal_sent_ = false;
    }
}

void default_action::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!joint_state_sent_) {
        return;
    }
    bool reached = true;
    for (size_t i = 0; i < msg->name.size (); i++) {
        for (size_t j = 0; j < command_joint_state_.name.size (); j++) {
            if (msg->name[i] == command_joint_state_.name[j]) {
                auto   it_tol = joint_tolerances_.find (msg->name[i]);
                double tol    = 0.01;
                if (it_tol != joint_tolerances_.end ()) {
                    tol = it_tol->second;
                }
                if (fabs (msg->position[i] - command_joint_state_.position[j]) > tol) {
                    reached = false;
                    RCLCPP_INFO (this->get_logger (), "Joint %s not reached: current=%.4f, command=%.4f, tol=%.4f", msg->name[i].c_str (), msg->position[i], command_joint_state_.position[j], tol);
                }
            }
        }
    }
    natto_msgs::msg::StateResult result;
    result.state_id    = joint_state_id_;
    result.success     = reached;
    result.action_name = "set_joint_position";
    state_result_publisher_->publish (result);
}

void default_action::timer_callback () {
    if (wait_started_) {
        rclcpp::Time now = this->now ();
        if ((now - wait_start_time_).seconds () >= wait_duration_sec_) {
            natto_msgs::msg::StateResult result;
            result.state_id    = wait_state_id_;
            result.success     = true;
            result.action_name = "wait";
            state_result_publisher_->publish (result);
            wait_started_ = false;
        }
    }
    if (allow_auto_drive_) {
        command_joint_state_.header.stamp = this->now ();
        joint_state_publisher_->publish (command_joint_state_);
    }
}

}  // namespace default_action

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (default_action::default_action)
