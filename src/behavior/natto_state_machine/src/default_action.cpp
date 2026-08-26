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

#include <functional>

namespace default_action {

default_action::default_action (const rclcpp::NodeOptions &node_options) : Node ("default_action", rclcpp::NodeOptions (node_options).allow_undeclared_parameters (true).automatically_declare_parameters_from_overrides (true)) {
    state_result_publisher_       = this->create_publisher<natto_msgs::msg::StateResult> ("state_result", 10);
    joint_control_type_publisher_ = this->create_publisher<natto_msgs::msg::JointControlType> ("/set_joint_control_type", 10);
    goal_publisher_               = this->create_publisher<geometry_msgs::msg::PoseStamped> ("goal_pose", 10);
    joint_state_publisher_        = this->create_publisher<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::SensorDataQoS ());
    power_publisher_              = this->create_publisher<std_msgs::msg::Bool> ("power", 10);
    origin_get_publisher_         = this->create_publisher<std_msgs::msg::String> ("get_origin_joint_name", 10);
    origin_cancel_publisher_      = this->create_publisher<std_msgs::msg::String> ("cancel_origin_joint_name", 10);
    origin_status_subscriber_     = this->create_subscription<natto_msgs::msg::OriginStatus> ("origin_status", 10, std::bind (&default_action::origin_status_callback, this, std::placeholders::_1));
    state_action_subscriber_      = this->create_subscription<natto_msgs::msg::StateAction> ("state_action", 10, std::bind (&default_action::state_action_callback, this, std::placeholders::_1));
    goal_result_subscriber_       = this->create_subscription<std_msgs::msg::Bool> ("goal_reached", 10, std::bind (&default_action::goal_result_callback, this, std::placeholders::_1));
    current_pose_subscriber_      = this->create_subscription<geometry_msgs::msg::PoseStamped> ("current_pose", 10, [this] (const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_pose_ = msg->pose; });
    joint_state_subscriber_       = this->create_subscription<sensor_msgs::msg::JointState> ("joint_states", rclcpp::SensorDataQoS (), std::bind (&default_action::joint_state_callback, this, std::placeholders::_1));
    allow_auto_drive_subscriber_  = this->create_subscription<std_msgs::msg::Bool> ("allow_auto_drive", 10, [this] (const std_msgs::msg::Bool::SharedPtr msg) { allow_auto_drive_ = msg->data; });
    cancel_sequence_subscriber_   = this->create_subscription<std_msgs::msg::Empty> ("cancel_sequence", 10, std::bind (&default_action::cancel_sequence_callback, this, std::placeholders::_1));

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

    set_pose_goal_sent_   = false;
    joint_state_sent_     = false;
    wait_started_         = false;
    origin_action_active_ = false;
}

void default_action::state_action_callback (const natto_msgs::msg::StateAction::SharedPtr msg) {
    if (msg->action_name == "set_pose") {
        handle_set_pose (msg);
    } else if (msg->action_name == "wait") {
        handle_wait (msg);
    } else if (msg->action_name == "set_joint_position") {
        handle_set_joint_position (msg);
    } else if (msg->action_name == "set_joint_velocity") {
        handle_set_joint_velocity (msg);
    } else if (msg->action_name == "set_control_type") {
        handle_set_control_type (msg);
    } else if (msg->action_name == "get_origin") {
        handle_get_origin (msg);
    } else if (msg->action_name == "set_power" || msg->action_name == "poweroff") {
        handle_power (msg);
    }
}

void default_action::cancel_sequence_callback (const std_msgs::msg::Empty::SharedPtr) {
    if (origin_action_active_) {
        for (const auto &request : origin_requests_) {
            std_msgs::msg::String cancel_msg;
            cancel_msg.data = request.joint_name;
            origin_cancel_publisher_->publish (cancel_msg);
        }
        origin_action_active_ = false;
        origin_requests_.clear ();
    }

    set_pose_goal_sent_ = false;
    wait_started_       = false;
    joint_state_sent_   = false;
    command_joint_state_.name.clear ();
    command_joint_state_.position.clear ();
    command_joint_state_.velocity.clear ();
    command_joint_state_.effort.clear ();
    RCLCPP_WARN (this->get_logger (), "Cancelled active default actions.");
}

void default_action::handle_wait (const natto_msgs::msg::StateAction::SharedPtr msg) {
    wait_state_id_ = msg->state_id;
    if (!wait_started_) {
        wait_start_time_ = this->now ();
    }
    wait_started_ = true;
    for (size_t i = 0; i < msg->arguments_names.size () && i < msg->arguments_values.size (); i++) {
        if (msg->arguments_names[i] == "duration_sec") {
            wait_duration_sec_ = std::stod (msg->arguments_values[i]);
        }
    }
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
