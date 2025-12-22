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

namespace default_action {

default_action::default_action (const rclcpp::NodeOptions &node_options) : Node ("default_action", node_options) {
    state_result_publisher_  = this->create_publisher<natto_msgs::msg::StateResult> ("state_result", 10);
    goal_publisher_          = this->create_publisher<geometry_msgs::msg::PoseStamped> ("goal_pose", 10);
    state_action_subscriber_ = this->create_subscription<natto_msgs::msg::StateAction> ("state_action", 10, std::bind (&default_action::state_action_callback, this, std::placeholders::_1));
    goal_result_subscriber_  = this->create_subscription<std_msgs::msg::Bool> ("goal_reached", 10, std::bind (&default_action::goal_result_callback, this, std::placeholders::_1));
    current_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped> ("current_pose", 10, [this] (const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_pose_ = msg->pose; });

    xy_tolerance_m_    = this->declare_parameter<double> ("xy_tolerance_m", 0.2);
    yaw_tolerance_deg_ = this->declare_parameter<double> ("yaw_tolerance_deg", 10.0);
    frequency_         = this->declare_parameter<double> ("frequency", 10.0);
    timer_             = this->create_wall_timer (std::chrono::duration (std::chrono::duration<double> (1.0 / frequency_)), std::bind (&default_action::timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "default_action node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "xy_tolerance_m: %.3f", xy_tolerance_m_);
    RCLCPP_INFO (this->get_logger (), "yaw_tolerance_deg: %.3f", yaw_tolerance_deg_);
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency_);

    set_pose_goal_sent_ = false;
    wait_started_       = false;
}

void default_action::state_action_callback (const natto_msgs::msg::StateAction::SharedPtr msg) {
    if (msg->action_name == "set_pose") {
        for (size_t i = 0; i < msg->arguments_names.size (); i++) {
            if (msg->arguments_names[i] == "x") {
                goal_pose_.position.x = std::stod (msg->arguments_values[i]);
            } else if (msg->arguments_names[i] == "y") {
                goal_pose_.position.y = std::stod (msg->arguments_values[i]);
            } else if (msg->arguments_names[i] == "yaw") {
                tf2::Quaternion q;
                q.setRPY (0.0, 0.0, std::stod (msg->arguments_values[i]) * M_PI / 180.0);
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
    }
}

void default_action::goal_result_callback (const std_msgs::msg::Bool::SharedPtr msg) {
    if (set_pose_goal_sent_) {
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
}

}  // namespace default_action

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (default_action::default_action)