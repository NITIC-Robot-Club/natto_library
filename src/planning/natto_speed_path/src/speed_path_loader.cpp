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

#include "natto_speed_path/speed_path_loader.hpp"

namespace speed_path_loader {

speed_path_loader::speed_path_loader (const rclcpp::NodeOptions &node_options) : Node ("speed_path_loader", node_options) {
    speed_path_publisher_    = this->create_publisher<natto_msgs::msg::SpeedPath> ("speed_path", 10);
    state_result_publisher_  = this->create_publisher<natto_msgs::msg::StateResult> ("state_result", 10);
    state_action_subscriber_ = this->create_subscription<natto_msgs::msg::StateAction> ("state_action", 10, std::bind (&speed_path_loader::state_action_callback, this, std::placeholders::_1));
    goal_reached_subscriber_ = this->create_subscription<std_msgs::msg::Bool> ("goal_reached", 10, std::bind (&speed_path_loader::goal_reached_callback, this, std::placeholders::_1));

    file_directory_ = this->declare_parameter<std::string> ("file_directory", "");
    if (file_directory_.empty ()) {
        RCLCPP_ERROR (this->get_logger (), "File directory parameter is empty. Please set the 'file_directory' parameter.");
        throw std::runtime_error ("File directory parameter is empty.");
    }

    double frequency = this->declare_parameter<double> ("frequency", 1.0);

    speed_path_.header.frame_id = "map";
    speed_path_.path.clear ();
    speed_path_.twist.clear ();

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&speed_path_loader::timer_callback, this));
}

void speed_path_loader::load_speed_path (std::string file_path) {
    std::ifstream file (file_path);
    if (!file.is_open ()) {
        RCLCPP_ERROR (this->get_logger (), "Failed to open speed path file: %s", file_path.c_str ());
        return;
    }

    std::string line;
    std::getline (file, line);

    speed_path_.header.frame_id = "map";
    speed_path_.path.clear ();
    speed_path_.twist.clear ();

    while (std::getline (file, line)) {
        std::stringstream                ss (line);
        geometry_msgs::msg::PoseStamped  pose;
        geometry_msgs::msg::TwistStamped twist;
        std::string                      val;

        std::getline (ss, val, ',');
        double t = std::stod (val);

        std::getline (ss, val, ',');
        pose.pose.position.x = std::stod (val);
        std::getline (ss, val, ',');
        pose.pose.position.y = std::stod (val);
        std::getline (ss, val, ',');
        double yaw              = std::stod (val);
        pose.pose.orientation.z = std::sin (yaw / 2.0);
        pose.pose.orientation.w = std::cos (yaw / 2.0);

        std::getline (ss, val, ',');
        twist.twist.linear.x = std::stod (val);
        std::getline (ss, val, ',');
        twist.twist.linear.y = std::stod (val);
        std::getline (ss, val, ',');
        twist.twist.angular.z = std::stod (val);

        speed_path_.path.push_back (pose);
        speed_path_.twist.push_back (twist);
    }
    RCLCPP_INFO (this->get_logger (), "Loaded speed path with %zu points from file: %s", speed_path_.path.size (), file_path.c_str ());
}

void speed_path_loader::state_action_callback (const natto_msgs::msg::StateAction::SharedPtr msg) {
    if (msg->action_name == "load_speed_path") {
        RCLCPP_INFO (this->get_logger (), "Received load_speed_path action. Loading speed path from file: %s", msg->arguments_values[0].c_str ());
        load_speed_path (file_directory_ + "/" + msg->arguments_values[0]);
        set_speed_path_state_id_ = msg->state_id;
    } else {
        RCLCPP_WARN (this->get_logger (), "Received unknown action: %s", msg->action_name.c_str ());
    }
}

void speed_path_loader::goal_reached_callback (const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        RCLCPP_INFO (this->get_logger (), "Goal reached. Clearing speed path.");
        speed_path_.header.stamp = this->now ();
        speed_path_.path.clear ();
        speed_path_.twist.clear ();
        speed_path_publisher_->publish (speed_path_);

        natto_msgs::msg::StateResult state_result_msg;
        state_result_msg.state_id = set_speed_path_state_id_;
        state_result_msg.success  = true;
        state_result_publisher_->publish (state_result_msg);
    }
}

void speed_path_loader::timer_callback () {
    speed_path_publisher_->publish (speed_path_);
}

}  // namespace speed_path_loader

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (speed_path_loader::speed_path_loader)