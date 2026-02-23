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

#include "natto_speed_path_controller/speed_path_controller.hpp"

namespace speed_path_controller {

speed_path_controller::speed_path_controller (const rclcpp::NodeOptions &node_options) : Node ("speed_path_controller", node_options) {
    twist_publisher_         = this->create_publisher<geometry_msgs::msg::TwistStamped> ("command_velocity", 10);
    goal_reached_publisher_  = this->create_publisher<std_msgs::msg::Bool> ("goal_reached", 10);
    speed_path_subscriber_   = this->create_subscription<natto_msgs::msg::SpeedPath> ("speed_path", 10, std::bind (&speed_path_controller::speed_path_callback, this, std::placeholders::_1));
    current_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped> ("current_pose", 10, std::bind (&speed_path_controller::current_pose_callback, this, std::placeholders::_1));

    position_error_p_               = this->declare_parameter<double> ("position_error_p", 3.0);
    angle_error_p_                  = this->declare_parameter<double> ("angle_error_p", 3.0);
    position_error_allowance_m_     = this->declare_parameter<double> ("position_error_allowance_m", 0.2);
    angle_error_allowance_rad_      = this->declare_parameter<double> ("angle_error_allowance_rad", 0.2);
    goal_position_tolerance_        = this->declare_parameter<double> ("goal_position_tolerance", 0.02);
    goal_yaw_tolerance_deg_         = this->declare_parameter<double> ("goal_yaw_tolerance_deg", 5.0);
    goal_speed_tolerance_xy_m_s_    = this->declare_parameter<double> ("goal_speed_tolerance_xy_m_s", 0.1);
    goal_speed_tolerance_yaw_deg_s_ = this->declare_parameter<double> ("goal_speed_tolerance_yaw_deg_s", 10.0);
    double frequency                = this->declare_parameter<double> ("frequency", 100.0);
    timer_                          = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&speed_path_controller::timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "speed_path_controller has been initialized.");
    RCLCPP_INFO (this->get_logger (), "position_error_p: %f", position_error_p_);
    RCLCPP_INFO (this->get_logger (), "angle_error_p: %f", angle_error_p_);
    RCLCPP_INFO (this->get_logger (), "position_error_allowance_m: %f", position_error_allowance_m_);
    RCLCPP_INFO (this->get_logger (), "angle_error_allowance_rad: %f", angle_error_allowance_rad_);
    RCLCPP_INFO (this->get_logger (), "goal_position_tolerance: %f", goal_position_tolerance_);
    RCLCPP_INFO (this->get_logger (), "goal_yaw_tolerance_deg: %f", goal_yaw_tolerance_deg_);
    RCLCPP_INFO (this->get_logger (), "goal_speed_tolerance_xy_m_s: %f", goal_speed_tolerance_xy_m_s_);
    RCLCPP_INFO (this->get_logger (), "goal_speed_tolerance_yaw_deg_s: %f", goal_speed_tolerance_yaw_deg_s_);
    RCLCPP_INFO (this->get_logger (), "frequency: %f Hz", frequency);
}

void speed_path_controller::speed_path_callback (const natto_msgs::msg::SpeedPath::SharedPtr msg) {
    speed_path_ = *msg;
}

void speed_path_controller::current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void speed_path_controller::timer_callback () {
    if (speed_path_.path.empty ()) {
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = this->get_clock ()->now ();
        twist_publisher_->publish (twist);
        return;
    }

    size_t closest_index    = 0;
    double closest_distance = std::numeric_limits<double>::max ();
    for (size_t i = 0; i < speed_path_.path.size (); ++i) {
        double dx       = speed_path_.path[i].pose.position.x - current_pose_.pose.position.x;
        double dy       = speed_path_.path[i].pose.position.y - current_pose_.pose.position.y;
        double distance = std::hypot (dx, dy);
        if (distance < closest_distance) {
            closest_distance = distance;
            closest_index    = i;
        }
    }

    closest_index++;

    if (closest_index >= speed_path_.path.size ()) {
        closest_index = speed_path_.path.size () - 1;
    }

    double error_x   = speed_path_.path[closest_index].pose.position.x - current_pose_.pose.position.x;
    double error_y   = speed_path_.path[closest_index].pose.position.y - current_pose_.pose.position.y;
    double error_yaw = tf2::getYaw (speed_path_.path[closest_index].pose.orientation) - tf2::getYaw (current_pose_.pose.orientation);

    double goal_error_x = speed_path_.path.back ().pose.position.x - current_pose_.pose.position.x;
    double goal_error_y = speed_path_.path.back ().pose.position.y - current_pose_.pose.position.y;
    double goal_error_position = std::hypot (goal_error_x, goal_error_y);
    double goal_error_yaw = tf2::getYaw (speed_path_.path.back ().pose.orientation) - tf2::getYaw (current_pose_.pose.orientation);

    if (goal_error_position < goal_position_tolerance_ && std::abs (goal_error_yaw) < goal_yaw_tolerance_deg_ / 180.0 * M_PI) {
        std_msgs::msg::Bool goal_reached_msg;
        goal_reached_msg.data = true;
        goal_reached_publisher_->publish (goal_reached_msg);

        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = this->get_clock ()->now ();
        twist_publisher_->publish (twist);
        return;
    } else {
        std_msgs::msg::Bool goal_reached_msg;
        goal_reached_msg.data = false;
        goal_reached_publisher_->publish (goal_reached_msg);

    }

    if (std::abs (error_x) > position_error_allowance_m_) {
        RCLCPP_WARN (this->get_logger (), "Position error is within the allowance. No correction will be applied. error x: %f", error_x);
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = this->get_clock ()->now ();
        twist_publisher_->publish (twist);
        return;
    }

    if (std::abs (error_y) > position_error_allowance_m_) {
        RCLCPP_WARN (this->get_logger (), "Position error is within the allowance. No correction will be applied. error y: %f", error_y);
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = this->get_clock ()->now ();
        twist_publisher_->publish (twist);
        return;
    }

    if (std::abs (error_yaw) > angle_error_allowance_rad_) {
        RCLCPP_WARN (this->get_logger (), "Angle error is within the allowance. No correction will be applied. error yaw: %f", error_yaw);
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = this->get_clock ()->now ();
        twist_publisher_->publish (twist);
        return;
    }

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp    = this->get_clock ()->now ();
    twist.twist.linear.x  = speed_path_.twist[closest_index].twist.linear.x + position_error_p_ * error_x;
    twist.twist.linear.y  = speed_path_.twist[closest_index].twist.linear.y + position_error_p_ * error_y;
    twist.twist.angular.z = speed_path_.twist[closest_index].twist.angular.z + angle_error_p_ * error_yaw;
    twist_publisher_->publish (twist);
}

}  // namespace speed_path_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (speed_path_controller::speed_path_controller)