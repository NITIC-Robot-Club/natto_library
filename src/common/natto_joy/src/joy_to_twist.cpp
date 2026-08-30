#include "natto_joy/joy_to_twist.hpp"

#include <cmath>

namespace joy_to_twist {

joy_to_twist::joy_to_twist (const rclcpp::NodeOptions &node_options) : Node ("joy_to_twist", node_options) {
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped> ("command_velocity", 1);
    joy_subscriber_  = this->create_subscription<sensor_msgs::msg::Joy> ("joy", 1, std::bind (&joy_to_twist::joy_callback, this, std::placeholders::_1));

    max_xy_speed_m_s_            = this->declare_parameter<double> ("max_xy_speed_m_s", 2.0);
    slow_max_xy_speed_m_s_       = this->declare_parameter<double> ("slow_max_xy_speed_m_s", max_xy_speed_m_s_);
    max_yaw_speed_rad_s_         = this->declare_parameter<double> ("max_yaw_speed_rad_s", 3.1415);
    slow_max_yaw_speed_rad_s_    = this->declare_parameter<double> ("slow_max_yaw_speed_rad_s", max_yaw_speed_rad_s_);
    left_stick_release_button_   = static_cast<int> (this->declare_parameter<int> ("left_stick_release_button", 7));
    left_stick_x_axis_           = static_cast<int> (this->declare_parameter<int> ("left_stick_x_axis", 0));
    left_stick_y_axis_           = static_cast<int> (this->declare_parameter<int> ("left_stick_y_axis", 1));
    left_stick_motion_threshold_ = this->declare_parameter<double> ("left_stick_motion_threshold", 0.1);

    RCLCPP_INFO (this->get_logger (), "joy_to_twist node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "max_xy_speed_m_s: %.2f", max_xy_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "slow_max_xy_speed_m_s: %.2f", slow_max_xy_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "max_yaw_speed_rad_s: %.4f", max_yaw_speed_rad_s_);
    RCLCPP_INFO (this->get_logger (), "slow_max_yaw_speed_rad_s: %.4f", slow_max_yaw_speed_rad_s_);
}

void joy_to_twist::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    const bool release_button_pressed       = left_stick_release_button_ >= 0 && static_cast<size_t> (left_stick_release_button_) < msg->buttons.size () && msg->buttons[static_cast<size_t> (left_stick_release_button_)] != 0;
    const bool release_button_positive_edge = release_button_pressed && !last_release_button_pressed_;
    const bool left_stick_is_moving         = left_stick_x_axis_ >= 0 && left_stick_y_axis_ >= 0 && static_cast<size_t> (left_stick_x_axis_) < msg->axes.size () && static_cast<size_t> (left_stick_y_axis_) < msg->axes.size () &&
                                      (std::abs (msg->axes[static_cast<size_t> (left_stick_x_axis_)]) > left_stick_motion_threshold_ || std::abs (msg->axes[static_cast<size_t> (left_stick_y_axis_)]) > left_stick_motion_threshold_);
    const bool left_stick_is_stopped = left_stick_x_axis_ >= 0 && left_stick_y_axis_ >= 0 && static_cast<size_t> (left_stick_x_axis_) < msg->axes.size () && static_cast<size_t> (left_stick_y_axis_) < msg->axes.size () &&
                                       std::abs (msg->axes[static_cast<size_t> (left_stick_x_axis_)]) <= left_stick_motion_threshold_ && std::abs (msg->axes[static_cast<size_t> (left_stick_y_axis_)]) <= left_stick_motion_threshold_;

    if (release_button_positive_edge) {
        speed_limit_release_active_           = true;
        left_stick_motion_seen_since_release_ = left_stick_is_moving;
    } else if (speed_limit_release_active_) {
        left_stick_motion_seen_since_release_ = left_stick_motion_seen_since_release_ || left_stick_is_moving;

        if (left_stick_motion_seen_since_release_ && left_stick_is_stopped) {
            speed_limit_release_active_ = false;
        }
    }
    last_release_button_pressed_ = release_button_pressed;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp    = this->now ();
    twist_msg.header.frame_id = "command/base_link";
    const double xy_speed     = speed_limit_release_active_ ? max_xy_speed_m_s_ : slow_max_xy_speed_m_s_;
    const double yaw_speed    = speed_limit_release_active_ ? max_yaw_speed_rad_s_ : slow_max_yaw_speed_rad_s_;
    twist_msg.twist.linear.x  = msg->axes[1] * xy_speed;
    twist_msg.twist.linear.y  = msg->axes[0] * xy_speed;
    twist_msg.twist.angular.z = msg->axes[2] * yaw_speed;
    twist_publisher_->publish (twist_msg);
}

}  // namespace joy_to_twist

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (joy_to_twist::joy_to_twist)
