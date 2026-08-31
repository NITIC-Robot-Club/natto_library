#include "natto_joy/joy_to_twist.hpp"

#include <cmath>

namespace joy_to_twist {

joy_to_twist::joy_to_twist (const rclcpp::NodeOptions &node_options) : Node ("joy_to_twist", node_options) {
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped> ("command_velocity", 1);
    joy_subscriber_  = this->create_subscription<sensor_msgs::msg::Joy> ("joy", 1, std::bind (&joy_to_twist::joy_callback, this, std::placeholders::_1));

    max_xy_speed_m_s_            = this->declare_parameter<double> ("max_xy_speed_m_s", 2.0);
    slow_max_xy_speed_m_s_       = this->declare_parameter<double> ("slow_max_xy_speed_m_s", 1.0);
    max_yaw_speed_rad_s_         = this->declare_parameter<double> ("max_yaw_speed_rad_s", 6.2830);
    slow_max_yaw_speed_rad_s_    = this->declare_parameter<double> ("slow_max_yaw_speed_rad_s", 3.1415);
    enable_speed_limit_release_  = this->declare_parameter<bool> ("enable_speed_limit_release", true);
    left_stick_release_button_   = static_cast<int> (this->declare_parameter<int> ("left_stick_release_button", 7));
    left_stick_x_axis_           = static_cast<int> (this->declare_parameter<int> ("left_stick_x_axis", 0));
    left_stick_y_axis_           = static_cast<int> (this->declare_parameter<int> ("left_stick_y_axis", 1));
    left_stick_motion_threshold_ = this->declare_parameter<double> ("left_stick_motion_threshold", 0.1);

    RCLCPP_INFO (this->get_logger (), "joy_to_twist node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "max_xy_speed_m_s: %.2f", max_xy_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "slow_max_xy_speed_m_s: %.2f", slow_max_xy_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "max_yaw_speed_rad_s: %.4f", max_yaw_speed_rad_s_);
    RCLCPP_INFO (this->get_logger (), "slow_max_yaw_speed_rad_s: %.4f", slow_max_yaw_speed_rad_s_);
    RCLCPP_INFO (this->get_logger (), "enable_speed_limit_release: %s", enable_speed_limit_release_ ? "true" : "false");
}

void joy_to_twist::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    const bool release_button_pressed       = left_stick_release_button_ >= 0 && static_cast<size_t> (left_stick_release_button_) < msg->buttons.size () && msg->buttons[static_cast<size_t> (left_stick_release_button_)] != 0;
    const bool release_button_positive_edge = release_button_pressed && !last_release_button_pressed_;
    const bool left_stick_is_moving         = left_stick_x_axis_ >= 0 && left_stick_y_axis_ >= 0 && static_cast<size_t> (left_stick_x_axis_) < msg->axes.size () && static_cast<size_t> (left_stick_y_axis_) < msg->axes.size () &&
                                      (std::abs (msg->axes[static_cast<size_t> (left_stick_x_axis_)]) > left_stick_motion_threshold_ || std::abs (msg->axes[static_cast<size_t> (left_stick_y_axis_)]) > left_stick_motion_threshold_);
    const bool left_stick_is_stopped = left_stick_x_axis_ >= 0 && left_stick_y_axis_ >= 0 && static_cast<size_t> (left_stick_x_axis_) < msg->axes.size () && static_cast<size_t> (left_stick_y_axis_) < msg->axes.size () &&
                                       std::abs (msg->axes[static_cast<size_t> (left_stick_x_axis_)]) <= left_stick_motion_threshold_ && std::abs (msg->axes[static_cast<size_t> (left_stick_y_axis_)]) <= left_stick_motion_threshold_;
    const bool yaw_is_moving     = msg->axes.size () > 2 && std::abs (msg->axes[2]) > left_stick_motion_threshold_;
    const bool yaw_is_stopped    = msg->axes.size () > 2 && std::abs (msg->axes[2]) <= left_stick_motion_threshold_;
    const bool motion_is_moving  = left_stick_is_moving || yaw_is_moving;
    const bool motion_is_stopped = left_stick_is_stopped && yaw_is_stopped;

<<<<<<< Updated upstream
    if (release_button_positive_edge) {
        speed_limit_release_active_ = true;
        motion_seen_since_release_  = motion_is_moving;
    } else if (speed_limit_release_active_) {
        motion_seen_since_release_ = motion_seen_since_release_ || motion_is_moving;
=======
    if (enable_speed_limit_release_) {
        if (release_button_positive_edge) {
            speed_limit_release_active_ = true;
            motion_seen_since_release_  = motion_is_moving;
        } else if (speed_limit_release_active_) {
            motion_seen_since_release_ = motion_seen_since_release_ || motion_is_moving;
>>>>>>> Stashed changes

            if (motion_seen_since_release_ && motion_is_stopped) {
                speed_limit_release_active_ = false;
            }
        }
    } else {
        speed_limit_release_active_ = false;
    }
    last_release_button_pressed_ = release_button_pressed;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp    = this->now ();
    twist_msg.header.frame_id = "command/base_link";
    const bool   use_slow_speed = enable_speed_limit_release_ && !speed_limit_release_active_;
    const double xy_speed       = use_slow_speed ? slow_max_xy_speed_m_s_ : max_xy_speed_m_s_;
    const double yaw_speed      = use_slow_speed ? slow_max_yaw_speed_rad_s_ : max_yaw_speed_rad_s_;
    twist_msg.twist.linear.x  = msg->axes[1] * xy_speed;
    twist_msg.twist.linear.y  = msg->axes[0] * xy_speed;
    twist_msg.twist.angular.z = msg->axes[2] * yaw_speed;
    twist_publisher_->publish (twist_msg);
}

}  // namespace joy_to_twist

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (joy_to_twist::joy_to_twist)
