#include "natto_joy/joy_to_twist.hpp"

namespace joy_to_twist {

joy_to_twist::joy_to_twist (const rclcpp::NodeOptions& node_options) : Node ("joy_to_twist", node_options) {
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped> ("command_velocity", 1);
    joy_subscriber_  = this->create_subscription<sensor_msgs::msg::Joy> ("joy", 1, std::bind (&joy_to_twist::joy_callback, this, std::placeholders::_1));

    max_xy_speed_m_s_      = this->declare_parameter<double> ("max_xy_speed_m_s", 2.0);
    max_theta_speed_rad_s_ = this->declare_parameter<double> ("max_theta_speed_rad_s", 3.1415);
}

void joy_to_twist::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header          = msg->header;
    twist_msg.twist.linear.x  = msg->axes[1] * max_xy_speed_m_s_;
    twist_msg.twist.linear.y  = msg->axes[0] * max_xy_speed_m_s_;
    twist_msg.twist.angular.z = msg->axes[2] * max_theta_speed_rad_s_;
    twist_publisher_->publish (twist_msg);
}

}  // namespace joy_to_twist

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (joy_to_twist::joy_to_twist)