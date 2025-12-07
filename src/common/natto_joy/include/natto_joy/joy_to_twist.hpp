#ifndef __JOY_TO_TWIST_HPP__
#define __JOY_TO_TWIST_HPP__

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace joy_to_twist {
class joy_to_twist : public rclcpp::Node {
   public:
    joy_to_twist (const rclcpp::NodeOptions &node_options);

   private:
    double max_xy_speed_m_s_;
    double max_theta_speed_rad_s_;

    void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         joy_subscriber_;
    rclcpp::TimerBase::SharedPtr                                   timer_;
};
}  // namespace joy_to_twist

#endif  // __JOY_TO_TWIST_HPP__
