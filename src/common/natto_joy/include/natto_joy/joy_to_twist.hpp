#ifndef __JOY_TO_TWIST_HPP__
#define __JOY_TO_TWIST_HPP__

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <chrono>
#include <cmath>
#include <mutex>

namespace joy_to_twist {
class joy_to_twist : public rclcpp::Node {
   public:
    joy_to_twist (const rclcpp::NodeOptions &node_options);

   private:
    double max_xy_speed_m_s_;
    double slow_max_xy_speed_m_s_;
    double max_yaw_speed_rad_s_;
    double slow_max_yaw_speed_rad_s_;
    double joy_timeout_sec_ = 0.0;
    bool   enable_speed_limit_release_ = false;
    int    left_stick_release_button_;
    int    left_stick_x_axis_;
    int    left_stick_y_axis_;
    double left_stick_motion_threshold_;
    bool   speed_limit_release_active_  = false;
    bool   motion_seen_since_release_   = false;
    bool   last_release_button_pressed_ = false;
    bool   timeout_zero_published_      = false;

    std::chrono::steady_clock::time_point last_joy_received_time_;
    std::mutex                            timeout_mutex_;

    void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg);
    void timeout_callback ();

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         joy_subscriber_;
    rclcpp::TimerBase::SharedPtr                                   timeout_timer_;
};
}  // namespace joy_to_twist

#endif  // __JOY_TO_TWIST_HPP__
