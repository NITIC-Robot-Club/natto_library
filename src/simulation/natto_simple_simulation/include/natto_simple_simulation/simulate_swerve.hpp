#ifndef __SIMULATE_SWERVE_HPP__
#define __SIMULATE_SWERVE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "natto_msgs/msg/swerve.hpp"

namespace simulate_swerve {
class simulate_swerve : public rclcpp::Node {
   public:
    simulate_swerve (const rclcpp::NodeOptions &node_options);

   private:
    bool   infinite_swerve_mode_;
    double wheel_radius_;
    int    num_wheels_, period_ms;
    double angle_gain_p_, angle_gain_d_;
    double speed_gain_p_, speed_gain_d_;

    std::vector<double> wheel_position_x;
    std::vector<double> wheel_position_y;

    std::vector<natto_msgs::msg::Swerve> received_commands;
    natto_msgs::msg::Swerve              command;
    natto_msgs::msg::Swerve              result;
    geometry_msgs::msg::PoseStamped      current_pose;

    void swerve_command_callback (const natto_msgs::msg::Swerve::SharedPtr msg);
    void timer_callback ();

    rclcpp::Publisher<natto_msgs::msg::Swerve>::SharedPtr         swerve_result_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Swerve>::SharedPtr      swerve_command_subscriber_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
};
}  // namespace simulate_swerve

#endif  // __SIMULATE_SWERVE_HPP__