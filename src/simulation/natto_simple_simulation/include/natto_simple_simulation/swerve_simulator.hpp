#ifndef __SWERVE_SIMULATOR_HPP__
#define __SWERVE_SIMULATOR_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "natto_msgs/msg/swerve.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/transform_broadcaster.h>

namespace swerve_simulator {
class swerve_simulator : public rclcpp::Node {
   public:
    swerve_simulator (const rclcpp::NodeOptions &node_options);

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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr simulation_pose_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Swerve>::SharedPtr      swerve_command_subscriber_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
};
}  // namespace swerve_simulator

#endif  // __SWERVE_SIMULATOR_HPP__