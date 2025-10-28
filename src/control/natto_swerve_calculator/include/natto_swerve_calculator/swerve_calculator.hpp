#ifndef __SWERVE_CALCULATOR_HPP__
#define __SWERVE_CALCULATOR_HPP__

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "natto_msgs/msg/swerve.hpp"

namespace swerve_calculator {
class swerve_calculator : public rclcpp::Node {
   public:
    swerve_calculator (const rclcpp::NodeOptions &node_options);

   private:
    bool   infinite_swerve_mode_;
    double wheel_radius_;
    int    num_wheels_;

    std::vector<double> wheel_position_x;
    std::vector<double> wheel_position_y;

    natto_msgs::msg::Swerve swerve_result_;

    void command_velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void swerve_result_callback (const natto_msgs::msg::Swerve::SharedPtr msg);

    rclcpp::Publisher<natto_msgs::msg::Swerve>::SharedPtr             swerve_command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_subscriber_;
    rclcpp::Subscription<natto_msgs::msg::Swerve>::SharedPtr          swerve_result_subscriber_;
};
}  // namespace swerve_calculator

#endif  // __SWERVE_CALCULATOR_HPP__