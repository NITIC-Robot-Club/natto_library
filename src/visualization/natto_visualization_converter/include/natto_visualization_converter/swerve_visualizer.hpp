#ifndef __SWERVE_VISUALIZER_HPP__
#define __SWERVE_VISUALIZER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "natto_msgs/msg/swerve.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace swerve_visualizer {
class swerve_visualizer : public rclcpp::Node {
   public:
    swerve_visualizer (const rclcpp::NodeOptions &node_options);

   private:
    visualization_msgs::msg::MarkerArray marker_array_;

    int    num_wheels_;
    double arrow_r, arrow_g, arrow_b, arrow_scale, arrow_min_size;

    std::vector<double> wheel_position_x;
    std::vector<double> wheel_position_y;

    void timer_callback ();
    void swerve_callback (const natto_msgs::msg::Swerve::SharedPtr msg);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Swerve>::SharedPtr           swerve_subscription_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
};
}  // namespace swerve_visualizer

#endif  // __SWERVE_VISUALIZER_HPP__