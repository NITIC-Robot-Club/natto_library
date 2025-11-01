#ifndef __LASER_FILTER_HPP__
#define __LASER_FILTER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

namespace laser_filter {
class laser_filter : public rclcpp::Node {
   public:
    laser_filter (const rclcpp::NodeOptions &node_options);

   private:
    double threshold_;

    void scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
};
}  // namespace laser_filter

#endif  // __LASER_FILTER_HPP__
