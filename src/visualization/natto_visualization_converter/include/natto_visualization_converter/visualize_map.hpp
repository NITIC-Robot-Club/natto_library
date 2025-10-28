#ifndef __VISUALIZE_MAP_HPP__
#define __VISUALIZE_MAP_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace visualize_map {
class visualize_map : public rclcpp::Node {
   public:
    visualize_map (const rclcpp::NodeOptions &node_options);

   private:
    visualization_msgs::msg::MarkerArray marker_array_;

    void timer_callback ();
    void map_callback (const natto_msgs::msg::Map::SharedPtr msg);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Map>::SharedPtr              map_subscription_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
};
}  // namespace visualize_map

#endif  // __MAP_LOADER_HPP__