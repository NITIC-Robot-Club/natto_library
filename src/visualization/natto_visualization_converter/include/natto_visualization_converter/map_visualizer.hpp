#ifndef __MAP_VISUALIZER_HPP__
#define __MAP_VISUALIZER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace map_visualizer {
class map_visualizer : public rclcpp::Node {
   public:
    map_visualizer (const rclcpp::NodeOptions &node_options);

   private:
    visualization_msgs::msg::MarkerArray marker_array_;

    void timer_callback ();
    void map_callback (const natto_msgs::msg::Map::SharedPtr msg);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Map>::SharedPtr              map_subscription_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
};
}  // namespace map_visualizer

#endif  // __MAP_VISUALIZER_HPP__