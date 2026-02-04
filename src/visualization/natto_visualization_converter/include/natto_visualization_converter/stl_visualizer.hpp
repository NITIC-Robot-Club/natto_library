#ifndef __STL_VISUALIZER_HPP__
#define __STL_VISUALIZER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker.hpp"

namespace stl_visualizer {
class stl_visualizer : public rclcpp::Node {
   public:
    explicit stl_visualizer (const rclcpp::NodeOptions &node_options);

   private:
    void                                                          timer_callback ();
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher;
    visualization_msgs::msg::Marker                               marker_msg;
    rclcpp::TimerBase::SharedPtr                                  timer;
};
}  // namespace stl_visualizer

#endif