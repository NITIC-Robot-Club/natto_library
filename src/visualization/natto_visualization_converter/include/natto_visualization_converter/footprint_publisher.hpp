#ifndef __FOOTPRINT_PUBLISHER_HPP__
#define __FOOTPRINT_PUBLISHER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace footprint_publisher {
class footprint_publisher : public rclcpp::Node {
   public:
    footprint_publisher (const rclcpp::NodeOptions &node_options);

   private:
    geometry_msgs::msg::PolygonStamped footprint_;

    void timer_callback ();

    // メンバ変数宣言例:
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_publisher_;
    rclcpp::TimerBase::SharedPtr                                     timer_;
};
}  // namespace footprint_publisher

#endif  // __FOOTPRINT_PUBLISHER_HPP__