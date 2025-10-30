#ifndef __MAP_CONVERTER_HPP__
#define __MAP_CONVERTER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace map_converter {
class map_converter : public rclcpp::Node {
   public:
    map_converter (const rclcpp::NodeOptions &node_options);

   private:
    double resolution_;

    void map_callback (const natto_msgs::msg::Map::SharedPtr msg);

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Map>::SharedPtr      map_subscription_;
};
}  // namespace map_converter

#endif  // __MAP_CONVERTER_HPP__