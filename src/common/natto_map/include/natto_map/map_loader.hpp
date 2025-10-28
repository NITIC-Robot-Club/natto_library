#ifndef __MAP_LOADER_HPP__
#define __MAP_LOADER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/map.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace map_loader {
class map_loader : public rclcpp::Node {
   public:
    map_loader (const rclcpp::NodeOptions &node_options);

   private:
    void timer_callback ();

    void load_line_segments (const std::string &path);
    void load_circles (const std::string &path);

    natto_msgs::msg::Map map_;

    rclcpp::Publisher<natto_msgs::msg::Map>::SharedPtr map_publisher_;
    rclcpp::TimerBase::SharedPtr                       timer_;
};
}  // namespace map_loader

#endif  // __MAP_LOADER_HPP__