#include "natto_visualization_converter/visualize_map.hpp"

namespace visualize_map {

visualize_map::visualize_map (const rclcpp::NodeOptions &node_options) : Node ("visualize_map", node_options) {
    marker_publisher_     = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    map_subscription_     = this->create_subscription<natto_msgs::msg::Map> ("map", 10, std::bind (&visualize_map::map_callback, this, std::placeholders::_1));
    int publish_period_ms = this->declare_parameter<int> ("publish_period_ms", 1000);
    timer_                = this->create_wall_timer (std::chrono::milliseconds (publish_period_ms), std::bind (&visualize_map::timer_callback, this));
}

void visualize_map::map_callback (const natto_msgs::msg::Map::SharedPtr msg) {
    int id = 0;
    marker_array_.markers.clear ();
    for (const auto &line_segment : msg->line_segments) {
        visualization_msgs::msg::Marker marker;
        marker.id              = id++;
        marker.header.frame_id = "map";
        marker.header.stamp    = this->now ();
        marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action          = visualization_msgs::msg::Marker::ADD;
        marker.scale.x         = 0.05;
        marker.color.a         = 1.0;

        geometry_msgs::msg::Point p_start;
        p_start.x = line_segment.start.x;
        p_start.y = line_segment.start.y;
        p_start.z = line_segment.start.z;

        geometry_msgs::msg::Point p_end;
        p_end.x = line_segment.end.x;
        p_end.y = line_segment.end.y;
        p_end.z = line_segment.end.z;

        marker.points.push_back (p_start);
        marker.points.push_back (p_end);

        marker_array_.markers.push_back (marker);
    }

    for (const auto &circle : msg->circles) {
        visualization_msgs::msg::Marker marker;
        marker.id              = id++;
        marker.header.frame_id = "map";
        marker.header.stamp    = this->now ();
        marker.type            = visualization_msgs::msg::Marker::CYLINDER;
        marker.action          = visualization_msgs::msg::Marker::ADD;
        marker.scale.x         = circle.radius * 2.0;
        marker.scale.y         = circle.radius * 2.0;
        marker.scale.z         = 0.05;
        marker.color.a         = 1.0;
        marker.pose.position   = circle.center;
        marker_array_.markers.push_back (marker);
    }
}

void visualize_map::timer_callback () {
    marker_publisher_->publish (marker_array_);
}

}  // namespace visualize_map

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (visualize_map::visualize_map)