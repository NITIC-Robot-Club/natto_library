// Copyright 2025 Kazusa Hashimoto
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "natto_visualization_converter/map_visualizer.hpp"

namespace map_visualizer {

map_visualizer::map_visualizer (const rclcpp::NodeOptions &node_options) : Node ("map_visualizer", node_options) {
    marker_publisher_     = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    map_subscription_     = this->create_subscription<natto_msgs::msg::Map> ("map", rclcpp::QoS (rclcpp::KeepLast (1)).transient_local ().reliable (), std::bind (&map_visualizer::map_callback, this, std::placeholders::_1));
    double frequency = this->declare_parameter<double> ("frequency", 100.0);
    timer_                = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&map_visualizer::timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "map_visualizer node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frequency : %.2f", frequency);
}

void map_visualizer::map_callback (const natto_msgs::msg::Map::SharedPtr msg) {
    int id = 0;
    marker_array_.markers.clear ();
    for (const auto &line_segment : msg->line_segments.line_segments) {
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

    for (const auto &circle : msg->circles.circles) {
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

void map_visualizer::timer_callback () {
    marker_publisher_->publish (marker_array_);
}

}  // namespace map_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (map_visualizer::map_visualizer)