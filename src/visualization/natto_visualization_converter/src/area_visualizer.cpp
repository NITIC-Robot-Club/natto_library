// Copyright 2026 Kazusa Hashimoto
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

#include <algorithm>
#include <chrono>
#include <cmath>

#include "natto_visualization_converter/area_visualizer.hpp"

namespace area_visualizer {

area_visualizer::area_visualizer (const rclcpp::NodeOptions &options) : Node ("area_visualizer", options) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker> ("marker", 10);

    const double frequency = this->declare_parameter<double> ("frequency", 100.0);
    const auto   frame_id  = this->declare_parameter<std::string> ("frame_id", "map");

    const double start_x = this->declare_parameter<double> ("start.x", 0.0);
    const double start_y = this->declare_parameter<double> ("start.y", 0.0);
    const double end_x   = this->declare_parameter<double> ("end.x", 1.0);
    const double end_y   = this->declare_parameter<double> ("end.y", 1.0);
    const double z       = this->declare_parameter<double> ("z", 0.1);

    const double color_red   = this->declare_parameter<double> ("color.red", 1.0);
    double       color_green = this->declare_parameter<double> ("color.green", std::nan(""));
    const double color_geen  = this->declare_parameter<double> ("color.geen", 0.0);
    const double color_blue  = this->declare_parameter<double> ("color.blue", 0.0);

    if (std::isnan (color_green)) {
        color_green = color_geen;
    }

    const double center_x  = (start_x + end_x) * 0.5;
    const double center_y  = (start_y + end_y) * 0.5;
    const double thickness = std::max (z, 0.001);

    marker_.header.frame_id         = frame_id;
    marker_.ns                      = "area_visualizer";
    marker_.id                      = 0;
    marker_.type                    = visualization_msgs::msg::Marker::CUBE;
    marker_.action                  = visualization_msgs::msg::Marker::ADD;
    marker_.pose.position.x         = center_x;
    marker_.pose.position.y         = center_y;
    marker_.pose.position.z         = thickness * 0.5;
    marker_.pose.orientation.w      = 1.0;
    marker_.scale.x                 = std::abs (end_x - start_x);
    marker_.scale.y                 = std::abs (end_y - start_y);
    marker_.scale.z                 = thickness;
    marker_.color.r                 = static_cast<float> (color_red);
    marker_.color.g                 = static_cast<float> (color_green);
    marker_.color.b                 = static_cast<float> (color_blue);
    marker_.color.a                 = 1.0f;

    RCLCPP_INFO (this->get_logger (), "area_visualizer node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency);
    RCLCPP_INFO (this->get_logger (), "frame_id: %s", frame_id.c_str ());
    RCLCPP_INFO (this->get_logger (), "start: (%.3f, %.3f)", start_x, start_y);
    RCLCPP_INFO (this->get_logger (), "end: (%.3f, %.3f)", end_x, end_y);
    RCLCPP_INFO (this->get_logger (), "z: %.3f", z);
    RCLCPP_INFO (this->get_logger (), "color.rgb: (%.3f, %.3f, %.3f)", color_red, color_green, color_blue);

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&area_visualizer::timer_callback, this));
}

void area_visualizer::timer_callback () {
    marker_.header.stamp = this->now ();
    marker_pub_->publish (marker_);
}

}  // namespace area_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (area_visualizer::area_visualizer)