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

#include "natto_visualization_converter/line_segments_visualizer.hpp"

namespace line_segments_visualizer {

line_segments_visualizer::line_segments_visualizer (const rclcpp::NodeOptions &options) : Node ("line_segments_visualizer", options) {
    marker_pub_        = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    line_segments_sub_ = this->create_subscription<natto_msgs::msg::LineSegmentArray> ("line_segments", 10, std::bind (&line_segments_visualizer::line_segments_callback, this, std::placeholders::_1));

    int publish_period_ms = this->declare_parameter<int> ("publish_period_ms", 10);
    line_width_           = this->declare_parameter<double> ("line_width", 0.05);
    frame_id_             = this->declare_parameter<std::string> ("frame_id", "");

    timer_ = this->create_wall_timer (std::chrono::milliseconds (publish_period_ms), std::bind (&line_segments_visualizer::timer_callback, this));
}

void line_segments_visualizer::line_segments_callback (const natto_msgs::msg::LineSegmentArray::SharedPtr msg) {
    marker_array_.markers.clear ();

    int id = 0;
    for (const auto &seg : msg->line_segments) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id_;
        m.header.stamp    = this->now ();
        m.ns              = "line_segments";
        m.id              = id++;
        m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action          = visualization_msgs::msg::Marker::ADD;

        m.scale.x = line_width_;
        m.color.r = 1.0f;
        m.color.g = 0.0f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;

        m.points.push_back (seg.start);
        m.points.push_back (seg.end);

        marker_array_.markers.push_back (m);
    }
}

void line_segments_visualizer::timer_callback () {
    marker_pub_->publish (marker_array_);
}

}  // namespace line_segments_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (line_segments_visualizer::line_segments_visualizer)
