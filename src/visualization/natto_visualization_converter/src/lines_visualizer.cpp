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

#include "natto_visualization_converter/lines_visualizer.hpp"

namespace lines_visualizer {

lines_visualizer::lines_visualizer (const rclcpp::NodeOptions &options) : Node ("lines_visualizer", options) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    lines_sub_   = this->create_subscription<natto_msgs::msg::LineArray> ("lines", 10, std::bind (&lines_visualizer::lines_callback, this, std::placeholders::_1));

    int publish_period_ms = this->declare_parameter<int> ("publish_period_ms", 10);
    line_length_          = this->declare_parameter<double> ("line_length", 10.0);
    line_width_           = this->declare_parameter<double> ("line_width", 0.05);
    frame_id_             = this->declare_parameter<std::string> ("frame_id", "");

    timer_ = this->create_wall_timer (std::chrono::milliseconds (publish_period_ms), std::bind (&lines_visualizer::timer_callback, this));
}

void lines_visualizer::lines_callback (const natto_msgs::msg::LineArray::SharedPtr msg) {
    marker_array_.markers.clear ();

    int id = 0;
    for (const auto &line : msg->lines) {
        double a    = line.a;
        double b    = line.b;
        double c    = line.c;
        double norm = std::sqrt (a * a + b * b);
        if (norm == 0.0) continue;

        // 法線ベクトル (a,b) に垂直な方向ベクトル (−b,a)
        double dir_x = -b / norm;
        double dir_y = a / norm;

        // 線分中心: ax + by + c = 0 → 原点からの距離 = -c/norm
        double offset = -c / norm;
        double cx     = a / norm * offset;
        double cy     = b / norm * offset;

        // 長さ line_length_ の半分を使って両端を算出
        double                    half = line_length_ / 2.0;
        geometry_msgs::msg::Point p1, p2;
        p1.x = cx + dir_x * half;
        p1.y = cy + dir_y * half;
        p1.z = 0.0;
        p2.x = cx - dir_x * half;
        p2.y = cy - dir_y * half;
        p2.z = 0.0;

        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id_;
        m.header.stamp    = this->now ();
        m.ns              = "lines";
        m.id              = id++;
        m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.scale.x         = line_width_;
        m.color.r         = 0.0f;
        m.color.g         = 1.0f;
        m.color.b         = 0.0f;
        m.color.a         = 1.0f;
        m.points          = {p1, p2};
        m.lifetime        = rclcpp::Duration::from_seconds (0.01);

        marker_array_.markers.push_back (m);
    }
}

void lines_visualizer::timer_callback () {
    marker_pub_->publish (marker_array_);
}

}  // namespace lines_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (lines_visualizer::lines_visualizer)
