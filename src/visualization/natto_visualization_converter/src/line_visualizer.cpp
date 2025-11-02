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

#include "natto_visualization_converter/line_visualizer.hpp"

namespace line_visualizer {

line_visualizer::line_visualizer (const rclcpp::NodeOptions &options) : Node ("line_visualizer", options) {
    line_length_ = this->declare_parameter<double> ("line_length", 10.0);

    line_sub_ = this->create_subscription<natto_msgs::msg::LineArray> ("lines", 10, std::bind (&line_visualizer::line_callback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray> ("line_markers", 10);
}

void line_visualizer::line_callback (const natto_msgs::msg::LineArray::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.clear ();

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
        m.header.frame_id = "base_link";
        m.header.stamp    = this->now ();
        m.ns              = "line_visualizer";
        m.id              = id++;
        m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.scale.x         = 0.05;
        m.color.r         = 0.0f;
        m.color.g         = 1.0f;
        m.color.b         = 0.0f;
        m.color.a         = 1.0f;
        m.points          = {p1, p2};
        m.lifetime       = rclcpp::Duration::from_seconds (0.01);

        marker_array.markers.push_back (m);
    }

    marker_pub_->publish (marker_array);
}

}  // namespace line_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (line_visualizer::line_visualizer)
