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

#include "natto_visualization_converter/corners_visualizer.hpp"

namespace corners_visualizer {

corners_visualizer::corners_visualizer (const rclcpp::NodeOptions &options) : Node ("corners_visualizer", options) {
    marker_pub_  = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    corners_sub_ = this->create_subscription<natto_msgs::msg::CornerArray> ("corners", 10, std::bind (&corners_visualizer::corners_callback, this, std::placeholders::_1));

    int publish_period_ms = this->declare_parameter<int> ("publish_period_ms", 10);
    frame_id_             = this->declare_parameter<std::string> ("frame_id", "");

    timer_ = this->create_wall_timer (std::chrono::milliseconds (publish_period_ms), std::bind (&corners_visualizer::timer_callback, this));
}

void corners_visualizer::corners_callback (const natto_msgs::msg::CornerArray::SharedPtr msg) {
    marker_array_.markers.clear ();
    int id = 0;
    for (const auto &corner : msg->corners) {
        for (const double yaw : {corner.yaw1, corner.yaw2}) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp    = this->now ();

            m.ns      = "corners";
            m.id      = id++;
            m.scale.x = 0.2;
            m.scale.y = 0.02;
            m.scale.z = 0.02;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.color.a = 1.0;

            m.type               = visualization_msgs::msg::Marker::CYLINDER;
            m.action             = visualization_msgs::msg::Marker::ADD;
            m.pose.position      = corner.position;
            m.pose.orientation.z = std::sin (yaw / 2);
            m.pose.orientation.w = std::cos (yaw / 2);
            m.lifetime           = rclcpp::Duration::from_seconds (0.01);
            marker_array_.markers.push_back (m);
        }
    }
}

void corners_visualizer::timer_callback () {
    marker_pub_->publish (marker_array_);
}

}  // namespace corners_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (corners_visualizer::corners_visualizer)
