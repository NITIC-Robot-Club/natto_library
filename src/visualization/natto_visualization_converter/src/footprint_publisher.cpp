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

#include "natto_visualization_converter/footprint_publisher.hpp"

namespace footprint_publisher {

footprint_publisher::footprint_publisher (const rclcpp::NodeOptions &node_options) : Node ("footprint_publisher", node_options) {
    footprint_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped> ("footprint", 10);

    std::vector<double> footprint_points_x = this->declare_parameter<std::vector<double>> ("footprint_points_x", {0.5, 0.5, -0.5, -0.5});
    std::vector<double> footprint_points_y = this->declare_parameter<std::vector<double>> ("footprint_points_y", {0.5, -0.5, -0.5, 0.5});

    footprint_.header.frame_id = this->declare_parameter<std::string> ("frame_id", "base_link");
    footprint_.polygon.points.clear ();
    for (size_t i = 0; i < footprint_points_x.size () && i < footprint_points_y.size (); ++i) {
        geometry_msgs::msg::Point32 point;
        point.x = static_cast<float> (footprint_points_x[i]);
        point.y = static_cast<float> (footprint_points_y[i]);
        point.z = 0.0f;
        footprint_.polygon.points.push_back (point);
    }

    double frequency = this->declare_parameter<double> ("frequency", 100.0);

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&footprint_publisher::timer_callback, this));
}

void footprint_publisher::timer_callback () {
    footprint_.header.stamp = this->get_clock ()->now ();
    footprint_publisher_->publish (footprint_);
}

}  // namespace footprint_publisher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (footprint_publisher::footprint_publisher)