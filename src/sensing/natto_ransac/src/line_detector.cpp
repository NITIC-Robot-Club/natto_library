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

#include "natto_ransac/line_detector.hpp"

namespace line_detector {

line_detector::line_detector (const rclcpp::NodeOptions &node_options) : Node ("line_detector", node_options) {
    lines_publisher_               = this->create_publisher<natto_msgs::msg::LineArray> ("lines", 10);
    filtered_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2> ("filtered_pointcloud", 10);
    pointcloud_subscriber_         = this->create_subscription<sensor_msgs::msg::PointCloud2> ("pointcloud", 10, std::bind (&line_detector::pointcloud_callback, this, std::placeholders::_1));

    max_iterations_     = this->declare_parameter<int> ("max_iterations", 100);
    max_lines_          = this->declare_parameter<int> ("max_lines", 10);
    min_inliers_        = this->declare_parameter<int> ("min_inliers", 50);
    distance_threshold_ = this->declare_parameter<double> ("distance_threshold", 0.01);
}

void line_detector::pointcloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    data_ = *msg;
    for (int i = 0; i < max_lines_; i++) {
        process_pointcloud ();
    }
    filtered_pointcloud_publisher_->publish (data_);
    lines_publisher_->publish (lines_);
}

void line_detector::process_pointcloud () {
}

}  // namespace line_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (line_detector::line_detector)
