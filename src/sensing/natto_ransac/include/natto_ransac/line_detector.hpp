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

#ifndef __LINE_DETECTOR_HPP__
#define __LINE_DETECTOR_HPP__

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "natto_msgs/msg/line_array.hpp"
#include "natto_msgs/msg/line_segment_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <cmath>
#include <cstring>
#include <random>
#include <vector>

namespace line_detector {
class line_detector : public rclcpp::Node {
   public:
    line_detector (const rclcpp::NodeOptions &node_options);

   private:
    int    max_iterations_, max_lines_, min_inliers_;
    double distance_threshold_;

    sensor_msgs::msg::PointCloud2     raw_data_;
    sensor_msgs::msg::PointCloud2     data_;
    natto_msgs::msg::LineArray        lines_;
    natto_msgs::msg::LineSegmentArray line_segments_;
    geometry_msgs::msg::PoseArray     corners_;

    void process_pointcloud ();
    void calculate_corner ();
    void calculate_line_segment ();

    void pointcloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Publisher<natto_msgs::msg::LineArray>::SharedPtr        lines_publisher_;
    rclcpp::Publisher<natto_msgs::msg::LineSegmentArray>::SharedPtr line_segments_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr     corners_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  pointcloud_subscriber_;
};
}  // namespace line_detector

#endif  // __LINE_DETECTOR_HPP__