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

#ifndef __POINTCLOUD2__TO_LASERSCAN_HPP__
#define __POINTCLOUD2__TO_LASERSCAN_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace pointcloud2_to_laserscan {
class pointcloud2_to_laserscan : public rclcpp::Node {
   public:
    pointcloud2_to_laserscan (const rclcpp::NodeOptions &node_options);

   private:
    std::string frame_id_;
    double      angle_increment_, range_min_, range_max_;

    void pointcloud2_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr      laserscan_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_subscriber_;
};
}  // namespace pointcloud2_to_laserscan

#endif  // __POINTCLOUD2__TO_LASERSCAN_HPP__