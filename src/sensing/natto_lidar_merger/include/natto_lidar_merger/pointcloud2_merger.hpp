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

#ifndef __POINTCLOUD2_MERGER_HPP__
#define __POINTCLOUD2_MERGER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace pointcloud2_merger {
class pointcloud2_merger : public rclcpp::Node {
   public:
    pointcloud2_merger (const rclcpp::NodeOptions &node_options);

   private:
    std::string frame_id_;

    geometry_msgs::msg::PolygonStamped footprint_;
    sensor_msgs::msg::PointCloud2      first_pointcloud2_, second_pointcloud2_;

    void pointcloud2_callback_first (const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pointcloud2_callback_second (const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void footprint_callback (const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    bool check_footprint (double x, double y);
    void publish_pointcloud2 ();

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         pointcloud2_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      pointcloud2_first_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      pointcloud2_second_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_subscriber_;
};
}  // namespace pointcloud2_merger

#endif  // __POINTCLOUD2_MERGER_HPP__