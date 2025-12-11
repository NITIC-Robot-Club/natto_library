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

#include "natto_lidar_converter/laserscan_to_pointcloud2.hpp"

namespace laserscan_to_pointcloud2 {

laserscan_to_pointcloud2::laserscan_to_pointcloud2 (const rclcpp::NodeOptions &node_options) : Node ("laserscan_to_pointcloud2", node_options) {
    pointcloud2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2> ("pointcloud2", rclcpp::SensorDataQoS ());
    laserscan_subscriber_  = this->create_subscription<sensor_msgs::msg::LaserScan> ("laserscan", rclcpp::SensorDataQoS (), std::bind (&laserscan_to_pointcloud2::laserscan_callback, this, std::placeholders::_1));

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    frame_id_ = this->declare_parameter<std::string> ("frame_id", "pointcloud2_frame");

    RCLCPP_INFO (this->get_logger (), "laserscan_to_pointcloud2 node has been started.");
    RCLCPP_INFO (this->get_logger (), "frame id: %s", frame_id_.c_str ());
}

void laserscan_to_pointcloud2::laserscan_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2 pointcloud2;
    pointcloud2.header = msg->header;

    sensor_msgs::PointCloud2Modifier pointcloud2_modifier (pointcloud2);
    pointcloud2_modifier.setPointCloud2FieldsByString (1, "xyz");
    pointcloud2_modifier.resize (msg->ranges.size ());

    sensor_msgs::PointCloud2Iterator<float> iter_x (pointcloud2, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y (pointcloud2, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z (pointcloud2, "z");

    for (int i = 0; i < msg->ranges.size (); ++i, ++iter_x, ++iter_y, ++iter_z) {
        float r = msg->ranges[i];
        if (r < msg->range_min || r > msg->range_max) {
            *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN ();
            continue;
        }
        float angle = msg->angle_min + i * msg->angle_increment;
        *iter_x     = r * std::cos (angle);
        *iter_y     = r * std::sin (angle);
        *iter_z     = 0.0;
    }

    try {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform (frame_id_, msg->header.frame_id, msg->header.stamp);
        sensor_msgs::msg::PointCloud2        pointcloud2_transformed;
        tf2::doTransform (pointcloud2, pointcloud2_transformed, tf);
        pointcloud2_publisher_->publish (pointcloud2_transformed);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN (this->get_logger (), "Transform error: %s", ex.what ());
        return;
    }
}

}  // namespace laserscan_to_pointcloud2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (laserscan_to_pointcloud2::laserscan_to_pointcloud2)