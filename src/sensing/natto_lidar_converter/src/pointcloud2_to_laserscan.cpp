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

#include "natto_lidar_converter/pointcloud2_to_laserscan.hpp"

namespace pointcloud2_to_laserscan {

pointcloud2_to_laserscan::pointcloud2_to_laserscan (const rclcpp::NodeOptions &node_options) : Node ("pointcloud2_to_laserscan", node_options) {
    laserscan_publisher_    = this->create_publisher<sensor_msgs::msg::LaserScan> ("laserscan", rclcpp::SensorDataQoS ());
    pointcloud2_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2> ("pointcloud2", rclcpp::SensorDataQoS (), std::bind (&pointcloud2_to_laserscan::pointcloud2_callback, this, std::placeholders::_1));

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    frame_id_        = this->declare_parameter<std::string> ("frame_id", "pointcloud2_frame");
    angle_increment_ = this->declare_parameter<double> ("angle_increment", M_PI / 180.0);
    range_min_       = this->declare_parameter<double> ("range_min", 0.0);
    range_max_       = this->declare_parameter<double> ("range_max", 30.0);

    RCLCPP_INFO (this->get_logger (), "pointcloud2_to_laserscan node has been started.");
    RCLCPP_INFO (this->get_logger (), "frame id: %s", frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "angle increment: %f", angle_increment_);
    RCLCPP_INFO (this->get_logger (), "range min: %f", range_min_);
    RCLCPP_INFO (this->get_logger (), "range max: %f", range_max_);
}

void pointcloud2_to_laserscan::pointcloud2_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2 cloud_transformed;
    try {
        geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform (frame_id_, msg->header.frame_id, msg->header.stamp);
        tf2::doTransform (*msg, cloud_transformed, tf_stamped);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN (this->get_logger (), "Transform error: %s", ex.what ());
        return;
    }

    float min_angle = std::numeric_limits<float>::max ();
    float max_angle = std::numeric_limits<float>::lowest ();

    sensor_msgs::PointCloud2ConstIterator<float> iter_x (cloud_transformed, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y (cloud_transformed, "y");
    for (; iter_x != iter_x.end (); ++iter_x, ++iter_y) {
        float angle = std::atan2 (*iter_y, *iter_x);
        min_angle   = std::min (min_angle, angle);
        max_angle   = std::max (max_angle, angle);
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp    = cloud_transformed.header.stamp;
    scan.header.frame_id = frame_id_;
    scan.angle_min       = min_angle;
    scan.angle_max       = max_angle;
    scan.angle_increment = angle_increment_;
    scan.range_min       = range_min_;
    scan.range_max       = range_max_;
    size_t num_ranges    = static_cast<size_t> ((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
    scan.ranges.assign (num_ranges, std::numeric_limits<float>::infinity ());

    iter_x = sensor_msgs::PointCloud2ConstIterator<float> (cloud_transformed, "x");
    iter_y = sensor_msgs::PointCloud2ConstIterator<float> (cloud_transformed, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z (cloud_transformed, "z");

    for (; iter_x != iter_x.end (); ++iter_x, ++iter_y, ++iter_z) {
        float x     = *iter_x;
        float y     = *iter_y;
        float range = std::hypot (x, y);
        if (range < range_min_ || range > range_max_) continue;

        float angle = std::atan2 (y, x);
        int   index = static_cast<int> ((angle - scan.angle_min) / scan.angle_increment);
        if (index >= 0 && index < static_cast<int> (num_ranges)) {
            if (range < scan.ranges[index]) {
                scan.ranges[index] = range;
            }
        }
    }

    laserscan_publisher_->publish (scan);
}

}  // namespace pointcloud2_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (pointcloud2_to_laserscan::pointcloud2_to_laserscan)