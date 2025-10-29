#ifndef __LASERSCAN_TO_POINTCLOUD2_HPP__
#define __LASERSCAN_TO_POINTCLOUD2_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace laserscan_to_pointcloud2 {
class laserscan_to_pointcloud2 : public rclcpp::Node {
   public:
    laserscan_to_pointcloud2 (const rclcpp::NodeOptions &node_options);

   private:
    std::string frame_id_;

    void laser_scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg);

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  pointcloud2_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_subscriber_;
};
}  // namespace laserscan_to_pointcloud2

#endif  // __LASERSCAN_TO_POINTCLOUD2_HPP__