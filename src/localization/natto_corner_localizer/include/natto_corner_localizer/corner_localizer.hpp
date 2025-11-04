#ifndef __CORNER_LOCALIZER_HPP__
#define __CORNER_LOCALIZER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "natto_msgs/msg/corner_array.hpp"
#include "natto_msgs/msg/map.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace corner_localizer {
class corner_localizer : public rclcpp::Node {
   public:
    corner_localizer (const rclcpp::NodeOptions &node_options);

   private:
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string map_frame_id_;

    natto_msgs::msg::CornerArray         map_corners_;
    geometry_msgs::msg::PoseStamped      current_pose_;
    geometry_msgs::msg::TransformStamped last_map_to_odom_;

    bool is_same_corner (natto_msgs::msg::Corner corner1, natto_msgs::msg::Corner corner2);
    void corner_callback (const natto_msgs::msg::CornerArray::SharedPtr msg);
    void map_callback (const natto_msgs::msg::Map::SharedPtr msg);

    std::unique_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<natto_msgs::msg::CornerArray>::SharedPtr    debug_coner_publisher_;
    rclcpp::Subscription<natto_msgs::msg::CornerArray>::SharedPtr corner_subscriber_;
    rclcpp::Subscription<natto_msgs::msg::Map>::SharedPtr         map_subscriber_;
};
}  // namespace corner_localizer

#endif  // __CORNER_LOCALIZER_HPP__