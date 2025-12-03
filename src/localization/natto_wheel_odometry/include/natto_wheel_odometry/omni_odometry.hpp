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

#ifndef __OMNI_ODOMETRY_HPP__
#define __OMNI_ODOMETRY_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "natto_msgs/msg/omni.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace omni_odometry {
class omni_odometry : public rclcpp::Node {
   public:
    omni_odometry (const rclcpp::NodeOptions &node_options);

   private:
    double      wheel_radius_;
    int         num_wheels_;
    std::string frame_id_;
    std::string child_frame_id_;
    bool        publish_tf_;

    std::vector<double> wheel_position_x;
    std::vector<double> wheel_position_y;
    std::vector<double> wheel_angle;

    geometry_msgs::msg::PoseStamped last_pose;

    void omni_callback (const natto_msgs::msg::Omni::SharedPtr msg);

    // メンバ変数宣言例:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Omni>::SharedPtr       omni_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                 tf_broadcaster_;
};
}  // namespace omni_odometry

#endif  // __OMNI_ODOMETRY_HPP__