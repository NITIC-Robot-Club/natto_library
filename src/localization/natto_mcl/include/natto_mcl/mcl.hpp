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

#ifndef __MCL_HPP__
#define __MCL_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <random>

namespace mcl {

struct particle {
    double x;
    double y;
    double theta;
    double weight;
};

class mcl : public rclcpp::Node {
   public:
    mcl (const rclcpp::NodeOptions &node_options);

   private:
    std::string map_frame_id_, odom_frame_id_, base_frame_id_;
    int         max_num_particles_, min_num_particles_;
    double      initial_pose_x_, initial_pose_y_, initial_pose_theta_, motion_noise_position_, motion_noise_orientation_, expansion_radius_position_, expansion_radius_orientation_, laser_likelihood_max_dist_, transform_tolerance_, resolution_;

    std::vector<particle>             particles_;
    geometry_msgs::msg::Transform     last_odom_transform_;
    std::vector<std::vector<uint8_t>> likelihood_field_;

    void   occupancy_grid_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void   pointcloud2_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void   initial_pose_with_covariance_callback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void   initialize_particles (double x, double y, double theta);
    void   motion_update (double delta_x, double delta_y, double delta_theta);
    void   resample_particles ();
    double compute_laser_likelihood (const sensor_msgs::msg::PointCloud2 &scan, const particle &p);

    geometry_msgs::msg::Pose get_mean_pose ();

    std::default_random_engine rng_;

    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr                  pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr                    particles_publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr                  occupancy_grid_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                 pointcloud2_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_with_covariance_subscriber_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr likelihood_field_publisher_;
};
}  // namespace mcl

#endif  // __MCL_HPP__