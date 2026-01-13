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

#ifndef __LIDAR_SIMULATOR_HPP__
#define __LIDAR_SIMULATOR_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "natto_msgs/msg/map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <random>

namespace lidar_simulator {
class lidar_simulator : public rclcpp::Node {
   public:
    lidar_simulator (const rclcpp::NodeOptions &node_options);

   private:
    double      position_x_, position_y_, position_z_, angle_, range_min_, range_max_, angle_min_, angle_max_, simulation_resolution_, frequency_;
    int         point_rate_;
    std::string frame_id_;

    geometry_msgs::msg::PoseStamped::SharedPtr simulation_pose_;
    natto_msgs::msg::Map::SharedPtr            map_;

    std::default_random_engine             generator_;
    std::uniform_real_distribution<double> distribution_;

    void simulation_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void map_callback (const natto_msgs::msg::Map::SharedPtr msg);
    void timer_callback ();

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr        laser_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr simulation_pose_subscriber_;
    rclcpp::Subscription<natto_msgs::msg::Map>::SharedPtr            map_subscriber_;
    rclcpp::TimerBase::SharedPtr                                     timer_;
};
}  // namespace lidar_simulator

#endif  // __LIDAR_SIMULATOR_HPP__