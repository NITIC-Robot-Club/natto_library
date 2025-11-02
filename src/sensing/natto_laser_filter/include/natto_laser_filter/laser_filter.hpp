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

#ifndef __LASER_FILTER_HPP__
#define __LASER_FILTER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

namespace laser_filter {
class laser_filter : public rclcpp::Node {
   public:
    laser_filter (const rclcpp::NodeOptions &node_options);

   private:
    double threshold_;

    void scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
};
}  // namespace laser_filter

#endif  // __LASER_FILTER_HPP__
