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

#ifndef __MAP_CONVERTER_HPP__
#define __MAP_CONVERTER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

namespace map_converter {
class map_converter : public rclcpp::Node {
   public:
    map_converter (const rclcpp::NodeOptions &node_options);

   private:
    double resolution_;

    void map_callback (const natto_msgs::msg::Map::SharedPtr msg);

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Map>::SharedPtr      map_subscription_;
};
}  // namespace map_converter

#endif  // __MAP_CONVERTER_HPP__