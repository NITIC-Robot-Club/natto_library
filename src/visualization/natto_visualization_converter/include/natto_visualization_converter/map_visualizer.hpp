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

#ifndef __MAP_VISUALIZER_HPP__
#define __MAP_VISUALIZER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace map_visualizer {
class map_visualizer : public rclcpp::Node {
   public:
    map_visualizer (const rclcpp::NodeOptions &node_options);

   private:
    visualization_msgs::msg::MarkerArray marker_array_;

    void timer_callback ();
    void map_callback (const natto_msgs::msg::Map::SharedPtr msg);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Map>::SharedPtr              map_subscription_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
};
}  // namespace map_visualizer

#endif  // __MAP_VISUALIZER_HPP__