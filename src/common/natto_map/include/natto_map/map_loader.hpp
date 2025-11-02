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

#ifndef __MAP_LOADER_HPP__
#define __MAP_LOADER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/map.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace map_loader {
class map_loader : public rclcpp::Node {
   public:
    map_loader (const rclcpp::NodeOptions &node_options);

   private:
    void timer_callback ();

    void load_line_segments (const std::string &path);
    void load_circles (const std::string &path);

    natto_msgs::msg::Map map_;

    rclcpp::Publisher<natto_msgs::msg::Map>::SharedPtr map_publisher_;
    rclcpp::TimerBase::SharedPtr                       timer_;
};
}  // namespace map_loader

#endif  // __MAP_LOADER_HPP__