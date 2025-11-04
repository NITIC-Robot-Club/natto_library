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

#ifndef __LINES_VISUALIZER_HPP__
#define __LINES_VISUALIZER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/line_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lines_visualizer {

class lines_visualizer : public rclcpp::Node {
   public:
    explicit lines_visualizer (const rclcpp::NodeOptions &options);

   private:
    std::string frame_id_;
    double      line_length_, line_width_;

    visualization_msgs::msg::MarkerArray marker_array_;

    void lines_callback (const natto_msgs::msg::LineArray::SharedPtr msg);
    void timer_callback ();

    rclcpp::Subscription<natto_msgs::msg::LineArray>::SharedPtr        lines_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
};

}  // namespace line_visualizer

#endif  // __LINE_VISUALIZER_HPP__
