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

#ifndef __SWERVE_VISUALIZER_HPP__
#define __SWERVE_VISUALIZER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "natto_msgs/msg/swerve.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace swerve_visualizer {
class swerve_visualizer : public rclcpp::Node {
   public:
    swerve_visualizer (const rclcpp::NodeOptions &node_options);

   private:
    visualization_msgs::msg::MarkerArray marker_array_;

    size_t num_wheels_;
    double arrow_r, arrow_g, arrow_b, arrow_scale, arrow_min_size;

    std::vector<double> wheel_position_x_;
    std::vector<double> wheel_position_y_;

    void timer_callback ();
    void swerve_callback (const natto_msgs::msg::Swerve::SharedPtr msg);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<natto_msgs::msg::Swerve>::SharedPtr           swerve_subscription_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
};
}  // namespace swerve_visualizer

#endif  // __SWERVE_VISUALIZER_HPP__