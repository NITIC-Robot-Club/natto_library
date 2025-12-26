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

#ifndef __MECANUM_CALCULATOR_HPP__
#define __MECANUM_CALCULATOR_HPP__

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "natto_msgs/msg/mecanum.hpp"

namespace mecanum_calculator {
class mecanum_calculator : public rclcpp::Node {
   public:
    mecanum_calculator (const rclcpp::NodeOptions &node_options);

   private:
    double wheel_radius_;
    size_t num_wheels_;

    std::vector<double> wheel_position_x_;
    std::vector<double> wheel_position_y_;
    std::vector<double> wheel_angle_;

    void command_velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    rclcpp::Publisher<natto_msgs::msg::Mecanum>::SharedPtr               mecanum_command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_subscriber_;
};
}  // namespace mecanum_calculator

#endif  // __MECANUM_CALCULATOR_HPP__
