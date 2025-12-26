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

#ifndef __TWO_WHEEL_CALCULATOR_HPP__
#define __TWO_WHEEL_CALCULATOR_HPP__

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "natto_msgs/msg/two_wheel.hpp"

namespace two_wheel_calculator {
class two_wheel_calculator : public rclcpp::Node {
   public:
    two_wheel_calculator (const rclcpp::NodeOptions &node_options);

   private:
    double wheel_radius_;
    double wheel_base_;

    void command_velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    rclcpp::Publisher<natto_msgs::msg::TwoWheel>::SharedPtr               two_wheel_command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_subscriber_;
};
}  // namespace two_wheel_calculator

#endif  // __TWO_WHEEL_CALCULATOR_HPP__
