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

#ifndef __SWERVE_CALCULATOR_HPP__
#define __SWERVE_CALCULATOR_HPP__

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "natto_msgs/msg/swerve.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace swerve_calculator {
class swerve_calculator : public rclcpp::Node {
   public:
    swerve_calculator (const rclcpp::NodeOptions &node_options);

   private:
    bool   infinite_swerve_mode_;
    double wheel_radius_;
    size_t num_wheels_;

    std::vector<double>     wheel_position_x_;
    std::vector<double>     wheel_position_y_;
    natto_msgs::msg::Swerve swerve_result_;

    bool                         publish_joint_state_;
    std::vector<std::string>     steer_names_;
    std::vector<std::string>     wheel_names_;
    sensor_msgs::msg::JointState joint_state_msg_;

    void command_velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void swerve_result_callback (const natto_msgs::msg::Swerve::SharedPtr msg);

    rclcpp::Publisher<natto_msgs::msg::Swerve>::SharedPtr             swerve_command_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        joint_state_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_subscriber_;
    rclcpp::Subscription<natto_msgs::msg::Swerve>::SharedPtr          swerve_result_subscriber_;
};
}  // namespace swerve_calculator

#endif  // __SWERVE_CALCULATOR_HPP__