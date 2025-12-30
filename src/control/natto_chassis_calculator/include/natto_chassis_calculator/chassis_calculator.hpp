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

#ifndef __CHASSIS_CALCULATOR_HPP__
#define __CHASSIS_CALCULATOR_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace chassis_calculator {
class chassis_calculator : public rclcpp::Node {
   public:
    chassis_calculator (const rclcpp::NodeOptions &node_options);

   private:
    std::string chassis_type_;
    bool        infinite_swerve_mode_;
    double      wheel_radius_;
    size_t      num_wheels_;

    sensor_msgs::msg::JointState joint_state_;

    std::vector<std::string>     wheel_names_;
    std::vector<std::string>     wheel_base_names_;
    sensor_msgs::msg::JointState command_joint_state_;

    void command_velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        command_joint_state_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_state_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_subscriber_;
};
}  // namespace chassis_calculator

#endif  // __CHASSIS_CALCULATOR_HPP__