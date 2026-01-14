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

#ifndef __JOINT_STATE_SIMULATOR_HPP__
#define __JOINT_STATE_SIMULATOR_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace joint_state_simulator {
class joint_state_simulator : public rclcpp::Node {
   public:
    joint_state_simulator (const rclcpp::NodeOptions &node_options);

   private:
    std::string chassis_type_;
    bool        infinite_swerve_mode_;
    double      wheel_radius_;
    size_t      num_wheels_;
    size_t      num_joints_;
    double      frequency_;

    std::vector<std::string> wheel_names_;
    std::vector<std::string> wheel_base_names_;

    double initial_pose_x_   = 0.0;
    double initial_pose_y_   = 0.0;
    double initial_pose_yaw_ = 0.0;

    std::vector<std::string> joint_names_;
    std::vector<std::string> control_modes_;
    std::vector<double>      initial_positions_;
    std::vector<double>      joint_position_tau_;
    std::vector<double>      joint_velocity_tau_;
    std::vector<double>      joint_velocity_max_;

    sensor_msgs::msg::JointState command_;
    sensor_msgs::msg::JointState current_;

    geometry_msgs::msg::PoseStamped current_pose_;

    void timer_callback ();
    void command_joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    joint_state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr simulation_pose_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_joint_state_subscriber_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
};
}  // namespace joint_state_simulator

#endif  // __JOINT_STATE_SIMULATOR_HPP__