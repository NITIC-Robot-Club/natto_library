// Copyright 2026 Kazusa Hashimoto
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

#ifndef __STEERING_VECTOR_VISUALIZER_HPP__
#define __STEERING_VECTOR_VISUALIZER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <string>
#include <vector>

namespace steering_vector_visualizer {

class steering_vector_visualizer : public rclcpp::Node {
   public:
    explicit steering_vector_visualizer (const rclcpp::NodeOptions &options);

   private:
    std::string                  chassis_type_;
    bool                         infinite_swerve_mode_;
    double                       wheel_radius_;
    std::string                  frame_id_;
    double                       line_width_;
    double                       vector_scale_;
    double                       arrow_color_r_;
    double                       arrow_color_g_;
    double                       arrow_color_b_;
    double                       arrow_color_a_;
    std::vector<std::string>     wheel_names_;
    std::vector<std::string>     wheel_base_names_;
    sensor_msgs::msg::JointState joint_state_;

    visualization_msgs::msg::MarkerArray marker_array_;

    void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);
    void timer_callback ();

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr      joint_state_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
};

}  // namespace steering_vector_visualizer

#endif  // __STEERING_VECTOR_VISUALIZER_HPP__
