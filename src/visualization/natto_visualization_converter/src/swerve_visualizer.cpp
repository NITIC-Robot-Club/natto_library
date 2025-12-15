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

#include "natto_visualization_converter/swerve_visualizer.hpp"

namespace swerve_visualizer {

swerve_visualizer::swerve_visualizer (const rclcpp::NodeOptions &node_options) : Node ("swerve_visualizer", node_options) {
    marker_publisher_    = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    swerve_subscription_ = this->create_subscription<natto_msgs::msg::Swerve> ("swerve", 10, std::bind (&swerve_visualizer::swerve_callback, this, std::placeholders::_1));

    double frequency = this->declare_parameter<double> ("frequency", 100.0);
    arrow_r          = this->declare_parameter<double> ("arrow_r", 0.0);
    arrow_g          = this->declare_parameter<double> ("arrow_g", 1.0);
    arrow_b          = this->declare_parameter<double> ("arrow_b", 0.0);
    arrow_scale      = this->declare_parameter<double> ("arrow_scale", 0.2);
    arrow_min_size   = this->declare_parameter<double> ("arrow_min_size", 0.1);

    wheel_position_x_ = this->declare_parameter<std::vector<double>> ("wheel_position_x", {0.5, -0.5, -0.5, 0.5});
    wheel_position_y_ = this->declare_parameter<std::vector<double>> ("wheel_position_y", {0.5, 0.5, -0.5, -0.5});

    timer_      = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&swerve_visualizer::timer_callback, this));
    num_wheels_ = wheel_position_x_.size ();
    if (wheel_position_y_.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_position_x and wheel_position_y must have the same size.");
        throw std::runtime_error ("wheel_position_x and wheel_position_y must have the same size.");
    }

    RCLCPP_INFO (this->get_logger (), "swerve_visualizer node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frequency : %.2f", frequency);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %zu", num_wheels_);
    RCLCPP_INFO (this->get_logger (), "arrow_color: (%.2f, %.2f, %.2f)", arrow_r, arrow_g, arrow_b);
    RCLCPP_INFO (this->get_logger (), "arrow_scale: %.2f", arrow_scale);
    for (size_t i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "wheel_position_xy[%zu]: (%.2f, %.2f)", i, wheel_position_x_[i], wheel_position_y_[i]);
    }
}

void swerve_visualizer::swerve_callback (const natto_msgs::msg::Swerve::SharedPtr msg) {
    marker_array_.markers.clear ();
    for (size_t i = 0; i < num_wheels_; i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp    = this->now ();
        marker.ns              = "swerve_wheel";
        marker.id              = static_cast<int>(i);
        marker.type            = visualization_msgs::msg::Marker::ARROW;
        marker.action          = visualization_msgs::msg::Marker::ADD;

        double wheel_x     = wheel_position_x_[i];
        double wheel_y     = wheel_position_y_[i];
        double wheel_angle = msg->wheel_angle[i];
        double wheel_speed = msg->wheel_speed[i];

        if (std::signbit (wheel_speed)) {
            wheel_angle += M_PI;
        }

        wheel_speed = std::abs (wheel_speed);

        marker.pose.position.x = wheel_x;
        marker.pose.position.y = wheel_y;
        marker.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY (0, 0, wheel_angle);
        marker.pose.orientation.x = q.x ();
        marker.pose.orientation.y = q.y ();
        marker.pose.orientation.z = q.z ();
        marker.pose.orientation.w = q.w ();

        marker.scale.x = std::max (arrow_scale * wheel_speed, arrow_min_size);
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.r = static_cast<float>(arrow_r);
        marker.color.g = static_cast<float>(arrow_g);
        marker.color.b = static_cast<float>(arrow_b);
        marker.color.a = 1.0f;

        marker_array_.markers.push_back (marker);
    }
}

void swerve_visualizer::timer_callback () {
    marker_publisher_->publish (marker_array_);
}

}  // namespace swerve_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (swerve_visualizer::swerve_visualizer)