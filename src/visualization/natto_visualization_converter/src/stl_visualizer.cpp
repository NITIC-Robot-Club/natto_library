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

#include "natto_visualization_converter/stl_visualizer.hpp"

#include <chrono>

namespace stl_visualizer {

namespace {

std::string to_mesh_resource_uri (const std::string &file_path) {
    if (file_path.empty ()) {
        return file_path;
    }

    if (file_path.rfind ("file://", 0) == 0 || file_path.rfind ("package://", 0) == 0 || file_path.rfind ("http://", 0) == 0 || file_path.rfind ("https://", 0) == 0) {
        return file_path;
    }

    return "file://" + file_path;
}

}  // namespace

stl_visualizer::stl_visualizer (const rclcpp::NodeOptions &options) : Node ("stl_visualizer", options) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker> ("marker", 10);

    double frequency = this->declare_parameter<double> ("frequency", 100.0);
    frame_id_        = this->declare_parameter<std::string> ("frame_id", "map");
    file_path_       = this->declare_parameter<std::string> ("file_path", "");

    const double position_x = this->declare_parameter<double> ("position.x", 0.0);
    const double position_y = this->declare_parameter<double> ("position.y", 0.0);
    const double position_z = this->declare_parameter<double> ("position.z", 0.0);
    const double qx         = this->declare_parameter<double> ("quaternion.x", 0.0);
    const double qy         = this->declare_parameter<double> ("quaternion.y", 0.0);
    const double qz         = this->declare_parameter<double> ("quaternion.z", 0.0);
    const double qw         = this->declare_parameter<double> ("quaternion.w", 1.0);
    const double scale_x    = this->declare_parameter<double> ("scale.x", 1.0);
    const double scale_y    = this->declare_parameter<double> ("scale.y", 1.0);
    const double scale_z    = this->declare_parameter<double> ("scale.z", 1.0);
    const double color_r    = this->declare_parameter<double> ("color.red", 1.0);
    const double color_g    = this->declare_parameter<double> ("color.green", 1.0);
    const double color_b    = this->declare_parameter<double> ("color.blue", 1.0);
    const double alpha      = this->declare_parameter<double> ("alpha", 1.0);

    const bool   reverse_y        = this->declare_parameter<bool> ("reverse_y", false);
    const double reverse_y_offset = this->declare_parameter<double> ("reverse_y_offset", 0.0);

    marker_.header.frame_id             = frame_id_;
    marker_.ns                          = "stl_visualizer";
    marker_.id                          = 0;
    marker_.type                        = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_.action                      = visualization_msgs::msg::Marker::ADD;
    marker_.pose.position.x             = position_x;
    marker_.pose.position.y             = position_y;
    marker_.pose.position.z             = position_z;
    marker_.pose.orientation.x          = qx;
    marker_.pose.orientation.y          = qy;
    marker_.pose.orientation.z          = qz;
    marker_.pose.orientation.w          = qw;
    marker_.scale.x                     = scale_x;
    marker_.scale.y                     = scale_y;
    marker_.scale.z                     = scale_z;
    marker_.color.r                     = static_cast<float> (color_r);
    marker_.color.g                     = static_cast<float> (color_g);
    marker_.color.b                     = static_cast<float> (color_b);
    marker_.color.a                     = static_cast<float> (alpha);
    marker_.mesh_use_embedded_materials = false;
    marker_.mesh_resource               = to_mesh_resource_uri (file_path_);

    if (reverse_y) {
        marker_.pose.position.y *= -1.0;
        marker_.pose.position.y += reverse_y_offset;
        marker_.scale.y *= -1.0;
    }

    RCLCPP_INFO (this->get_logger (), "stl_visualizer node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency);
    RCLCPP_INFO (this->get_logger (), "frame_id: %s", frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "file_path: %s", file_path_.c_str ());
    RCLCPP_INFO (this->get_logger (), "position: (%.3f, %.3f, %.3f)", position_x, position_y, position_z);
    RCLCPP_INFO (this->get_logger (), "quaternion: (%.3f, %.3f, %.3f, %.3f)", qx, qy, qz, qw);
    RCLCPP_INFO (this->get_logger (), "scale: (%.3f, %.3f, %.3f)", scale_x, scale_y, scale_z);
    RCLCPP_INFO (this->get_logger (), "alpha: %.3f", alpha);

    if (file_path_.empty ()) {
        RCLCPP_WARN (this->get_logger (), "file_path parameter is empty.");
    }

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&stl_visualizer::timer_callback, this));
}

void stl_visualizer::timer_callback () {
    marker_.header.stamp = this->now ();
    marker_pub_->publish (marker_);
}

}  // namespace stl_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (stl_visualizer::stl_visualizer)