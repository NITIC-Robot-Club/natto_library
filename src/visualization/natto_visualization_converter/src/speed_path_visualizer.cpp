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

#include "natto_visualization_converter/speed_path_visualizer.hpp"

namespace speed_path_visualizer {

speed_path_visualizer::speed_path_visualizer (const rclcpp::NodeOptions &options) : Node ("speed_path_visualizer", options) {
    marker_pub_     = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    speed_path_sub_ = this->create_subscription<natto_msgs::msg::SpeedPath> ("speed_path", 10, std::bind (&speed_path_visualizer::speed_path_callback, this, std::placeholders::_1));

    double frequency = this->declare_parameter<double> ("frequency", 100.0);
    path_width_      = this->declare_parameter<double> ("path_width", 0.05);
    frame_id_        = this->declare_parameter<std::string> ("frame_id", "map");

    RCLCPP_INFO (this->get_logger (), "speed_path_visualizer node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency);
    RCLCPP_INFO (this->get_logger (), "path_width: %.2f", path_width_);
    RCLCPP_INFO (this->get_logger (), "frame_id: %s", frame_id_.c_str ());

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&speed_path_visualizer::timer_callback, this));
}

void speed_path_visualizer::speed_path_callback (const natto_msgs::msg::SpeedPath::SharedPtr msg) {
    marker_array_.markers.clear ();

    int id = 0;
    marker_array_.markers.clear ();
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = frame_id_;
    path_marker.header.stamp    = this->now ();
    path_marker.lifetime        = rclcpp::Duration (std::chrono::seconds (1));
    path_marker.ns              = "path";
    path_marker.id              = id++;
    path_marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action          = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x         = path_width_;
    path_marker.color.a         = 1.0;
    double max_speed            = 0.0;
    for (const auto &twist : msg->twist) {
        double speed = std::hypot (twist.twist.linear.x, twist.twist.linear.y);
        if (speed > max_speed) {
            max_speed = speed;
        }
    }

    for (size_t i = 0; i < msg->path.size (); ++i) {
        const auto &pose  = msg->path[i];
        const auto &twist = msg->twist[i];
        double      speed = std::hypot (twist.twist.linear.x, twist.twist.linear.y);

        geometry_msgs::msg::Point p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = pose.pose.position.z;
        path_marker.points.push_back (p);

        std_msgs::msg::ColorRGBA color;
        color.a = 1.0f;
        if (max_speed > 0.0) {
            color.r = static_cast<float> (std::max (0.0, 1.0 - speed / max_speed));
            color.g = static_cast<float> (std::min (1.0, speed / max_speed));
            color.b = 0.0f;
        } else {
            color.r = 1.0f;
            color.g = 0.0f;
            color.b = 0.0f;
        }
        path_marker.colors.push_back (color);
    }
    marker_array_.markers.push_back (path_marker);

    visualization_msgs::msg::Marker pose_marker;
    pose_marker.header.frame_id = frame_id_;
    pose_marker.header.stamp    = this->now ();
    pose_marker.ns              = "pose";
    pose_marker.type            = visualization_msgs::msg::Marker::ARROW;
    pose_marker.action          = visualization_msgs::msg::Marker::ADD;
    pose_marker.lifetime        = rclcpp::Duration (std::chrono::seconds (1));
    pose_marker.scale.x         = path_width_ * 3.0;
    pose_marker.scale.y         = path_width_ * 0.2;
    pose_marker.scale.z         = path_width_ * 0.2;
    for (size_t i = 0; i < msg->path.size (); ++i) {
        const auto                     &pose         = msg->path[i];
        visualization_msgs::msg::Marker arrow_marker = pose_marker;
        arrow_marker.id                              = id++;
        arrow_marker.pose                            = pose.pose;
        arrow_marker.color.a                         = 1.0f;
        arrow_marker.color.r                         = 0.0f;
        arrow_marker.color.g                         = 1.0f;
        arrow_marker.color.b                         = 0.0f;
        marker_array_.markers.push_back (arrow_marker);
    }

    visualization_msgs::msg::Marker twist_marker;
    twist_marker.header.frame_id = frame_id_;
    twist_marker.header.stamp    = this->now ();
    twist_marker.ns              = "twist";
    twist_marker.type            = visualization_msgs::msg::Marker::ARROW;
    twist_marker.action          = visualization_msgs::msg::Marker::ADD;
    twist_marker.lifetime        = rclcpp::Duration (std::chrono::seconds (1));
    twist_marker.scale.x         = path_width_ * 0.2f;
    twist_marker.scale.y         = path_width_ * 0.25;
    twist_marker.scale.z         = path_width_ * 0.25;
    for (size_t i = 0; i < msg->path.size (); ++i) {
        const auto &pose  = msg->path[i];
        const auto &twist = msg->twist[i];

        visualization_msgs::msg::Marker arrow_marker = twist_marker;
        arrow_marker.id                              = id++;

        geometry_msgs::msg::Point start_point;
        start_point.x = pose.pose.position.x;
        start_point.y = pose.pose.position.y;

        double yaw = tf2::getYaw (pose.pose.orientation);

        double vx = twist.twist.linear.x;
        double vy = twist.twist.linear.y;

        double vx_global = std::cos (yaw) * vx - std::sin (yaw) * vy;
        double vy_global = std::sin (yaw) * vx + std::cos (yaw) * vy;

        double                    scale = 0.5;
        geometry_msgs::msg::Point end_point;
        end_point.x = start_point.x + vx_global * scale;
        end_point.y = start_point.y + vy_global * scale;

        arrow_marker.points.push_back (start_point);
        arrow_marker.points.push_back (end_point);

        arrow_marker.color.a = 0.5f;
        arrow_marker.color.r = 0.0f;
        arrow_marker.color.g = 0.0f;
        arrow_marker.color.b = 1.0f;

        marker_array_.markers.push_back (arrow_marker);
    }
}

void speed_path_visualizer::timer_callback () {
    marker_pub_->publish (marker_array_);
}

}  // namespace speed_path_visualizer
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (speed_path_visualizer::speed_path_visualizer)