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

#include "natto_visualization_converter/swerve_visualizer.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace swerve_visualizer {

namespace {

int find_index (const std::vector<std::string> &names, const std::string &name) {
    const auto it = std::find (names.begin (), names.end (), name);
    if (it == names.end ()) {
        return -1;
    }
    return static_cast<int> (std::distance (names.begin (), it));
}

}  // namespace

swerve_visualizer::swerve_visualizer (const rclcpp::NodeOptions &options) : Node ("swerve_visualizer", options) {
    marker_pub_      = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::SensorDataQoS (), std::bind (&swerve_visualizer::joint_state_callback, this, std::placeholders::_1));

    double frequency     = this->declare_parameter<double> ("frequency", 100.0);
    chassis_type_        = this->declare_parameter<std::string> ("chassis_type", "");
    wheel_radius_        = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_names_         = this->declare_parameter<std::vector<std::string>> ("wheel_names", {""});
    wheel_base_names_    = this->declare_parameter<std::vector<std::string>> ("wheel_base_names", {""});
    infinite_swerve_mode_ = this->declare_parameter<bool> ("infinite_swerve_mode", false);
    frame_id_            = this->declare_parameter<std::string> ("frame_id", "command/base_link");
    line_width_          = this->declare_parameter<double> ("line_width", 0.05);
    vector_scale_        = this->declare_parameter<double> ("vector_scale", 0.25);
    arrow_color_r_       = this->declare_parameter<double> ("arrow_color_r", 0.1);
    arrow_color_g_       = this->declare_parameter<double> ("arrow_color_g", 0.8);
    arrow_color_b_       = this->declare_parameter<double> ("arrow_color_b", 0.2);
    arrow_color_a_       = this->declare_parameter<double> ("arrow_color_a", 1.0);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    if (wheel_names_.size () != wheel_base_names_.size ()) {
        throw std::runtime_error ("Invalid parameters: wheel_names and wheel_base_names size mismatch.");
    }

    RCLCPP_INFO (this->get_logger (), "swerve_visualizer node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency);
    RCLCPP_INFO (this->get_logger (), "chassis_type: %s", chassis_type_.c_str ());
    if (chassis_type_ != "swerve") {
        RCLCPP_WARN (this->get_logger (), "swerve_visualizer supports only swerve chassis_type. No markers will be published.");
    }
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.3f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "frame_id: %s", frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "line_width: %.3f", line_width_);
    RCLCPP_INFO (this->get_logger (), "vector_scale: %.3f", vector_scale_);
    RCLCPP_INFO (this->get_logger (), "arrow_color_rgba: %.3f %.3f %.3f %.3f", arrow_color_r_, arrow_color_g_, arrow_color_b_, arrow_color_a_);
    RCLCPP_INFO (this->get_logger (), "infinite_swerve_mode: %s", infinite_swerve_mode_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "num_wheels: %zu", wheel_names_.size ());

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&swerve_visualizer::timer_callback, this));
}

void swerve_visualizer::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_state_ = *msg;
}

void swerve_visualizer::timer_callback () {
    marker_array_.markers.clear ();

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp    = this->now ();
    clear_marker.action          = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_.markers.push_back (clear_marker);

    if (chassis_type_ != "swerve") {
        marker_pub_->publish (marker_array_);
        return;
    }

    if (joint_state_.name.empty ()) {
        marker_pub_->publish (marker_array_);
        return;
    }

    for (size_t i = 0; i < wheel_names_.size (); ++i) {
        const int wheel_idx = find_index (joint_state_.name, wheel_names_[i]);

        if (wheel_idx < 0 || static_cast<size_t> (wheel_idx) >= joint_state_.velocity.size ()) {
            RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 3000, "Could not find wheel joint: %s", wheel_names_[i].c_str ());
            continue;
        }

        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_->lookupTransform (frame_id_, "command/" + wheel_base_names_[i] + "_link", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 3000, "Could not get transform for %s: %s", wheel_base_names_[i].c_str (), ex.what ());
            continue;
        }

        const double steering_yaw = tf2::getYaw (tf_stamped.transform.rotation);
        const double wheel_speed  = joint_state_.velocity[static_cast<size_t> (wheel_idx)] * wheel_radius_;
        const double length       = wheel_speed * vector_scale_;

        if (std::abs (length) < 1e-6) {
            continue;
        }

        geometry_msgs::msg::Point start;
        start.x = tf_stamped.transform.translation.x;
        start.y = tf_stamped.transform.translation.y;
        start.z = tf_stamped.transform.translation.z;

        geometry_msgs::msg::Point end;
        end.x = start.x + std::cos (steering_yaw) * length;
        end.y = start.y + std::sin (steering_yaw) * length;
        end.z = start.z;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp    = this->now ();
        marker.ns              = "swerve_visualizer";
        marker.id              = static_cast<int> (i);
        marker.type            = visualization_msgs::msg::Marker::ARROW;
        marker.action          = visualization_msgs::msg::Marker::ADD;
        marker.lifetime        = rclcpp::Duration::from_seconds (0.5);
        marker.scale.x         = line_width_;
        marker.scale.y         = line_width_ * 2.0;
        marker.scale.z         = line_width_ * 2.5;
        marker.color.r         = static_cast<float> (arrow_color_r_);
        marker.color.g         = static_cast<float> (arrow_color_g_);
        marker.color.b         = static_cast<float> (arrow_color_b_);
        marker.color.a         = static_cast<float> (arrow_color_a_);
        marker.points          = {start, end};

        marker_array_.markers.push_back (marker);
    }

    marker_pub_->publish (marker_array_);
}

}  // namespace swerve_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (swerve_visualizer::swerve_visualizer)
