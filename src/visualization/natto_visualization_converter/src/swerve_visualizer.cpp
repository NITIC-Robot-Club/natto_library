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

namespace steering_vector_visualizer {

namespace {

int find_index (const std::vector<std::string> &names, const std::string &name) {
    const auto it = std::find (names.begin (), names.end (), name);
    if (it == names.end ()) {
        return -1;
    }
    return static_cast<int> (std::distance (names.begin (), it));
}

double normalize_angle (double angle) {
    return std::atan2 (std::sin (angle), std::cos (angle));
}

std::vector<geometry_msgs::msg::Point> build_arc_points (const geometry_msgs::msg::Point &center, double start_angle, double delta, double radius) {
    const int                              segments = std::max (8, static_cast<int> (std::ceil (std::abs (delta) / (M_PI / 18.0))));
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve (static_cast<size_t> (segments) + 1);

    for (int i = 0; i <= segments; ++i) {
        const double t     = static_cast<double> (i) / static_cast<double> (segments);
        const double angle = start_angle + delta * t;

        geometry_msgs::msg::Point point;
        point.x = center.x + std::cos (angle) * radius;
        point.y = center.y + std::sin (angle) * radius;
        point.z = center.z;
        points.push_back (point);
    }

    return points;
}

}  // namespace

steering_vector_visualizer::steering_vector_visualizer (const rclcpp::NodeOptions &options) : Node ("steering_vector_visualizer", options) {
    marker_pub_      = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::SensorDataQoS (), std::bind (&steering_vector_visualizer::joint_state_callback, this, std::placeholders::_1));

    double frequency            = this->declare_parameter<double> ("frequency", 100.0);
    chassis_type_               = this->declare_parameter<std::string> ("chassis_type", "");
    wheel_radius_               = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_names_                = this->declare_parameter<std::vector<std::string>> ("wheel_names", {""});
    wheel_base_names_           = this->declare_parameter<std::vector<std::string>> ("wheel_base_names", {""});
    infinite_swerve_mode_       = this->declare_parameter<bool> ("infinite_swerve_mode", false);
    frame_id_                   = this->declare_parameter<std::string> ("frame_id", "command/base_link");
    line_width_                 = this->declare_parameter<double> ("line_width", 0.05);
    vector_scale_               = this->declare_parameter<double> ("vector_scale", 0.25);
    rotation_vector_scale_      = this->declare_parameter<double> ("rotation_vector_scale", 0.12);
    rotation_vector_line_width_ = this->declare_parameter<double> ("rotation_vector_line_width", 0.03);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    if (wheel_names_.size () != wheel_base_names_.size ()) {
        throw std::runtime_error ("Invalid parameters: wheel_names and wheel_base_names size mismatch.");
    }

    steering_speeds_.assign (wheel_names_.size (), 0.0);
    steering_speed_average_.assign (wheel_names_.size (), 0.0);
    steering_speed_history_.assign (wheel_names_.size (), std::vector<double> (5, 0.0));
    steering_speed_history_index_.assign (wheel_names_.size (), 0);
    steering_speed_history_count_.assign (wheel_names_.size (), 0);
    steering_speed_directions_.assign (wheel_names_.size (), 0);

    RCLCPP_INFO (this->get_logger (), "steering_vector_visualizer node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency);
    RCLCPP_INFO (this->get_logger (), "chassis_type: %s", chassis_type_.c_str ());
    if (chassis_type_ != "swerve") {
        RCLCPP_WARN (this->get_logger (), "steering_vector_visualizer supports only swerve chassis_type. No markers will be published.");
    }
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.3f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "frame_id: %s", frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "line_width: %.3f", line_width_);
    RCLCPP_INFO (this->get_logger (), "vector_scale: %.3f", vector_scale_);
    RCLCPP_INFO (this->get_logger (), "rotation_vector_scale: %.3f", rotation_vector_scale_);
    RCLCPP_INFO (this->get_logger (), "rotation_vector_line_width: %.3f", rotation_vector_line_width_);
    RCLCPP_INFO (this->get_logger (), "infinite_swerve_mode: %s", infinite_swerve_mode_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "num_wheels: %zu", wheel_names_.size ());

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&steering_vector_visualizer::timer_callback, this));
}

void steering_vector_visualizer::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    // command_joint_states の連続差分を 5 サンプル平均して、低速域の符号反転を抑える。
    if (has_previous_joint_state_) {
        const rclcpp::Time current_stamp (msg->header.stamp);
        const rclcpp::Time previous_stamp (previous_joint_state_.header.stamp);
        const double       dt = (current_stamp - previous_stamp).seconds ();

        if (dt > 1e-6) {
            constexpr double direction_threshold = 0.25;
            constexpr size_t history_length      = 5;
            for (size_t i = 0; i < wheel_base_names_.size (); ++i) {
                const int current_idx  = find_index (msg->name, wheel_base_names_[i]);
                const int previous_idx = find_index (previous_joint_state_.name, wheel_base_names_[i]);

                if (current_idx < 0 || previous_idx < 0 || static_cast<size_t> (current_idx) >= msg->position.size () || static_cast<size_t> (previous_idx) >= previous_joint_state_.position.size ()) {
                    continue;
                }

                const double current_angle  = msg->position[static_cast<size_t> (current_idx)];
                const double previous_angle = previous_joint_state_.position[static_cast<size_t> (previous_idx)];
                const double raw_speed      = normalize_angle (current_angle - previous_angle) / dt;

                steering_speeds_[i] = raw_speed;

                auto        &history             = steering_speed_history_[i];
                const size_t slot                = steering_speed_history_index_[i];
                history[slot]                    = raw_speed;
                steering_speed_history_index_[i] = (slot + 1) % history_length;
                steering_speed_history_count_[i] = std::min (steering_speed_history_count_[i] + 1, history_length);

                double sum = 0.0;
                for (size_t j = 0; j < history_length; ++j) {
                    sum += history[j];
                }
                const double average_speed = sum / static_cast<double> (history_length);
                steering_speed_average_[i] = average_speed;

                if (std::abs (average_speed) >= direction_threshold) {
                    steering_speed_directions_[i] = average_speed >= 0.0 ? 1 : -1;
                } else {
                    steering_speed_directions_[i] = 0;
                    steering_speed_average_[i]    = 0.0;
                }
            }
        }
    }

    previous_joint_state_     = *msg;
    has_previous_joint_state_ = true;
    joint_state_              = *msg;
}

void steering_vector_visualizer::timer_callback () {
    marker_array_.markers.clear ();

    if (chassis_type_ != "swerve") {
        marker_pub_->publish (marker_array_);
        return;
    }

    if (joint_state_.name.empty ()) {
        marker_pub_->publish (marker_array_);
        return;
    }

    for (size_t i = 0; i < wheel_names_.size (); ++i) {
        const int wheel_idx      = find_index (joint_state_.name, wheel_names_[i]);
        const int wheel_base_idx = find_index (joint_state_.name, wheel_base_names_[i]);

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

        geometry_msgs::msg::Point start;
        start.x = tf_stamped.transform.translation.x;
        start.y = tf_stamped.transform.translation.y;
        start.z = tf_stamped.transform.translation.z;

        if (std::abs (length) >= 1e-6) {
            geometry_msgs::msg::Point end;
            end.x = start.x + std::cos (steering_yaw) * length;
            end.y = start.y + std::sin (steering_yaw) * length;
            end.z = start.z;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp    = this->now ();
            marker.ns              = "steering_vector";
            marker.id              = static_cast<int> (i);
            marker.type            = visualization_msgs::msg::Marker::ARROW;
            marker.action          = visualization_msgs::msg::Marker::ADD;
            marker.lifetime        = rclcpp::Duration::from_seconds (0.5);
            marker.scale.x         = line_width_;
            marker.scale.y         = line_width_ * 2.0;
            marker.scale.z         = line_width_ * 2.5;
            marker.color.r         = 0.1f;
            marker.color.g         = 0.8f;
            marker.color.b         = 0.2f;
            marker.color.a         = 1.0f;
            marker.points          = {start, end};

            marker_array_.markers.push_back (marker);
        }

        const bool has_target_steering = wheel_base_idx >= 0 && static_cast<size_t> (wheel_base_idx) < joint_state_.position.size ();
        if (!has_target_steering) {
            RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 3000, "Could not find steering joint: %s", wheel_base_names_[i].c_str ());
            continue;
        }

        const int direction = steering_speed_directions_[i];
        if (direction == 0) {
            continue;
        }

        const double steering_speed   = steering_speed_average_[i];
        const double speed_magnitude  = std::max (std::abs (steering_speed), 0.35);
        const double arc_radius       = std::max (rotation_vector_scale_ * 1.8, rotation_vector_line_width_ * 5.0);
        const double arc_delta        = std::clamp (speed_magnitude, 0.35, M_PI * 0.9);
        const double signed_arc_delta = static_cast<double> (direction) * arc_delta;
        const auto   arc_points       = build_arc_points (start, 0.0, signed_arc_delta, arc_radius);
        if (arc_points.size () < 2) {
            continue;
        }

        visualization_msgs::msg::Marker rotation_marker;
        rotation_marker.header.frame_id = frame_id_;
        rotation_marker.header.stamp    = this->now ();
        rotation_marker.ns              = "steering_speed_arc";
        rotation_marker.id              = static_cast<int> (i);
        rotation_marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        rotation_marker.action          = visualization_msgs::msg::Marker::ADD;
        rotation_marker.lifetime        = rclcpp::Duration::from_seconds (0.5);
        rotation_marker.scale.x         = rotation_vector_line_width_;
        rotation_marker.color.r         = 0.95f;
        rotation_marker.color.g         = 0.55f;
        rotation_marker.color.b         = 0.15f;
        rotation_marker.color.a         = 1.0f;
        rotation_marker.points          = arc_points;

        marker_array_.markers.push_back (rotation_marker);

        const geometry_msgs::msg::Point &arc_end       = arc_points.back ();
        const double                     arc_end_angle = signed_arc_delta;
        const double                     tangent_yaw   = arc_end_angle + (direction >= 0 ? M_PI / 2.0 : -M_PI / 2.0);
        const double                     arrow_length  = std::max (rotation_vector_line_width_ * 8.0, arc_radius * 0.85);

        geometry_msgs::msg::Point arrow_start;
        arrow_start.x = arc_end.x - std::cos (tangent_yaw) * arrow_length;
        arrow_start.y = arc_end.y - std::sin (tangent_yaw) * arrow_length;
        arrow_start.z = arc_end.z;

        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = frame_id_;
        arrow_marker.header.stamp    = this->now ();
        arrow_marker.ns              = "steering_speed_arc_head";
        arrow_marker.id              = static_cast<int> (i + wheel_names_.size ());
        arrow_marker.type            = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action          = visualization_msgs::msg::Marker::ADD;
        arrow_marker.lifetime        = rclcpp::Duration::from_seconds (0.5);
        arrow_marker.scale.x         = rotation_vector_line_width_;
        arrow_marker.scale.y         = rotation_vector_line_width_ * 3.0;
        arrow_marker.scale.z         = rotation_vector_line_width_ * 3.8;
        arrow_marker.color.r         = 0.95f;
        arrow_marker.color.g         = 0.55f;
        arrow_marker.color.b         = 0.15f;
        arrow_marker.color.a         = 1.0f;
        arrow_marker.points          = {arrow_start, arc_end};

        marker_array_.markers.push_back (arrow_marker);
    }

    marker_pub_->publish (marker_array_);
}

}  // namespace steering_vector_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (steering_vector_visualizer::steering_vector_visualizer)
