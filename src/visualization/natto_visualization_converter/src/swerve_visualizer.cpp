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

double normalize_angle (double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

geometry_msgs::msg::Point make_point (double x, double y, double z) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

}  // namespace

swerve_visualizer::swerve_visualizer (const rclcpp::NodeOptions &options) : Node ("swerve_visualizer", options) {
    marker_pub_      = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState> ("joint_states", rclcpp::SensorDataQoS (), std::bind (&swerve_visualizer::joint_state_callback, this, std::placeholders::_1));

    double frequency      = this->declare_parameter<double> ("frequency", 100.0);
    chassis_type_         = this->declare_parameter<std::string> ("chassis_type", "");
    wheel_radius_         = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_names_          = this->declare_parameter<std::vector<std::string>> ("wheel_names", {""});
    wheel_base_names_     = this->declare_parameter<std::vector<std::string>> ("wheel_base_names", {""});
    infinite_swerve_mode_ = this->declare_parameter<bool> ("infinite_swerve_mode", false);
    frame_id_             = this->declare_parameter<std::string> ("frame_id", "command/base_link");
    line_width_           = this->declare_parameter<double> ("line_width", 0.05);
    vector_scale_         = this->declare_parameter<double> ("vector_scale", 0.25);
    arrow_color_r_        = this->declare_parameter<double> ("arrow_color_r", 0.1);
    arrow_color_g_        = this->declare_parameter<double> ("arrow_color_g", 0.8);
    arrow_color_b_        = this->declare_parameter<double> ("arrow_color_b", 0.2);
    arrow_color_a_        = this->declare_parameter<double> ("arrow_color_a", 1.0);
    show_steering_arc_    = this->declare_parameter<bool> ("show_steering_arc", false);
    rotation_vector_line_width_ = this->declare_parameter<double> ("rotation_vector_line_width", line_width_ * 0.9);
    rotation_vector_scale_      = this->declare_parameter<double> ("rotation_vector_scale", std::max (line_width_ * 4.5, 0.36));
    steering_arc_span_      = this->declare_parameter<double> ("steering_arc_span", 0.35);
    steering_arc_max_span_  = this->declare_parameter<double> ("steering_arc_max_span", 1.2);
    steering_arrow_length_          = this->declare_parameter<double> ("steering_arrow_length", 0.09);
    steering_arrow_head_scale_      = this->declare_parameter<double> ("steering_arrow_head_scale", 1.05);
    steering_arrow_head_cone_scale_ = this->declare_parameter<double> ("steering_arrow_head_cone_scale", 1.05);
    steering_color_r_       = this->declare_parameter<double> ("steering_color_r", 1.0);
    steering_color_g_       = this->declare_parameter<double> ("steering_color_g", 0.55);
    steering_color_b_       = this->declare_parameter<double> ("steering_color_b", 0.0);
    steering_color_a_       = this->declare_parameter<double> ("steering_color_a", 1.0);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    if (wheel_names_.size () != wheel_base_names_.size ()) {
        throw std::runtime_error ("Invalid parameters: wheel_names and wheel_base_names size mismatch.");
    }

    previous_steering_positions_.assign (wheel_names_.size (), 0.0);
    cached_steering_rates_.assign (wheel_names_.size (), 0.0);
    previous_steering_positions_valid_.assign (wheel_names_.size (), false);
    has_last_joint_state_stamp_ = false;

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
    RCLCPP_INFO (this->get_logger (), "show_steering_arc: %s", show_steering_arc_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "rotation_vector_line_width: %.3f", rotation_vector_line_width_);
    RCLCPP_INFO (this->get_logger (), "rotation_vector_scale: %.3f", rotation_vector_scale_);
    RCLCPP_INFO (this->get_logger (), "steering_arc_span: %.3f", steering_arc_span_);
    RCLCPP_INFO (this->get_logger (), "steering_arc_max_span: %.3f", steering_arc_max_span_);
    RCLCPP_INFO (this->get_logger (), "steering_arrow_length: %.3f", steering_arrow_length_);
    RCLCPP_INFO (this->get_logger (), "steering_arrow_head_scale: %.3f", steering_arrow_head_scale_);
    RCLCPP_INFO (this->get_logger (), "steering_arrow_head_cone_scale: %.3f", steering_arrow_head_cone_scale_);
    RCLCPP_INFO (this->get_logger (), "steering_color_rgba: %.3f %.3f %.3f %.3f", steering_color_r_, steering_color_g_, steering_color_b_, steering_color_a_);
    RCLCPP_INFO (this->get_logger (), "infinite_swerve_mode: %s", infinite_swerve_mode_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "num_wheels: %zu", wheel_names_.size ());

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&swerve_visualizer::timer_callback, this));
}

void swerve_visualizer::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_state_ = *msg;
}

void swerve_visualizer::update_steering_rates () {
    if (joint_state_.name.empty ()) {
        return;
    }

    const rclcpp::Time current_stamp = joint_state_.header.stamp;
    if (!has_last_joint_state_stamp_) {
        last_joint_state_stamp_     = current_stamp;
        has_last_joint_state_stamp_ = true;
        for (size_t i = 0; i < wheel_names_.size (); ++i) {
            const int wheel_base_idx = find_index (joint_state_.name, wheel_base_names_[i]);
            if (wheel_base_idx < 0 || static_cast<size_t> (wheel_base_idx) >= joint_state_.position.size ()) {
                continue;
            }
            previous_steering_positions_[i]       = joint_state_.position[static_cast<size_t> (wheel_base_idx)];
            previous_steering_positions_valid_[i] = true;
            cached_steering_rates_[i]             = 0.0;
        }
        return;
    }

    const double dt = (current_stamp - last_joint_state_stamp_).seconds ();
    if (!(dt > 0.0)) {
        return;
    }

    for (size_t i = 0; i < wheel_names_.size (); ++i) {
        const int wheel_base_idx = find_index (joint_state_.name, wheel_base_names_[i]);
        if (wheel_base_idx < 0 || static_cast<size_t> (wheel_base_idx) >= joint_state_.position.size ()) {
            continue;
        }

        const double current_position = joint_state_.position[static_cast<size_t> (wheel_base_idx)];
        if (!previous_steering_positions_valid_[i]) {
            previous_steering_positions_[i]       = current_position;
            previous_steering_positions_valid_[i] = true;
            cached_steering_rates_[i]             = 0.0;
            continue;
        }

        double delta = current_position - previous_steering_positions_[i];
        if (!infinite_swerve_mode_) {
            delta = normalize_angle (delta);
        }

        cached_steering_rates_[i]      = delta / dt;
        previous_steering_positions_[i] = current_position;
    }

    last_joint_state_stamp_ = current_stamp;
}

void swerve_visualizer::append_wheel_speed_marker (size_t wheel_index, const geometry_msgs::msg::TransformStamped &tf_stamped, double steering_yaw, double wheel_speed) {
    const double length = wheel_speed * vector_scale_;
    if (std::abs (length) < 1e-6) {
        return;
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
    marker.id              = static_cast<int> (wheel_index);
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

void swerve_visualizer::append_steering_arc_markers (size_t wheel_index, const geometry_msgs::msg::TransformStamped &tf_stamped, double steering_rate) {
    if (std::abs (steering_rate) < 1e-6) {
        return;
    }

    const double sign        = (steering_rate >= 0.0) ? 1.0 : -1.0;
    const double center_x    = tf_stamped.transform.translation.x;
    const double center_y    = tf_stamped.transform.translation.y;
    const double center_z    = tf_stamped.transform.translation.z;
    const double radius      = rotation_vector_scale_;
    const int    samples     = 24;
    const double span        = std::clamp (std::abs (steering_rate) * steering_arc_span_, 0.15, steering_arc_max_span_);
    const double start_angle = 0.0;
    const double end_angle   = sign * span;

    visualization_msgs::msg::Marker arc_marker;
    arc_marker.header.frame_id = frame_id_;
    arc_marker.header.stamp    = this->now ();
    arc_marker.ns              = "swerve_visualizer/steering_arc";
    arc_marker.id              = static_cast<int> (2000 + wheel_index);
    arc_marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    arc_marker.action          = visualization_msgs::msg::Marker::ADD;
    arc_marker.lifetime        = rclcpp::Duration::from_seconds (0.5);
    arc_marker.scale.x         = rotation_vector_line_width_;
    arc_marker.color.r         = static_cast<float> (steering_color_r_);
    arc_marker.color.g         = static_cast<float> (steering_color_g_);
    arc_marker.color.b         = static_cast<float> (steering_color_b_);
    arc_marker.color.a         = static_cast<float> (steering_color_a_);

    arc_marker.points.reserve (static_cast<size_t> (samples + 1));
    for (int i = 0; i <= samples; ++i) {
        const double t     = static_cast<double> (i) / static_cast<double> (samples);
        const double angle = start_angle + (end_angle - start_angle) * t;
        arc_marker.points.push_back (make_point (center_x + std::cos (angle) * radius, center_y + std::sin (angle) * radius, center_z));
    }
    marker_array_.markers.push_back (arc_marker);

    const double tangent_x = -sign * std::sin (end_angle);
    const double tangent_y = sign * std::cos (end_angle);
    const geometry_msgs::msg::Point arrow_start = arc_marker.points.back ();
    const geometry_msgs::msg::Point arrow_end   = make_point (arrow_start.x + tangent_x * steering_arrow_length_, arrow_start.y + tangent_y * steering_arrow_length_, arrow_start.z);

    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = frame_id_;
    arrow_marker.header.stamp    = this->now ();
    arrow_marker.ns              = "swerve_visualizer/steering_arc_head";
    arrow_marker.id              = static_cast<int> (3000 + wheel_index);
    arrow_marker.type            = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action          = visualization_msgs::msg::Marker::ADD;
    arrow_marker.lifetime        = rclcpp::Duration::from_seconds (0.5);
    arrow_marker.scale.x         = rotation_vector_line_width_;
    arrow_marker.scale.y         = rotation_vector_line_width_ * 2.0 * steering_arrow_head_scale_;
    arrow_marker.scale.z         = rotation_vector_line_width_ * 2.5 * steering_arrow_head_cone_scale_;
    arrow_marker.color.r         = static_cast<float> (steering_color_r_);
    arrow_marker.color.g         = static_cast<float> (steering_color_g_);
    arrow_marker.color.b         = static_cast<float> (steering_color_b_);
    arrow_marker.color.a         = static_cast<float> (steering_color_a_);
    arrow_marker.points          = {arrow_start, arrow_end};

    marker_array_.markers.push_back (arrow_marker);
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

    if (show_steering_arc_) {
        update_steering_rates ();
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

        append_wheel_speed_marker (i, tf_stamped, steering_yaw, wheel_speed);
        if (show_steering_arc_) {
            append_steering_arc_markers (i, tf_stamped, cached_steering_rates_[i]);
        }
    }

    marker_pub_->publish (marker_array_);
}

}  // namespace swerve_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (swerve_visualizer::swerve_visualizer)
