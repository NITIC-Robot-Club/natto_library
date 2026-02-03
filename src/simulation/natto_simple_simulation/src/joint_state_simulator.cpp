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

#include "natto_simple_simulation/joint_state_simulator.hpp"

namespace joint_state_simulator {

joint_state_simulator::joint_state_simulator (const rclcpp::NodeOptions &node_options) : Node ("joint_state_simulator", node_options) {
    joint_state_publisher_          = this->create_publisher<sensor_msgs::msg::JointState> ("joint_states", rclcpp::SensorDataQoS ());
    simulation_pose_publisher_      = this->create_publisher<geometry_msgs::msg::PoseStamped> ("simulation_pose", 10);
    command_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::SensorDataQoS (), std::bind (&joint_state_simulator::command_joint_state_callback, this, std::placeholders::_1));
    tf_broadcaster_                 = std::make_shared<tf2_ros::TransformBroadcaster> (this);

    chassis_type_ = this->declare_parameter<std::string> ("chassis_type", "");
    wheel_names_  = this->declare_parameter<std::vector<std::string>> ("wheel_names", {""});
    wheel_radius_ = this->declare_parameter<double> ("wheel_radius", 0.05);
    frequency_    = this->declare_parameter<double> ("frequency", 1000.0);

    initial_pose_x_   = this->declare_parameter<double> ("initial_pose_x", 0.0);
    initial_pose_y_   = this->declare_parameter<double> ("initial_pose_y", 0.0);
    initial_pose_yaw_ = this->declare_parameter<double> ("initial_pose_yaw_deg", 0.0) * M_PI / 180.0;

    current_pose_.header.frame_id = "map";
    current_pose_.pose.position.x = initial_pose_x_;
    current_pose_.pose.position.y = initial_pose_y_;

    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, initial_pose_yaw_);
    current_pose_.pose.orientation = tf2::toMsg (q);

    num_wheels_ = wheel_names_.size ();
    if (chassis_type_ == "") {
        RCLCPP_ERROR (this->get_logger (), "chassis_type parameter is required.");
        throw std::runtime_error ("chassis_type parameter is required.");
    }

    wheel_base_names_ = this->declare_parameter<std::vector<std::string>> ("wheel_base_names", {""});
    if (wheel_base_names_.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_base_names size must be equal to number of wheels.");
        throw std::runtime_error ("wheel_base_names size must be equal to number of wheels.");
    }

    if (chassis_type_ == "swerve") {
        infinite_swerve_mode_ = this->declare_parameter<bool> ("infinite_swerve_mode", false);
    } else if (chassis_type_ == "omni" || chassis_type_ == "mecanum") {
        // No additional parameters needed for omni or mecanum wheels
    } else {
        RCLCPP_ERROR (this->get_logger (), "Unsupported chassis_type: %s", chassis_type_.c_str ());
        throw std::runtime_error ("Unsupported chassis_type: " + chassis_type_);
    }

    joint_names_ = this->declare_parameter<std::vector<std::string>> ("joint_names", {""});
    num_joints_  = joint_names_.size ();

    control_modes_ = this->declare_parameter<std::vector<std::string>> ("control_modes", {""});
    if (control_modes_.size () != num_joints_) {
        RCLCPP_ERROR (this->get_logger (), "control_modes size must be equal to number of joints.");
        throw std::runtime_error ("control_modes size must be equal to number of joints.");
    }
    for (size_t i = 0; i < num_joints_; i++) {
        if (control_modes_[i] != "position" && control_modes_[i] != "speed") {
            RCLCPP_ERROR (this->get_logger (), "Unsupported control_mode: %s", control_modes_[i].c_str ());
            throw std::runtime_error ("Unsupported control_mode: " + control_modes_[i]);
        }
    }

    initial_positions_ = this->declare_parameter<std::vector<double>> ("initial_positions", {0.0});
    if (initial_positions_.size () != joint_names_.size ()) {
        RCLCPP_ERROR (this->get_logger (), "initial_positions size must be equal to number of joints.");
        throw std::runtime_error ("initial_positions size must be equal to number of joints.");
    }

    joint_position_tau_ = this->declare_parameter<std::vector<double>> ("joint_position_tau", {0.5});
    if (joint_position_tau_.size () != joint_names_.size ()) {
        RCLCPP_ERROR (this->get_logger (), "joint_position_tau size must be equal to number of joints.");
        throw std::runtime_error ("joint_position_tau size must be equal to number of joints.");
    }

    joint_velocity_tau_ = this->declare_parameter<std::vector<double>> ("joint_velocity_tau", {0.1});
    if (joint_velocity_tau_.size () != joint_names_.size ()) {
        RCLCPP_ERROR (this->get_logger (), "joint_velocity_tau size must be equal to number of joints.");
        throw std::runtime_error ("joint_velocity_tau size must be equal to number of joints.");
    }

    joint_velocity_max_ = this->declare_parameter<std::vector<double>> ("joint_velocity_max", {10.0});
    if (joint_velocity_max_.size () != joint_names_.size ()) {
        RCLCPP_ERROR (this->get_logger (), "joint_velocity_max size must be equal to number of joints.");
        throw std::runtime_error ("joint_velocity_max size must be equal to number of joints.");
    }

    current_.name     = joint_names_;
    current_.position = initial_positions_;
    current_.velocity.resize (joint_names_.size (), 0.0);
    current_.effort.resize (joint_names_.size (), 0.0);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
    timer_       = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency_), std::bind (&joint_state_simulator::timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "joint_state_simulator node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "chassis_type: %s", chassis_type_.c_str ());
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency_);
    RCLCPP_INFO (this->get_logger (), "initial pose: (%f, %f, %f)", initial_pose_x_, initial_pose_y_, initial_pose_yaw_);
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "num_wheels: %zu", num_wheels_);
    for (size_t i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "wheel_name[%zu]: %s", i, wheel_names_[i].c_str ());
    }
    RCLCPP_INFO (this->get_logger (), "num_joints: %zu", num_joints_);
    for (size_t i = 0; i < num_joints_; i++) {
        RCLCPP_INFO (this->get_logger (), "joint_name[%zu]: %s control_mode[%zu]: %s", i, joint_names_[i].c_str (), i, control_modes_[i].c_str ());
    }
}

void joint_state_simulator::command_joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    command_ = *msg;
}

void joint_state_simulator::timer_callback () {
    const double dt = 1.0 / frequency_;

    for (size_t i = 0; i < num_joints_; i++) {
        auto it = std::find (command_.name.begin (), command_.name.end (), joint_names_[i]);

        if (it == command_.name.end ()) {
            continue;
        }

        size_t index = static_cast<size_t> (std::distance (command_.name.begin (), it));

        if (control_modes_[i] == "position") {
            const double target_position  = command_.position[index];
            const double error            = target_position - current_.position[i];
            double       command_velocity = error / joint_position_tau_[i];

            command_velocity     = std::clamp (command_velocity, -joint_velocity_max_[i], joint_velocity_max_[i]);
            current_.velocity[i] = command_velocity;
            current_.position[i] += command_velocity * dt;
        } else if (control_modes_[i] == "speed") {
            const double target_velocity = command_.velocity[index];
            const double acceleration    = (target_velocity - current_.velocity[i]) / joint_velocity_tau_[i];

            current_.velocity[i] += acceleration / frequency_;
            current_.velocity[i] = std::clamp (current_.velocity[i], -joint_velocity_max_[i], joint_velocity_max_[i]);
            current_.position[i] += current_.velocity[i] / frequency_;
        }
    }

    current_.header.stamp = this->now ();
    joint_state_publisher_->publish (current_);

    double vx   = 0.0;
    double vy   = 0.0;
    double vyaw = 0.0;
    if (chassis_type_ == "swerve") {
        double ATA[3][3] = {};  // A^T * A
        double ATb[3]    = {};  // A^T * b

        for (size_t i = 0; i < num_wheels_; i++) {
            double angle            = 0.0;
            double speed            = 0.0;
            bool   found_wheel      = false;
            bool   found_wheel_base = false;
            for (size_t j = 0; j < current_.name.size (); j++) {
                if (current_.name[j] == wheel_names_[i]) {
                    speed       = current_.velocity[j] * wheel_radius_;
                    found_wheel = true;
                }
                if (current_.name[j] == wheel_base_names_[i]) {
                    angle            = current_.position[j];
                    found_wheel_base = true;
                }
            }

            if (!found_wheel) {
                RCLCPP_WARN (this->get_logger (), "Could not find wheel joint: %s", wheel_names_[i].c_str ());
                return;
            }
            if (!found_wheel_base) {
                RCLCPP_WARN (this->get_logger (), "Could not find wheel_base joint: %s", wheel_base_names_[i].c_str ());
                return;
            }

            double wheel_position_x = 0.0;
            double wheel_position_y = 0.0;

            try {
                geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform ("command/base_link", "command/" + wheel_base_names_[i] + "_link", tf2::TimePointZero);
                wheel_position_x                                = tf_stamped.transform.translation.x;
                wheel_position_y                                = tf_stamped.transform.translation.y;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 3000, "Could not get transform from %s to base_link: %s", wheel_base_names_[i].c_str (), ex.what ());
                return;
            }

            double ax[3] = {1.0, 0.0, -wheel_position_y};
            double ay[3] = {0.0, 1.0, +wheel_position_x};

            double bx = speed * std::cos (angle);
            double by = speed * std::sin (angle);

            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    ATA[row][col] += ax[row] * ax[col] + ay[row] * ay[col];
                }
                ATb[row] += ax[row] * bx + ay[row] * by;
            }
        }
        double A[3][4] = {
            {ATA[0][0], ATA[0][1], ATA[0][2], ATb[0]},
            {ATA[1][0], ATA[1][1], ATA[1][2], ATb[1]},
            {ATA[2][0], ATA[2][1], ATA[2][2], ATb[2]}
        };

        for (int i = 0; i < 3; ++i) {
            double pivot = A[i][i];
            for (int j = i; j < 4; ++j) {
                A[i][j] /= pivot;
            }
            for (int k = 0; k < 3; ++k) {
                if (k == i) continue;
                double factor = A[k][i];
                for (int j = i; j < 4; ++j) {
                    A[k][j] -= factor * A[i][j];
                }
            }
        }
        vx   = A[0][3];
        vy   = A[1][3];
        vyaw = A[2][3];
    } else if (chassis_type_ == "omni") {
        double ATA[3][3] = {};  // A^T * A
        double ATb[3]    = {};  // A^T * b

        for (size_t i = 0; i < num_wheels_; i++) {
            double speed = 0.0;
            bool   found = false;
            for (size_t j = 0; j < current_.name.size (); j++)
                if (current_.name[j] == wheel_names_[i]) {
                    speed = current_.velocity[j] * wheel_radius_;
                    found = true;
                }
            if (!found) {
                RCLCPP_WARN (this->get_logger (), "Could not find wheel joint: %s", wheel_names_[i].c_str ());
                return;
            }

            double wheel_position_x = 0.0;
            double wheel_position_y = 0.0;
            double wheel_angle      = 0.0;

            try {
                geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform ("command/base_link", "command/" + wheel_base_names_[i] + "_link", tf2::TimePointZero);
                wheel_position_x                                = tf_stamped.transform.translation.x;
                wheel_position_y                                = tf_stamped.transform.translation.y;
                wheel_angle                                     = tf2::getYaw (tf_stamped.transform.rotation);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN (this->get_logger (), "Could not get transform from %s to base_link: %s", wheel_base_names_[i].c_str (), ex.what ());
                return;
            }

            double cos   = std::cos (wheel_angle);
            double sin   = std::sin (wheel_angle);
            double ax[3] = {cos, sin, (-wheel_position_y * cos + wheel_position_x * sin)};
            double b     = speed;

            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    ATA[row][col] += ax[row] * ax[col];
                }
                ATb[row] += ax[row] * b;
            }
        }
        double A[3][4] = {
            {ATA[0][0], ATA[0][1], ATA[0][2], ATb[0]},
            {ATA[1][0], ATA[1][1], ATA[1][2], ATb[1]},
            {ATA[2][0], ATA[2][1], ATA[2][2], ATb[2]}
        };

        for (int i = 0; i < 3; ++i) {
            double pivot = A[i][i];
            if (std::fabs (pivot) < 1e-9) return;
            for (int j = i; j < 4; ++j) A[i][j] /= pivot;
            for (int k = 0; k < 3; ++k) {
                if (k == i) continue;
                double factor = A[k][i];
                for (int j = i; j < 4; ++j) A[k][j] -= factor * A[i][j];
            }
        }
        vx   = A[0][3];
        vy   = A[1][3];
        vyaw = A[2][3];
    }
    double yaw      = tf2::getYaw (current_pose_.pose.orientation);
    double vx_world = vx * cos (yaw) - vy * sin (yaw);
    double vy_world = vx * sin (yaw) + vy * cos (yaw);
    current_pose_.pose.position.x += vx_world / frequency_;
    current_pose_.pose.position.y += vy_world / frequency_;
    yaw += vyaw / frequency_;

    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, yaw);
    current_pose_.pose.orientation.x = q.x ();
    current_pose_.pose.orientation.y = q.y ();
    current_pose_.pose.orientation.z = q.z ();
    current_pose_.pose.orientation.w = q.w ();
    current_pose_.header.stamp       = this->now ();
    current_pose_.header.frame_id    = "map";
    simulation_pose_publisher_->publish (current_pose_);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = this->now ();
    tf_msg.header.frame_id         = "map";
    tf_msg.child_frame_id          = "simulation";
    tf_msg.transform.translation.x = current_pose_.pose.position.x;
    tf_msg.transform.translation.y = current_pose_.pose.position.y;
    tf_msg.transform.translation.z = current_pose_.pose.position.z;
    tf_msg.transform.rotation      = current_pose_.pose.orientation;

    tf_broadcaster_->sendTransform (tf_msg);
}

}  // namespace joint_state_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (joint_state_simulator::joint_state_simulator)