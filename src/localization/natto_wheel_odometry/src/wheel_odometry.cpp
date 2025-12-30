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

#include "natto_wheel_odometry/wheel_odometry.hpp"

namespace wheel_odometry {

wheel_odometry::wheel_odometry (const rclcpp::NodeOptions &node_options) : Node ("wheel_odometry", node_options) {
    twist_publisher_    = this->create_publisher<geometry_msgs::msg::TwistStamped> ("twist", 10);
    pose_publisher_     = this->create_publisher<geometry_msgs::msg::PoseStamped> ("pose", 10);
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry> ("odometry", rclcpp::SensorDataQoS ());

    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState> ("joint_states", rclcpp::QoS (10).best_effort (), std::bind (&wheel_odometry::joint_state_callback, this, std::placeholders::_1));
    tf_broadcaster_         = std::make_shared<tf2_ros::TransformBroadcaster> (this);

    wheel_radius_  = this->declare_parameter<double> ("wheel_radius", 0.05);
    odom_frame_id_ = this->declare_parameter<std::string> ("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter<std::string> ("base_frame_id", "base_link");
    publish_tf_    = this->declare_parameter<bool> ("publish_tf", false);

    wheel_names_ = this->declare_parameter<std::vector<std::string>> ("wheel_names", {""});
    steer_names_ = this->declare_parameter<std::vector<std::string>> ("steer_names", {""});
    num_wheels_  = wheel_names_.size ();
    if (steer_names_.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_names and steer_names must have the same size.");
        throw std::runtime_error ("Invalid parameters: wheel_names and steer_names size mismatch.");
    }

    RCLCPP_INFO (this->get_logger (), "wheel_odometry node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %zu", num_wheels_);
    RCLCPP_INFO (this->get_logger (), "publish_tf : %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "odom_frame_id : %s", odom_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "base_frame_id : %s", base_frame_id_.c_str ());

    last_pose_.header.frame_id = odom_frame_id_;
    last_pose_.header.stamp    = this->now ();

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
}

void wheel_odometry::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    double ATA[3][3] = {};  // A^T * A
    double ATb[3]    = {};  // A^T * b

    for (size_t i = 0; i < num_wheels_; i++) {
        double angle       = 0.0;
        double speed       = 0.0;
        bool   found_wheel = false;
        bool   found_steer = false;
        for (size_t j = 0; j < msg->name.size (); j++) {
            if (msg->name[j] == wheel_names_[i]) {
                speed       = msg->velocity[j] * wheel_radius_;
                found_wheel = true;
            }
            if (msg->name[j] == steer_names_[i]) {
                angle       = msg->position[j];
                found_steer = true;
            }
        }
        if (!found_wheel) {
            RCLCPP_WARN (this->get_logger (), "Could not find wheel joint: %s", wheel_names_[i].c_str ());
            return;
        }
        if (!found_steer) {
            RCLCPP_WARN (this->get_logger (), "Could not find steer joint: %s", steer_names_[i].c_str ());
            return;
        }

        double wheel_position_x = 0.0;
        double wheel_position_y = 0.0;

        try {
            geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform ("command/base_link", "command/" + steer_names_[i] + "_link", tf2::TimePointZero);
            wheel_position_x                                = tf_stamped.transform.translation.x;
            wheel_position_y                                = tf_stamped.transform.translation.y;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN (this->get_logger (), "Could not get transform from %s to base_link: %s", steer_names_[i].c_str (), ex.what ());
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

    double vx   = A[0][3];
    double vy   = A[1][3];
    double vyaw = A[2][3];

    double delta_t   = (this->now () - last_pose_.header.stamp).seconds ();
    double delta_x   = vx * delta_t;
    double delta_y   = vy * delta_t;
    double delta_yaw = vyaw * delta_t;

    double          last_yaw = tf2::getYaw (last_pose_.pose.orientation);
    double          new_yaw  = last_yaw + delta_yaw;
    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, new_yaw);
    last_pose_.pose.position.x += delta_x * std::cos (last_yaw) - delta_y * std::sin (last_yaw);
    last_pose_.pose.position.y += delta_x * std::sin (last_yaw) + delta_y * std::cos (last_yaw);
    last_pose_.pose.orientation.x = q.x ();
    last_pose_.pose.orientation.y = q.y ();
    last_pose_.pose.orientation.z = q.z ();
    last_pose_.pose.orientation.w = q.w ();
    last_pose_.header.stamp       = this->now ();
    pose_publisher_->publish (last_pose_);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = base_frame_id_;
    twist.header.stamp    = this->now ();
    twist.twist.linear.x  = vx;
    twist.twist.linear.y  = vy;
    twist.twist.angular.z = vyaw;
    twist_publisher_->publish (twist);

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id  = base_frame_id_;
    odom.header.stamp    = this->now ();
    odom.pose.pose       = last_pose_.pose;
    odom.twist.twist     = twist.twist;
    odometry_publisher_->publish (odom);

    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp            = this->now ();
        tf_msg.header.frame_id         = odom_frame_id_;
        tf_msg.child_frame_id          = base_frame_id_;
        tf_msg.transform.translation.x = last_pose_.pose.position.x;
        tf_msg.transform.translation.y = last_pose_.pose.position.y;
        tf_msg.transform.translation.z = last_pose_.pose.position.z;
        tf_msg.transform.rotation      = last_pose_.pose.orientation;
        tf_broadcaster_->sendTransform (tf_msg);
    }
}

}  // namespace wheel_odometry

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (wheel_odometry::wheel_odometry)