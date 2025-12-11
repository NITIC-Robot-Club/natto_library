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

#include "natto_wheel_odometry/omni_odometry.hpp"

namespace omni_odometry {

omni_odometry::omni_odometry (const rclcpp::NodeOptions &node_options) : Node ("omni_odometry", node_options) {
    twist_publisher_    = this->create_publisher<geometry_msgs::msg::TwistStamped> ("twist", 10);
    pose_publisher_     = this->create_publisher<geometry_msgs::msg::PoseStamped> ("pose", 10);
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry> ("odometry", rclcpp::SensorDataQoS ());

    omni_subscriber_ = this->create_subscription<natto_msgs::msg::Omni> ("omni_result", 10, std::bind (&omni_odometry::omni_callback, this, std::placeholders::_1));
    tf_broadcaster_  = std::make_shared<tf2_ros::TransformBroadcaster> (this);

    wheel_radius_    = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_position_x = this->declare_parameter<std::vector<double>> ("wheel_position_x", {0.5, -0.5, -0.5, 0.5});
    wheel_position_y = this->declare_parameter<std::vector<double>> ("wheel_position_y", {0.5, 0.5, -0.5, -0.5});
    wheel_angle      = this->declare_parameter<std::vector<double>> ("wheel_angle_deg", {-45.0, 45.0, 135.0, -135.0});
    frame_id_        = this->declare_parameter<std::string> ("odom_frame_id", "odom");
    child_frame_id_  = this->declare_parameter<std::string> ("base_frame_id", "base_link");
    publish_tf_      = this->declare_parameter<bool> ("publish_tf", true);

    num_wheels_ = wheel_position_x.size ();
    if (wheel_position_y.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_position_x and wheel_position_y must have the same size.");
        throw std::runtime_error ("wheel_position_x and wheel_position_y must have the same size.");
    }
    if (wheel_angle.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_angle must have the same size as wheel_position_x and wheel_position_y.");
        throw std::runtime_error ("wheel_angle must have the same size as wheel_position_x and wheel_position_y.");
    }

    RCLCPP_INFO (this->get_logger (), "omni_odometry node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "Wheel radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %d", num_wheels_);
    for (int i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "Wheel %d position: (%.2f, %.2f), angle: %.2f", i, wheel_position_x[i], wheel_position_y[i], wheel_angle[i]);
    }
    RCLCPP_INFO (this->get_logger (), "frame id : %s", frame_id_.c_str ());

    last_pose.header.frame_id = frame_id_;
    last_pose.header.stamp    = this->now ();
}

void omni_odometry::omni_callback (const natto_msgs::msg::Omni::SharedPtr msg) {
    if (msg->wheel_speed.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "Received omni message with incorrect number of wheels.");
        return;
    }

    double ATA[3][3] = {};  // A^T * A
    double ATb[3]    = {};  // A^T * b

    for (int i = 0; i < num_wheels_; i++) {
        double angle = wheel_angle[i] * M_PI / 180.0;
        double speed = msg->wheel_speed[i] * 2.0 * M_PI * wheel_radius_;

        double c     = std::cos (angle);
        double s     = std::sin (angle);
        double ax[3] = {c, s, (-wheel_position_y[i] * c + wheel_position_x[i] * s)};
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
    double vx   = A[0][3];
    double vy   = A[1][3];
    double vyaw = A[2][3];

    double delta_t   = (this->now () - last_pose.header.stamp).seconds ();
    double delta_x   = vx * delta_t;
    double delta_y   = vy * delta_t;
    double delta_yaw = vyaw * delta_t;

    double          last_yaw = tf2::getYaw (last_pose.pose.orientation);
    double          new_yaw  = last_yaw + delta_yaw;
    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, new_yaw);
    last_pose.pose.position.x += delta_x * std::cos (last_yaw) - delta_y * std::sin (last_yaw);
    last_pose.pose.position.y += delta_x * std::sin (last_yaw) + delta_y * std::cos (last_yaw);
    last_pose.pose.orientation.x = q.x ();
    last_pose.pose.orientation.y = q.y ();
    last_pose.pose.orientation.z = q.z ();
    last_pose.pose.orientation.w = q.w ();
    last_pose.header.stamp       = this->now ();
    pose_publisher_->publish (last_pose);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = child_frame_id_;
    twist.header.stamp    = this->now ();
    twist.twist.linear.x  = vx;
    twist.twist.linear.y  = vy;
    twist.twist.angular.z = vyaw;
    twist_publisher_->publish (twist);

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id  = child_frame_id_;
    odom.header.stamp    = this->now ();
    odom.pose.pose       = last_pose.pose;
    odom.twist.twist     = twist.twist;
    odometry_publisher_->publish (odom);

    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp            = this->now ();
        tf_msg.header.frame_id         = frame_id_;
        tf_msg.child_frame_id          = child_frame_id_;
        tf_msg.transform.translation.x = last_pose.pose.position.x;
        tf_msg.transform.translation.y = last_pose.pose.position.y;
        tf_msg.transform.translation.z = last_pose.pose.position.z;
        tf_msg.transform.rotation      = last_pose.pose.orientation;
        tf_broadcaster_->sendTransform (tf_msg);
    }
}

}  // namespace omni_odometry

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (omni_odometry::omni_odometry)