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

#include "natto_wheel_odometry/two_wheel_odometry.hpp"

namespace two_wheel_odometry {

two_wheel_odometry::two_wheel_odometry (const rclcpp::NodeOptions &node_options) : Node ("two_wheel_odometry", node_options) {
    twist_publisher_    = this->create_publisher<geometry_msgs::msg::TwistStamped> ("twist", 10);
    pose_publisher_     = this->create_publisher<geometry_msgs::msg::PoseStamped> ("pose", 10);
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry> ("odometry", rclcpp::SensorDataQoS ());

    two_wheel_subscriber_ = this->create_subscription<natto_msgs::msg::TwoWheel> ("two_wheel_result", 10, std::bind (&two_wheel_odometry::two_wheel_callback, this, std::placeholders::_1));
    tf_broadcaster_  = std::make_shared<tf2_ros::TransformBroadcaster> (this);

    wheel_radius_     = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_base_       = this->declare_parameter<double> ("wheel_base", 0.5);
    odom_frame_id_    = this->declare_parameter<std::string> ("odom_frame_id", "odom");
    base_frame_id_    = this->declare_parameter<std::string> ("base_frame_id", "base_link");
    publish_tf_       = this->declare_parameter<bool> ("publish_tf", true);

    RCLCPP_INFO (this->get_logger (), "two_wheel_odometry node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "wheel_base: %.2f m", wheel_base_);
    RCLCPP_INFO (this->get_logger (), "publish_tf : %s", publish_tf_ ? "true" : "false");

    RCLCPP_INFO (this->get_logger (), "odom_frame_id : %s", odom_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "base_frame_id : %s", base_frame_id_.c_str ());

    last_pose_.header.frame_id = odom_frame_id_;
    last_pose_.header.stamp    = this->now ();
}

void two_wheel_odometry::two_wheel_callback (const natto_msgs::msg::TwoWheel::SharedPtr msg) {
    if (msg->wheel_speed.size () != 2) {
        RCLCPP_ERROR (this->get_logger (), "Received two_wheel message with incorrect number of wheels.");
        return;
    }

    // Calculate linear and angular velocities from wheel speeds
    double v_left = msg->wheel_speed[0] * 2.0 * M_PI * wheel_radius_;
    double v_right = msg->wheel_speed[1] * 2.0 * M_PI * wheel_radius_;

    // Differential drive kinematics
    double vx   = (v_left + v_right) / 2.0;
    double vy   = 0.0;  // No lateral movement for differential drive
    double vyaw = (v_right - v_left) / wheel_base_;

    double delta_t   = (this->now () - last_pose_.header.stamp).seconds ();
    double delta_x   = vx * delta_t;
    double delta_y   = 0.0;  // No lateral movement for differential drive
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

}  // namespace two_wheel_odometry

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (two_wheel_odometry::two_wheel_odometry)
