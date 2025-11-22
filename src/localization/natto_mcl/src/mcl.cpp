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

#include "natto_mcl/mcl.hpp"

namespace mcl {

mcl::mcl (const rclcpp::NodeOptions &node_options)
    : Node ("mcl", node_options), rng_ (std::random_device{}()), tf_buffer_ (std::make_shared<tf2_ros::Buffer> (this->get_clock ())), tf_listener_ (std::make_shared<tf2_ros::TransformListener> (*tf_buffer_)) {
    pose_publisher_                          = this->create_publisher<geometry_msgs::msg::PoseStamped> ("pose", 10);
    particles_publisher_                     = this->create_publisher<geometry_msgs::msg::PoseArray> ("particles", 10);
    map_subscriber_                          = this->create_subscription<natto_msgs::msg::Map> ("map", rclcpp::QoS (rclcpp::KeepLast (1)).transient_local ().reliable (), std::bind (&mcl::map_callback, this, std::placeholders::_1));
    pointcloud2_subscriber_                  = this->create_subscription<sensor_msgs::msg::PointCloud2> ("pointcloud2", 1, std::bind (&mcl::pointcloud2_callback, this, std::placeholders::_1));
    initial_pose_with_covariance_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped> ("initial_pose", 10, std::bind (&mcl::initial_pose_with_covariance_callback, this, std::placeholders::_1));

    tf_buffer_      = std::make_shared<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_    = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster> (this);

    map_frame_id_                 = this->declare_parameter<std::string> ("map_frame_id", "map");
    odom_frame_id_                = this->declare_parameter<std::string> ("odom_frame_id", "odom");
    base_frame_id_                = this->declare_parameter<std::string> ("base_frame_id", "base_link");
    max_num_particles_            = this->declare_parameter<int> ("max_num_particles", 1000);
    min_num_particles_            = this->declare_parameter<int> ("min_num_particles", 100);
    initial_pose_x_               = this->declare_parameter<double> ("initial_pose_x", 0.0);
    initial_pose_y_               = this->declare_parameter<double> ("initial_pose_y", 0.0);
    initial_pose_theta_           = this->declare_parameter<double> ("initial_pose_theta", 0.0);
    motion_noise_position_        = this->declare_parameter<double> ("motion_noise_position", 0.01);
    motion_noise_orientation_     = this->declare_parameter<double> ("motion_noise_orientation", 0.01);
    alpha_threshold_              = this->declare_parameter<double> ("alpha_threshold", 0.5);
    expansion_radius_position_    = this->declare_parameter<double> ("expansion_radius_position", 1.0);
    expansion_radius_orientation_ = this->declare_parameter<double> ("expansion_radius_orientation", 3.14);
    laser_likelihood_max_dist_    = this->declare_parameter<double> ("laser_likelihood_max_dist", 0.2);

    RCLCPP_INFO (this->get_logger (), "MCL node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "map_frame_id: %s", map_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "odom_frame_id: %s", odom_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "base_frame_id: %s", base_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "max_num_particles: %d", max_num_particles_);
    RCLCPP_INFO (this->get_logger (), "min_num_particles: %d", min_num_particles_);
    RCLCPP_INFO (this->get_logger (), "initial_pose_x: %f", initial_pose_x_);
    RCLCPP_INFO (this->get_logger (), "initial_pose_y: %f", initial_pose_y_);
    RCLCPP_INFO (this->get_logger (), "initial_pose_theta: %f", initial_pose_theta_);
    RCLCPP_INFO (this->get_logger (), "alpha_threshold: %f", alpha_threshold_);
    RCLCPP_INFO (this->get_logger (), "expansion_radius_position: %f", expansion_radius_position_);
    RCLCPP_INFO (this->get_logger (), "expansion_radius_orientation: %f", expansion_radius_orientation_);
    RCLCPP_INFO (this->get_logger (), "laser_likelihood_max_dist: %f", laser_likelihood_max_dist_);

    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.frame_id         = map_frame_id_;
    map_to_odom.child_frame_id          = odom_frame_id_;
    map_to_odom.header.stamp            = this->now ();
    map_to_odom.transform.translation.x = initial_pose_x_;
    map_to_odom.transform.translation.y = initial_pose_y_;
    map_to_odom.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY (0, 0, initial_pose_theta_);
    map_to_odom.transform.rotation = tf2::toMsg (q);
    tf_broadcaster_->sendTransform (map_to_odom);
}

void mcl::map_callback (const natto_msgs::msg::Map::SharedPtr msg) {
    map_ = *msg;
    initialize_particles (initial_pose_x_, initial_pose_y_, initial_pose_theta_);
}

void mcl::pointcloud2_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped odom_to_base_link;
    try {
        odom_to_base_link = tf_buffer_->lookupTransform (odom_frame_id_, base_frame_id_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN (this->get_logger (), "Could not get transform from %s to %s: %s", odom_frame_id_.c_str (), base_frame_id_.c_str (), ex.what ());
        return;
    }

    double delta_x       = odom_to_base_link.transform.translation.x - last_odom_transform_.translation.x;
    double delta_y       = odom_to_base_link.transform.translation.y - last_odom_transform_.translation.y;
    double delta_theta   = tf2::getYaw (odom_to_base_link.transform.rotation) - tf2::getYaw (last_odom_transform_.rotation);
    last_odom_transform_ = odom_to_base_link.transform;

    motion_update (delta_x, delta_y, delta_theta);

    double total_weight = 0.0;
    for (auto &p : particles_) {
        p.weight = compute_laser_likelihood (*msg, p);
        total_weight += p.weight;
    }

    if (total_weight < alpha_threshold_) {
        resample_particles ();
    }

    geometry_msgs::msg::Pose mean_pose = get_mean_pose ();

    tf2::Transform tf_map_to_base, tf_odom_to_base, tf_map_to_odom;
    tf2::fromMsg (mean_pose, tf_map_to_base);
    tf2::fromMsg (odom_to_base_link.transform, tf_odom_to_base);

    tf_map_to_odom = tf_map_to_base * tf_odom_to_base.inverse ();

    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp    = this->now ();
    map_to_odom_msg.header.frame_id = map_frame_id_;
    map_to_odom_msg.child_frame_id  = odom_frame_id_;
    map_to_odom_msg.transform       = tf2::toMsg (tf_map_to_odom);
    tf_broadcaster_->sendTransform (map_to_odom_msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = map_frame_id_;
    pose_msg.header.stamp    = this->now ();
    pose_msg.pose            = mean_pose;
    pose_publisher_->publish (pose_msg);

    geometry_msgs::msg::PoseArray particles_msg;
    particles_msg.header.frame_id = map_frame_id_;
    particles_msg.header.stamp    = this->now ();
    for (const auto &p : particles_) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY (0, 0, p.theta);
        pose.orientation = tf2::toMsg (q);
        particles_msg.poses.push_back (pose);
    }
    particles_publisher_->publish (particles_msg);
}

void mcl::initial_pose_with_covariance_callback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    initialize_particles (msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw (msg->pose.pose.orientation));
}

void mcl::initialize_particles (double x, double y, double theta) {
    particles_.clear ();
    std::uniform_real_distribution<double> dx (-expansion_radius_position_, expansion_radius_position_);
    std::uniform_real_distribution<double> dy (-expansion_radius_position_, expansion_radius_position_);
    std::uniform_real_distribution<double> dtheta (-expansion_radius_orientation_, expansion_radius_orientation_);

    RCLCPP_INFO (this->get_logger (), "Initializing %d particles", max_num_particles_);
    for (int i = 0; i < max_num_particles_; ++i) {
        particle p;
        p.x      = x + dx (rng_);
        p.y      = y + dy (rng_);
        p.theta  = theta + dtheta (rng_);
        p.weight = 1.0 / max_num_particles_;
        particles_.push_back (p);
    }
    RCLCPP_INFO (this->get_logger (), "Particles initialized, size: %zu", particles_.size ());
}

void mcl::resample_particles () {
    if (particles_.empty ()) return;
    size_t N = particles_.size ();

    double total_weight = 0.0;
    for (auto &p : particles_) total_weight += p.weight;
    if (total_weight <= 0.0) {
        for (auto &p : particles_) p.weight = 1.0 / N;
        total_weight = 1.0;
    }
    for (auto &p : particles_) p.weight /= total_weight;

    std::vector<particle> new_particles;
    new_particles.reserve (N);

    double                                 step = 1.0 / N;
    std::uniform_real_distribution<double> dist (0.0, step);
    double                                 r = dist (rng_);

    size_t idx = 0;
    double c   = particles_[0].weight;

    for (size_t m = 0; m < N; ++m) {
        double U = r + m * step;
        while (U > c && idx < N - 1) {
            idx++;
            c += particles_[idx].weight;
        }
        new_particles.push_back (particles_[idx]);
        new_particles.back ().weight = 1.0 / N;
    }

    particles_ = std::move (new_particles);
}

void mcl::motion_update (double delta_x, double delta_y, double delta_theta) {
    if (particles_.empty ()) return;

    std::normal_distribution<double> noise_x (0.0, motion_noise_position_);
    std::normal_distribution<double> noise_y (0.0, motion_noise_position_);
    std::normal_distribution<double> noise_theta (0.0, motion_noise_orientation_);

    for (auto &p : particles_) {
        double nx     = noise_x (rng_);
        double ny     = noise_y (rng_);
        double ntheta = noise_theta (rng_);

        p.x += delta_x + nx;
        p.y += delta_y + ny;
        p.theta += delta_theta + ntheta;

        p.theta = std::atan2 (std::sin (p.theta), std::cos (p.theta));
    }
}

geometry_msgs::msg::Pose mcl::get_mean_pose () {
    geometry_msgs::msg::Pose pose;
    double                   x_sum = 0.0, y_sum = 0.0, s_sum = 0.0, c_sum = 0.0;

    for (auto &p : particles_) {
        x_sum += p.x;
        y_sum += p.y;
        s_sum += std::sin (p.theta);
        c_sum += std::cos (p.theta);
    }

    pose.position.x = x_sum / particles_.size ();
    pose.position.y = y_sum / particles_.size ();
    pose.position.z = 0.0;

    double          avg_theta = std::atan2 (s_sum / particles_.size (), c_sum / particles_.size ());
    tf2::Quaternion q;
    q.setRPY (0, 0, avg_theta);
    pose.orientation = tf2::toMsg (q);

    return pose;
}
double mcl::compute_laser_likelihood (const sensor_msgs::msg::PointCloud2 &scan, const particle &p) {
    const float    max_dist   = laser_likelihood_max_dist_;
    const int      point_step = scan.point_step;
    const uint8_t *data_ptr   = scan.data.data ();
    const int      width      = scan.width;

    const double sigma        = 0.05;
    const double inv_2_sigma2 = 1.0 / (2.0 * sigma * sigma);

    const int skip = 5;

    double log_likelihood = 0.0;

    for (int i = 0; i < width; i += skip) {
        const float *point = reinterpret_cast<const float *> (data_ptr + i * point_step);
        const double x     = point[0];
        const double y     = point[1];

        const double cos_t = std::cos (p.theta);
        const double sin_t = std::sin (p.theta);

        const double px = cos_t * x - sin_t * y + p.x;
        const double py = sin_t * x + cos_t * y + p.y;

        double min_dist = max_dist;

        for (const auto &ls : map_.line_segments.line_segments) {
            const double dx    = ls.end.x - ls.start.x;
            const double dy    = ls.end.y - ls.start.y;
            const double denom = dx * dx + dy * dy;

            double t = ((px - ls.start.x) * dx + (py - ls.start.y) * dy) / denom;
            t        = std::clamp (t, 0.0, 1.0);

            const double cx = ls.start.x + t * dx;
            const double cy = ls.start.y + t * dy;

            const double d = std::hypot (px - cx, py - cy);
            if (d < min_dist) {
                min_dist = d;
                if (min_dist < 0.01) break;
            }
        }

        for (const auto &c : map_.circles.circles) {
            const double d = std::abs (std::hypot (px - c.center.x, py - c.center.y) - c.radius);
            if (d < min_dist) {
                min_dist = d;
                if (min_dist < 0.01) break;
            }
        }

        log_likelihood += -min_dist * min_dist * inv_2_sigma2;
    }

    return std::exp (log_likelihood);
}

}  // namespace mcl

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (mcl::mcl)