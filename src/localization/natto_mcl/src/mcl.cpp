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
    pose_publisher_            = this->create_publisher<geometry_msgs::msg::PoseStamped> ("pose", 10);
    particles_publisher_       = this->create_publisher<geometry_msgs::msg::PoseArray> ("particles", 10);
    occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid> ("occupancy_grid", rclcpp::QoS (rclcpp::KeepLast (1)).transient_local ().reliable (), std::bind (&mcl::occupancy_grid_callback, this, std::placeholders::_1));
    pointcloud2_subscriber_    = this->create_subscription<sensor_msgs::msg::PointCloud2> ("pointcloud2", 1, std::bind (&mcl::pointcloud2_callback, this, std::placeholders::_1));
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
    expansion_radius_position_    = this->declare_parameter<double> ("expansion_radius_position", 1.0);
    expansion_radius_orientation_ = this->declare_parameter<double> ("expansion_radius_orientation", 3.14);
    laser_likelihood_max_dist_    = this->declare_parameter<double> ("laser_likelihood_max_dist", 0.2);
    transform_tolerance_          = this->declare_parameter<double> ("transform_tolerance", 0.1);

    RCLCPP_INFO (this->get_logger (), "MCL node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "map_frame_id: %s", map_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "odom_frame_id: %s", odom_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "base_frame_id: %s", base_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "max_num_particles: %d", max_num_particles_);
    RCLCPP_INFO (this->get_logger (), "min_num_particles: %d", min_num_particles_);
    RCLCPP_INFO (this->get_logger (), "initial_pose_x: %f", initial_pose_x_);
    RCLCPP_INFO (this->get_logger (), "initial_pose_y: %f", initial_pose_y_);
    RCLCPP_INFO (this->get_logger (), "initial_pose_theta: %f", initial_pose_theta_);
    RCLCPP_INFO (this->get_logger (), "motion_noise_position: %f", motion_noise_position_);
    RCLCPP_INFO (this->get_logger (), "motion_noise_orientation: %f", motion_noise_orientation_);
    RCLCPP_INFO (this->get_logger (), "expansion_radius_position: %f", expansion_radius_position_);
    RCLCPP_INFO (this->get_logger (), "expansion_radius_orientation: %f", expansion_radius_orientation_);
    RCLCPP_INFO (this->get_logger (), "laser_likelihood_max_dist: %f", laser_likelihood_max_dist_);
    RCLCPP_INFO (this->get_logger (), "transform_tolerance: %f", transform_tolerance_);

    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.frame_id         = map_frame_id_;
    map_to_odom.child_frame_id          = odom_frame_id_;
    map_to_odom.header.stamp            = this->now () + rclcpp::Duration::from_seconds (transform_tolerance_);
    map_to_odom.transform.translation.x = initial_pose_x_;
    map_to_odom.transform.translation.y = initial_pose_y_;
    map_to_odom.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY (0, 0, initial_pose_theta_);
    map_to_odom.transform.rotation = tf2::toMsg (q);
    tf_broadcaster_->sendTransform (map_to_odom);
    initialize_particles (initial_pose_x_, initial_pose_y_, initial_pose_theta_);

    likelihood_field_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid> ("likelihood_field", rclcpp::QoS (rclcpp::KeepLast (1)).transient_local ().reliable ());
}

void mcl::occupancy_grid_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO (this->get_logger (), "Received occupancy grid map.");
    likelihood_field_.clear ();
    likelihood_field_.resize (msg->info.width, std::vector<uint8_t> (msg->info.height));
    resolution_  = msg->info.resolution;
    int cell_num = static_cast<int> (ceil (laser_likelihood_max_dist_ / resolution_));

    std::vector<uint8_t> weights;
    for (int i = 0; i <= cell_num; i++) {
        weights.push_back (static_cast<uint8_t> (255 * (1.0 - static_cast<double> (i) / cell_num)));
    }
    for (int x = 0; x < msg->info.width; ++x) {
        for (int y = 0; y < msg->info.height; ++y) {
            int index = y * msg->info.width + x;
            if (msg->data[index] == -1) {
                likelihood_field_[x][y] = 0;
            } else if (msg->data[index] > 50) {
                for (int i = -cell_num; i <= cell_num; i++) {
                    for (int j = -cell_num; j <= cell_num; j++) {
                        if (i + x >= 0 && j + y >= 0 && i + x < msg->info.width && j + y < msg->info.height) {
                            likelihood_field_[i + x][j + y] = std::max (likelihood_field_[i + x][j + y], std::min (weights[abs (i)], weights[abs (j)]));
                        }
                    }
                }

            } else {
                likelihood_field_[x][y] = 0;
            }
        }
    }

    nav_msgs::msg::OccupancyGrid likelihood_field_msg;
    likelihood_field_msg.header = msg->header;
    likelihood_field_msg.info   = msg->info;
    likelihood_field_msg.data.resize (msg->info.width * msg->info.height);
    for (int x = 0; x < msg->info.width; ++x) {
        for (int y = 0; y < msg->info.height; ++y) {
            int index                        = y * msg->info.width + x;
            likelihood_field_msg.data[index] = static_cast<int8_t> (likelihood_field_[x][y]);
        }
    }
    likelihood_field_publisher_->publish (likelihood_field_msg);
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
        p.weight *= compute_laser_likelihood (*msg, p);
        total_weight += p.weight;
    }

    for (auto &p : particles_) {
        p.weight /= total_weight;
    }

    if (total_weight > 0.000001) {
        resample_particles ();
    } else {
        for (auto &p : particles_) {
            p.weight = 1.0 / particles_.size ();
        }
    }

    geometry_msgs::msg::Pose mean_pose = get_mean_pose ();

    tf2::Transform tf_map_to_base, tf_odom_to_base, tf_map_to_odom;
    tf2::fromMsg (mean_pose, tf_map_to_base);
    tf2::fromMsg (odom_to_base_link.transform, tf_odom_to_base);

    tf_map_to_odom = tf_map_to_base * tf_odom_to_base.inverse ();

    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp    = builtin_interfaces::msg::Time (rclcpp::Time (msg->header.stamp) + rclcpp::Duration::from_seconds (transform_tolerance_));
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
    for (int i = 0; i < max_num_particles_; i++) {
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
    std::vector<double> accum;
    accum.push_back (particles_[0].weight);
    for (size_t i = 1; i < particles_.size (); i++) {
        accum.push_back (accum.back () + particles_[i].weight);
    }

    std::vector<particle> old (particles_);

    double start = static_cast<double> (rand ()) / (RAND_MAX * particles_.size ());
    double step  = 1.0 / particles_.size ();

    std::vector<int> chosen;

    size_t tick = 0;
    for (size_t i = 0; i < particles_.size (); i++) {
        while (accum[tick] <= start + i * step) {
            tick++;
            if (tick == particles_.size ()) {
                RCLCPP_ERROR (this->get_logger (), "RESAMPLING FAILED");
                exit (1);
            }
        }
        chosen.push_back (tick);
    }

    for (size_t i = 0; i < particles_.size (); i++) {
        particles_[i] = old[chosen[i]];
    }
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
        p.theta = std::fmod (p.theta + M_PI, 2 * M_PI) - M_PI;
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
    double likelihood = 0.0;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x (scan, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y (scan, "y");

    for (; iter_x != iter_x.end () && iter_y != iter_y.end (); ++iter_x, ++iter_y) {
        double x = *iter_x;
        double y = *iter_y;

        double lx = p.x + (x * cos (p.theta) - y * sin (p.theta));
        double ly = p.y + (x * sin (p.theta) + y * cos (p.theta));

        int ix = static_cast<int> (lx / resolution_);
        int iy = static_cast<int> (ly / resolution_);
        if (ix < 0 || iy < 0 || ix >= static_cast<int> (likelihood_field_.size ()) || iy >= static_cast<int> (likelihood_field_[0].size ())) {
            continue;
        }
        likelihood += likelihood_field_[ix][iy];
    }
    return likelihood;
}

}  // namespace mcl

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (mcl::mcl)