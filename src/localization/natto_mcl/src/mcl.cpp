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

mcl::mcl (const rclcpp::NodeOptions &node_options) : Node ("mcl", node_options), rng_ (std::random_device{}()) {
    pose_publisher_                 = this->create_publisher<geometry_msgs::msg::PoseStamped> ("pose", 10);
    particles_publisher_            = this->create_publisher<geometry_msgs::msg::PoseArray> ("particles", 10);
    pose_with_covariance_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped> ("pose_with_covariance", 10);

    occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid> ("occupancy_grid", rclcpp::QoS (rclcpp::KeepLast (1)).transient_local ().reliable (), std::bind (&mcl::occupancy_grid_callback, this, std::placeholders::_1));
    pointcloud2_subscriber_    = this->create_subscription<sensor_msgs::msg::PointCloud2> ("pointcloud2", rclcpp::SensorDataQoS (), std::bind (&mcl::pointcloud2_callback, this, std::placeholders::_1));
    initial_pose_with_covariance_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped> ("initial_pose", 1, std::bind (&mcl::initial_pose_with_covariance_callback, this, std::placeholders::_1));

    use_odom_tf_ = this->declare_parameter<bool> ("use_odom_tf", false);
    if (!use_odom_tf_) {
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry> ("odometry", rclcpp::SensorDataQoS (), std::bind (&mcl::odometry_callback, this, std::placeholders::_1));
    }

    tf_buffer_      = std::make_shared<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_    = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster> (this);

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS> (get_node_base_interface (), get_node_timers_interface (), create_callback_group (rclcpp::CallbackGroupType::MutuallyExclusive, false));
    tf_buffer_->setCreateTimerInterface (timer_interface);

    double frequency                  = this->declare_parameter<double> ("frequency", 40.0);
    map_frame_id_                     = this->declare_parameter<std::string> ("map_frame_id", "map");
    odom_frame_id_                    = this->declare_parameter<std::string> ("odom_frame_id", "odom");
    base_frame_id_                    = this->declare_parameter<std::string> ("base_frame_id", "base_link");
    num_particles_                    = this->declare_parameter<int> ("num_particles", 500);
    initial_pose_x_                   = this->declare_parameter<double> ("initial_pose_x", 0.0);
    initial_pose_y_                   = this->declare_parameter<double> ("initial_pose_y", 0.0);
    initial_pose_yaw_deg_             = this->declare_parameter<double> ("initial_pose_yaw_deg", 0.0);
    motion_noise_xx_                  = this->declare_parameter<double> ("motion_noise_xx", 0.2);
    motion_noise_xy_                  = this->declare_parameter<double> ("motion_noise_xy", 0.1);
    motion_noise_yy_                  = this->declare_parameter<double> ("motion_noise_yy", 0.2);
    motion_noise_yaw_deg_             = this->declare_parameter<double> ("motion_noise_yaw_deg", 0.1);
    normal_noise_position_            = this->declare_parameter<double> ("normal_noise_position", 0.01);
    normal_noise_orientation_deg_     = this->declare_parameter<double> ("normal_noise_orientation_deg", 1.0);
    expansion_radius_position_        = this->declare_parameter<double> ("expansion_radius_position", 1.0);
    expansion_radius_orientation_deg_ = this->declare_parameter<double> ("expansion_radius_orientation_deg", 30.0);
    laser_likelihood_max_dist_        = this->declare_parameter<double> ("laser_likelihood_max_dist", 0.2);
    transform_tolerance_              = this->declare_parameter<double> ("transform_tolerance", 0.2);

    max_trajectory_length_ = static_cast<size_t> (this->declare_parameter<int> ("max_trajectory_length", 1000));
    trajectory_publisher_  = this->create_publisher<geometry_msgs::msg::PoseArray> ("trajectory", 10);

    trajectory_msg_.header.frame_id = map_frame_id_;

    RCLCPP_INFO (this->get_logger (), "MCL node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "use_odom_tf: %s", use_odom_tf_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "frequency: %f", frequency);
    RCLCPP_INFO (this->get_logger (), "map_frame_id: %s", map_frame_id_.c_str ());
    if (use_odom_tf_) {
        RCLCPP_INFO (this->get_logger (), "odom_frame_id: %s", odom_frame_id_.c_str ());
    }
    RCLCPP_INFO (this->get_logger (), "base_frame_id: %s", base_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "num_particles: %d", num_particles_);
    RCLCPP_INFO (this->get_logger (), "initial_pose_x: %f", initial_pose_x_);
    RCLCPP_INFO (this->get_logger (), "initial_pose_y: %f", initial_pose_y_);
    RCLCPP_INFO (this->get_logger (), "initial_pose_yaw_deg: %f", initial_pose_yaw_deg_);
    RCLCPP_INFO (this->get_logger (), "motion_noise_xx: %f", motion_noise_xx_);
    RCLCPP_INFO (this->get_logger (), "motion_noise_xy: %f", motion_noise_xy_);
    RCLCPP_INFO (this->get_logger (), "motion_noise_yy: %f", motion_noise_yy_);
    RCLCPP_INFO (this->get_logger (), "motion_noise_yaw_deg: %f", motion_noise_yaw_deg_);
    RCLCPP_INFO (this->get_logger (), "normal_noise_position: %f", normal_noise_position_);
    RCLCPP_INFO (this->get_logger (), "normal_noise_orientation_deg: %f", normal_noise_orientation_deg_);
    RCLCPP_INFO (this->get_logger (), "expansion_radius_position: %f", expansion_radius_position_);
    RCLCPP_INFO (this->get_logger (), "expansion_radius_orientation_deg: %f", expansion_radius_orientation_deg_);
    RCLCPP_INFO (this->get_logger (), "laser_likelihood_max_dist: %f", laser_likelihood_max_dist_);
    RCLCPP_INFO (this->get_logger (), "transform_tolerance: %f", transform_tolerance_);

    if (use_odom_tf_) {
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.frame_id         = map_frame_id_;
        map_to_odom.child_frame_id          = odom_frame_id_;
        map_to_odom.header.stamp            = this->now () + rclcpp::Duration::from_seconds (transform_tolerance_);
        map_to_odom.transform.translation.x = initial_pose_x_;
        map_to_odom.transform.translation.y = initial_pose_y_;
        map_to_odom.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY (0, 0, initial_pose_yaw_deg_ * M_PI / 180.0);
        map_to_odom.transform.rotation = tf2::toMsg (q);
        tf_broadcaster_->sendTransform (map_to_odom);
    }

    for (int i = 0; i < (1 << 16); i++) {
        cos_[i] = cos (M_PI * i / (1 << 15));
        sin_[i] = sin (M_PI * i / (1 << 15));
    }

    initialize_particles (initial_pose_x_, initial_pose_y_, initial_pose_yaw_deg_ * M_PI / 180.0);

    delta_t_ = 1.0 / frequency;
    timer_   = this->create_wall_timer (std::chrono::duration<double> (delta_t_), std::bind (&mcl::timer_callback, this));
}

void mcl::occupancy_grid_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO (this->get_logger (), "Received occupancy grid map.");
    likelihood_field_.clear ();
    likelihood_field_.resize (msg->info.width, std::vector<uint8_t> (msg->info.height));
    resolution_  = msg->info.resolution;
    width_       = msg->info.width;
    height_      = msg->info.height;
    int cell_num = static_cast<int> (ceil (laser_likelihood_max_dist_ / resolution_));

    std::vector<uint8_t> weights;
    for (int i = 0; i <= cell_num; i++) {
        weights.push_back (static_cast<uint8_t> (255 * (1.0 - static_cast<double> (i) / cell_num)));
    }
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            int index = y * width_ + x;
            if (msg->data[index] == -1) {
                likelihood_field_[x][y] = 0;
            } else if (msg->data[index] > 50) {
                for (int i = -cell_num; i <= cell_num; i++) {
                    for (int j = -cell_num; j <= cell_num; j++) {
                        if (i + x >= 0 && j + y >= 0 && i + x < width_ && j + y < height_) {
                            likelihood_field_[i + x][j + y] = std::max (likelihood_field_[i + x][j + y], std::min (weights[abs (i)], weights[abs (j)]));
                        }
                    }
                }
            }
        }
    }
}

void mcl::pointcloud2_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x (*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y (*msg, "y");

    scan_x_.clear ();
    scan_y_.clear ();
    scan_size_ = 0;

    for (; iter_x != iter_x.end () && iter_y != iter_y.end (); ++iter_x, ++iter_y) {
        scan_x_.push_back (*iter_x);
        scan_y_.push_back (*iter_y);
        scan_size_++;
    }
}

void mcl::odometry_callback (const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odometry_ = *msg;
}

void mcl::timer_callback () {
    if (likelihood_field_.empty ()) {
        RCLCPP_WARN (this->get_logger (), "Likelihood field is not ready yet.");
        return;
    }

    geometry_msgs::msg::TransformStamped odom_to_base_link;

    double delta_x, delta_y, delta_yaw;

    if (use_odom_tf_) {
        try {
            odom_to_base_link = tf_buffer_->lookupTransform (odom_frame_id_, base_frame_id_, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN (this->get_logger (), "Could not get transform from %s to %s: %s", odom_frame_id_.c_str (), base_frame_id_.c_str (), ex.what ());
            return;
        }
        double delta_x_in_odom = odom_to_base_link.transform.translation.x - last_odom_to_base_transform_.translation.x;
        double delta_y_in_odom = odom_to_base_link.transform.translation.y - last_odom_to_base_transform_.translation.y;

        double odom_to_base_link_yaw = tf2::getYaw (odom_to_base_link.transform.rotation);
        double last_odom_yaw         = tf2::getYaw (last_odom_to_base_transform_.rotation);

        delta_yaw = odom_to_base_link_yaw - last_odom_yaw;
        delta_x   = cos (last_map_to_odom_yaw_ - odom_to_base_link_yaw) * delta_x_in_odom - sin (last_map_to_odom_yaw_ - odom_to_base_link_yaw) * delta_y_in_odom;
        delta_y   = sin (last_map_to_odom_yaw_ - odom_to_base_link_yaw) * delta_x_in_odom + cos (last_map_to_odom_yaw_ - odom_to_base_link_yaw) * delta_y_in_odom;

        last_odom_to_base_transform_ = odom_to_base_link.transform;
    } else {
        double delta_x_in_odom = latest_odometry_.pose.pose.position.x - last_odometry_.pose.pose.position.x;
        double delta_y_in_odom = latest_odometry_.pose.pose.position.y - last_odometry_.pose.pose.position.y;

        double odom_to_base_link_yaw = tf2::getYaw (latest_odometry_.pose.pose.orientation);
        double last_odom_yaw         = tf2::getYaw (last_odometry_.pose.pose.orientation);

        delta_x   = cos (last_map_to_odom_yaw_) * delta_x_in_odom - sin (last_map_to_odom_yaw_) * delta_y_in_odom;
        delta_y   = sin (last_map_to_odom_yaw_) * delta_x_in_odom + cos (last_map_to_odom_yaw_) * delta_y_in_odom;
        delta_yaw = odom_to_base_link_yaw - last_odom_yaw;

        last_odometry_ = latest_odometry_;
    }

    while (delta_yaw > +M_PI) delta_yaw -= 2 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;

    motion_update (delta_x, delta_y, delta_yaw);
    double total_weight = 0.0;
    for (auto &p : particles_) {
        p.weight *= compute_laser_likelihood (p);
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
    geometry_msgs::msg::PoseWithCovariance mean_pose = get_mean_pose ();

    if (use_odom_tf_) {
        last_map_to_odom_yaw_ = tf2::getYaw (mean_pose.pose.orientation);

        tf2::Transform tf_map_to_base, tf_odom_to_base, tf_map_to_odom;
        tf2::fromMsg (mean_pose.pose, tf_map_to_base);
        tf2::fromMsg (odom_to_base_link.transform, tf_odom_to_base);

        tf_map_to_odom = tf_map_to_base * tf_odom_to_base.inverse ();

        geometry_msgs::msg::TransformStamped map_to_odom_msg;
        map_to_odom_msg.header.stamp    = this->now () + rclcpp::Duration::from_seconds (transform_tolerance_);
        map_to_odom_msg.header.frame_id = map_frame_id_;
        map_to_odom_msg.child_frame_id  = odom_frame_id_;
        map_to_odom_msg.transform       = tf2::toMsg (tf_map_to_odom);
        tf_broadcaster_->sendTransform (map_to_odom_msg);
    } else {
        geometry_msgs::msg::TransformStamped map_to_base_link;
        map_to_base_link.header.stamp    = this->now () + rclcpp::Duration::from_seconds (transform_tolerance_);
        map_to_base_link.header.frame_id = map_frame_id_;
        map_to_base_link.child_frame_id  = base_frame_id_;
        tf2::Transform tf_map_to_base;
        tf2::fromMsg (mean_pose.pose, tf_map_to_base);
        map_to_base_link.transform = tf2::toMsg (tf_map_to_base);
        tf_broadcaster_->sendTransform (map_to_base_link);
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = map_frame_id_;
    pose_msg.header.stamp    = this->now ();
    pose_msg.pose            = mean_pose.pose;
    pose_publisher_->publish (pose_msg);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_msg;
    pose_with_covariance_msg.header.frame_id = map_frame_id_;
    pose_with_covariance_msg.header.stamp    = this->now ();
    pose_with_covariance_msg.pose            = mean_pose;
    pose_with_covariance_publisher_->publish (pose_with_covariance_msg);

    geometry_msgs::msg::PoseArray particles_msg;
    particles_msg.header.frame_id = map_frame_id_;
    particles_msg.header.stamp    = this->now ();
    for (const auto &p : particles_) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY (0, 0, p.yaw);
        pose.orientation = tf2::toMsg (q);
        particles_msg.poses.push_back (pose);
    }
    particles_publisher_->publish (particles_msg);

    geometry_msgs::msg::Pose pose_for_traj;
    pose_for_traj.position    = pose_msg.pose.position;
    pose_for_traj.orientation = pose_msg.pose.orientation;

    trajectory_msg_.poses.push_back (pose_for_traj);

    if (trajectory_msg_.poses.size () > max_trajectory_length_) {
        size_t remove_count = trajectory_msg_.poses.size () - max_trajectory_length_;
        trajectory_msg_.poses.erase (trajectory_msg_.poses.begin (), trajectory_msg_.poses.begin () + remove_count);
    }

    trajectory_msg_.header.stamp    = this->now ();
    trajectory_msg_.header.frame_id = map_frame_id_;
    trajectory_publisher_->publish (trajectory_msg_);
}

void mcl::initial_pose_with_covariance_callback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    trajectory_msg_.poses.clear ();
    initialize_particles (msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw (msg->pose.pose.orientation));
}

void mcl::initialize_particles (double x, double y, double yaw) {
    particles_.clear ();
    std::uniform_real_distribution<double> dx (-expansion_radius_position_, expansion_radius_position_);
    std::uniform_real_distribution<double> dy (-expansion_radius_position_, expansion_radius_position_);
    std::uniform_real_distribution<double> dyaw (-expansion_radius_orientation_deg_ * M_PI / 180.0, expansion_radius_orientation_deg_ * M_PI / 180.0);

    RCLCPP_INFO (this->get_logger (), "Initializing %d particles", num_particles_);
    for (int i = 0; i < num_particles_; i++) {
        particle p;
        p.x      = x + dx (rng_);
        p.y      = y + dy (rng_);
        p.yaw    = yaw + dyaw (rng_);
        p.weight = 1.0 / num_particles_;
        particles_.push_back (p);
    }
    RCLCPP_INFO (this->get_logger (), "Particles initialized, size: %zu", particles_.size ());
}
void mcl::resample_particles () {
    std::vector<particle> old_particles (particles_);
    std::vector<double>   cumulative_weights;
    std::vector<int>      selected_indices;

    std::uniform_real_distribution<double> dist (0.0, 1.0 / particles_.size ());
    cumulative_weights.push_back (particles_[0].weight);
    for (size_t i = 1; i < particles_.size (); i++) {
        cumulative_weights.push_back (cumulative_weights.back () + particles_[i].weight);
    }

    double start_offset = dist (rng_);
    double step         = 1.0 / particles_.size ();

    size_t cumulative_index = 0;
    for (size_t i = 0; i < particles_.size (); i++) {
        while (cumulative_weights[cumulative_index] <= start_offset + i * step) {
            cumulative_index++;
            if (cumulative_index == particles_.size ()) {
                RCLCPP_FATAL (this->get_logger (), "RESAMPLING FAILED: Unable to resample particles. Initiating shutdown.");
                rclcpp::shutdown ();
                return;
            }
        }
        selected_indices.push_back (cumulative_index);
    }
    for (size_t i = 0; i < particles_.size (); i++) {
        particles_[i] = old_particles[selected_indices[i]];
    }
}

void mcl::motion_update (double delta_x, double delta_y, double delta_yaw) {
    if (particles_.empty ()) return;

    std::normal_distribution<double> noise_position (0.0, normal_noise_position_);
    std::normal_distribution<double> noise_yaw (0.0, normal_noise_orientation_deg_ * M_PI / 180.0);
    std::normal_distribution<double> normal_noise (0.0, 1.0);

    double x_dev   = sqrt (abs (delta_x) * motion_noise_xx_ * motion_noise_xx_ + abs (delta_y) * motion_noise_xy_ * motion_noise_xy_);
    double y_dev   = sqrt (abs (delta_x) * motion_noise_xy_ * motion_noise_xy_ + abs (delta_y) * motion_noise_yy_ * motion_noise_yy_);
    double yaw_dev = sqrt (abs (delta_yaw) * motion_noise_yaw_deg_ * motion_noise_yaw_deg_);

    for (auto &p : particles_) {
        double nx   = noise_position (rng_) + normal_noise (rng_) * x_dev;
        double ny   = noise_position (rng_) + normal_noise (rng_) * y_dev;
        double nyaw = noise_yaw (rng_) + normal_noise (rng_) * yaw_dev;

        p.x += delta_x + nx;
        p.y += delta_y + ny;
        p.yaw += delta_yaw + nyaw;
    }
}

geometry_msgs::msg::PoseWithCovariance mcl::get_mean_pose () {
    geometry_msgs::msg::PoseWithCovariance pose_with_covariance;

    double x_sum = 0.0, y_sum = 0.0, s_sum = 0.0, c_sum = 0.0;
    double xx_sum = 0.0, yy_sum = 0.0, ss_sum = 0.0, cc_sum = 0.0;
    double xy_sum = 0.0, xt_sum = 0.0, yt_sum = 0.0;

    for (auto &p : particles_) {
        x_sum += p.x;
        y_sum += p.y;
        uint16_t theta_16bit = get_16bit_theta (p.yaw);
        s_sum += sin_[theta_16bit];
        c_sum += cos_[theta_16bit];
    }

    double x_mean   = x_sum / particles_.size ();
    double y_mean   = y_sum / particles_.size ();
    double s_mean   = s_sum / particles_.size ();
    double c_mean   = c_sum / particles_.size ();
    double yaw_mean = std::atan2 (s_mean, c_mean);

    for (auto &p : particles_) {
        xx_sum += pow (p.x - x_mean, 2);
        yy_sum += pow (p.y - y_mean, 2);
        ss_sum += pow (sin_[get_16bit_theta (p.yaw)] - s_mean, 2);
        cc_sum += pow (cos_[get_16bit_theta (p.yaw)] - c_mean, 2);
    }

    double x_dev   = xx_sum / (particles_.size () - 1);
    double y_dev   = yy_sum / (particles_.size () - 1);
    double s_dev   = ss_sum / (particles_.size () - 1);
    double c_dev   = cc_sum / (particles_.size () - 1);
    double yaw_dev = (s_dev + c_dev) / 2.0;

    for (auto &p : particles_) {
        xy_sum += (p.x - x_mean) * (p.y - y_mean);
        xt_sum += (p.x - x_mean) * (p.yaw - yaw_mean);
        yt_sum += (p.y - y_mean) * (p.yaw - yaw_mean);
    }

    double xy_cov = xy_sum / (particles_.size () - 1);
    double xt_cov = xt_sum / (particles_.size () - 1);
    double yt_cov = yt_sum / (particles_.size () - 1);

    pose_with_covariance.pose.position.x = x_mean;
    pose_with_covariance.pose.position.y = y_mean;
    pose_with_covariance.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY (0, 0, yaw_mean);
    pose_with_covariance.pose.orientation = tf2::toMsg (q);

    pose_with_covariance.covariance[6 * 0 + 0] = x_dev;
    pose_with_covariance.covariance[6 * 1 + 1] = y_dev;
    pose_with_covariance.covariance[6 * 2 + 2] = yaw_dev;
    pose_with_covariance.covariance[6 * 0 + 1] = xy_cov;
    pose_with_covariance.covariance[6 * 1 + 0] = xy_cov;
    pose_with_covariance.covariance[6 * 0 + 5] = xt_cov;
    pose_with_covariance.covariance[6 * 5 + 0] = xt_cov;
    pose_with_covariance.covariance[6 * 1 + 5] = yt_cov;
    pose_with_covariance.covariance[6 * 5 + 1] = yt_cov;

    return pose_with_covariance;
}

double mcl::compute_laser_likelihood (const particle &p) {
    double likelihood = 0.0;

    uint16_t theta_16bit = get_16bit_theta (p.yaw);
    double   sin_theta   = sin_[theta_16bit];
    double   cos_theta   = cos_[theta_16bit];

    for (int i = 0; i < scan_size_; i++) {
        double x = scan_x_[i];
        double y = scan_y_[i];

        double lx = p.x + (x * cos_theta - y * sin_theta);
        double ly = p.y + (x * sin_theta + y * cos_theta);

        int ix = static_cast<int> (lx / resolution_);
        int iy = static_cast<int> (ly / resolution_);
        if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
            continue;
        }
        likelihood += likelihood_field_[ix][iy];
    }
    return likelihood;
}

uint16_t mcl::mcl::get_16bit_theta (double theta) {
    int result = theta / M_PI * (1 << 15);
    while (result < 0) {
        result += (1 << 16);
    }
    while (result >= (1 << 16)) {
        result -= (1 << 16);
    }

    return (uint16_t)result;
}

}  // namespace mcl

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (mcl::mcl)