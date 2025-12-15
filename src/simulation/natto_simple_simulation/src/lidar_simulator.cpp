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

#include "natto_simple_simulation/lidar_simulator.hpp"

namespace lidar_simulator {

lidar_simulator::lidar_simulator (const rclcpp::NodeOptions &node_options) : Node ("lidar_simulator", node_options) {
    laser_publisher_            = this->create_publisher<sensor_msgs::msg::LaserScan> ("laserscan", rclcpp::SensorDataQoS ());
    simulation_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped> ("simulation_pose", 1, std::bind (&lidar_simulator::simulation_pose_callback, this, std::placeholders::_1));
    map_subscriber_             = this->create_subscription<natto_msgs::msg::Map> ("map", rclcpp::QoS (rclcpp::KeepLast (1)).transient_local ().reliable (), std::bind (&lidar_simulator::map_callback, this, std::placeholders::_1));

    position_x_            = this->declare_parameter<double> ("position_x", 0.0);
    position_y_            = this->declare_parameter<double> ("position_y", 0.0);
    position_z_            = this->declare_parameter<double> ("position_z", 0.0);
    angle_                 = this->declare_parameter<double> ("angle_deg", 0.0);
    range_min_             = this->declare_parameter<double> ("range_min", 0.0);
    range_max_             = this->declare_parameter<double> ("range_max", 35.0);
    angle_min_             = this->declare_parameter<double> ("angle_min", -1.57);
    angle_max_             = this->declare_parameter<double> ("angle_max", 1.57);
    simulation_resolution_ = this->declare_parameter<double> ("simulation_resolution", 0.01);
    point_rate_            = this->declare_parameter<int> ("point_rate", 43200);
    frequency_             = this->declare_parameter<double> ("frequency", 30);
    frame_id_              = this->declare_parameter<std::string> ("frame_id", "laser_frame");

    RCLCPP_INFO (this->get_logger (), "lidar_simualtor node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "position_x: %.2f m", position_x_);
    RCLCPP_INFO (this->get_logger (), "position_y: %.2f m", position_y_);
    RCLCPP_INFO (this->get_logger (), "angle_deg: %.2f deg", angle_);
    RCLCPP_INFO (this->get_logger (), "range_min: %.2f m", range_min_);
    RCLCPP_INFO (this->get_logger (), "range_max: %.2f m", range_max_);
    RCLCPP_INFO (this->get_logger (), "angle_min: %.2f rad", angle_min_);
    RCLCPP_INFO (this->get_logger (), "angle_max: %.2f rad", angle_max_);
    RCLCPP_INFO (this->get_logger (), "simulation_resolution: %.2f m", simulation_resolution_);
    RCLCPP_INFO (this->get_logger (), "point_rate: %d points/sec", point_rate_);
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency_);
    RCLCPP_INFO (this->get_logger (), "frame_id: %s", frame_id_.c_str ());

    distribution_ = std::uniform_real_distribution<double> (-simulation_resolution_, simulation_resolution_);

    // タイマー宣言例:
    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency_), std::bind (&lidar_simulator::timer_callback, this));
}

void lidar_simulator::simulation_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    simulation_pose_ = msg;
}

void lidar_simulator::map_callback (const natto_msgs::msg::Map::SharedPtr msg) {
    map_ = msg;
}

void lidar_simulator::timer_callback () {
    if (simulation_pose_ == nullptr || map_ == nullptr) return;

    double                    yaw = tf2::getYaw (simulation_pose_->pose.orientation);
    geometry_msgs::msg::Point start_point;
    start_point.x = simulation_pose_->pose.position.x + position_x_ * cos (yaw) - position_y_ * sin (yaw);
    start_point.y = simulation_pose_->pose.position.y + position_x_ * sin (yaw) + position_y_ * cos (yaw);

    double lidar_yaw  = yaw + angle_ * M_PI / 180.0;
    int    num_points = point_rate_ / frequency_;

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp    = this->now ();
    scan.header.frame_id = frame_id_;
    scan.angle_min       = angle_min_;
    scan.angle_max       = angle_max_;
    scan.angle_increment = (angle_max_ - angle_min_) / num_points;
    scan.time_increment  = 0.0;
    scan.scan_time       = 1.0 / frequency_;
    scan.range_min       = range_min_;
    scan.range_max       = range_max_;
    scan.ranges.resize (num_points);

    for (int i = 0; i < num_points; i++) {
        double angle = angle_min_ + i * scan.angle_increment + lidar_yaw;
        double dx    = cos (angle);
        double dy    = sin (angle);

        double closest_range = range_max_;

        for (const auto &seg : map_->line_segments.line_segments) {
            if (position_z_ < seg.start.z || position_z_ > seg.end.z) continue;
            double vx = seg.end.x - seg.start.x;
            double vy = seg.end.y - seg.start.y;

            double det = dx * vy - dy * vx;
            if (fabs (det) < 1e-9) continue;  // 並行

            double t = ((seg.start.x - start_point.x) * vy - (seg.start.y - start_point.y) * vx) / det;
            double u = ((seg.start.x - start_point.x) * dy - (seg.start.y - start_point.y) * dx) / det;

            if (t >= 0.0 && t <= range_max_ && u >= 0.0 && u <= 1.0) {
                if (t < closest_range) closest_range = t;
            }
        }

        for (const auto &circle : map_->circles.circles) {
            if (position_z_ < circle.center.z) continue;
            double fx = start_point.x - circle.center.x;
            double fy = start_point.y - circle.center.y;

            double a = dx * dx + dy * dy;
            double b = 2 * (fx * dx + fy * dy);
            double c = fx * fx + fy * fy - circle.radius * circle.radius;

            double discriminant = b * b - 4 * a * c;
            if (discriminant < 0) continue;

            discriminant = sqrt (discriminant);
            double t1    = (-b - discriminant) / (2 * a);
            double t2    = (-b + discriminant) / (2 * a);

            auto check_t = [&] (double t) {
                if (t >= 0.0 && t <= range_max_) {
                    double hit_x     = start_point.x + t * dx;
                    double hit_y     = start_point.y + t * dy;
                    double angle_hit = atan2 (hit_y - circle.center.y, hit_x - circle.center.x);
                    if (angle_hit < circle.start_angle) angle_hit += 2 * M_PI;
                    if (angle_hit <= circle.end_angle) {
                        if (t < closest_range) closest_range = t;
                    }
                }
            };
            check_t (t1);
            check_t (t2);
        }
        scan.ranges[i] = closest_range + distribution_ (generator_);
        ;
    }
    laser_publisher_->publish (scan);
}

}  // namespace lidar_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (lidar_simulator::lidar_simulator)