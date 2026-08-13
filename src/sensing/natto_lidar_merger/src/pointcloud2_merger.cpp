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

#include "natto_lidar_merger/pointcloud2_merger.hpp"

#include <chrono>

namespace pointcloud2_merger {

pointcloud2_merger::pointcloud2_merger (const rclcpp::NodeOptions &node_options) : Node ("pointcloud2_merger", node_options) {
    pointcloud2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2> ("merged_pointcloud2", rclcpp::SensorDataQoS ());
    footprint_subscriber_  = this->create_subscription<geometry_msgs::msg::PolygonStamped> ("footprint", 1, std::bind (&pointcloud2_merger::footprint_callback, this, std::placeholders::_1));

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    std::vector<std::string> lidar_topics = this->declare_parameter<std::vector<std::string>> ("lidar_topics", {"/lidar1_pointcloud2", "/lidar2_pointcloud2"});
    std::string              merge_mode   = this->declare_parameter<std::string> ("mode", "merge");

    if (merge_mode == "merge") {
        merge_mode_ = merge_mode_t::merge;
    } else if (merge_mode == "passthrough") {
        merge_mode_ = merge_mode_t::passthrough;
    } else {
        RCLCPP_WARN (this->get_logger (), "Unknown mode '%s', falling back to 'merge'.", merge_mode.c_str ());
        merge_mode_ = merge_mode_t::merge;
        merge_mode  = "merge";
    }

    num_lidars_ = lidar_topics.size ();
    latest_pointclouds_.resize (num_lidars_);

    for (size_t i = 0; i < num_lidars_; ++i) {
        pointcloud2_subscribers_.push_back (this->create_subscription<sensor_msgs::msg::PointCloud2> (lidar_topics[i], rclcpp::SensorDataQoS (), [this, i] (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            if (merge_mode_ == merge_mode_t::passthrough) {
                auto footprint_points_tf = get_transformed_footprint ();
                auto filtered_points     = transform_and_filter_points (*msg, footprint_points_tf);
                if (filtered_points.empty ()) {
                    return;
                }
                pointcloud2_publisher_->publish (build_pointcloud2 (filtered_points, frame_id_, msg->header.stamp));
                return;
            }

            std::lock_guard<std::mutex> lock (mutex_);
            latest_pointclouds_[i] = *msg;
        }));
    }

    frame_id_        = this->declare_parameter<std::string> ("frame_id", "pointcloud2_frame");
    double frequency = this->declare_parameter<double> ("frequency", 40.0);

    RCLCPP_INFO (this->get_logger (), "pointcloud2_merger node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frame_id: %s", frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency);
    RCLCPP_INFO (this->get_logger (), "mode: %s", merge_mode.c_str ());
    for (size_t i = 0; i < num_lidars_; ++i) {
        RCLCPP_INFO (this->get_logger (), "Subscribed to LIDAR topic[%zu]: %s", i, lidar_topics[i].c_str ());
    }

    if (merge_mode_ == merge_mode_t::merge) {
        publish_timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&pointcloud2_merger::publish_pointcloud2, this));
    }
}

void pointcloud2_merger::footprint_callback (const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock (mutex_);
    footprint_ = *msg;
}

bool pointcloud2_merger::check_footprint (double x, double y, const std::vector<geometry_msgs::msg::Point32> &footprint_points_tf) const {
    if (footprint_points_tf.empty ()) return false;

    bool   inside = false;
    size_t n      = footprint_points_tf.size ();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const auto &pi        = footprint_points_tf[i];
        const auto &pj        = footprint_points_tf[j];
        bool        intersect = ((pi.y > y) != (pj.y > y)) && (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y + 1e-9) + pi.x);
        if (intersect) inside = !inside;
    }
    return inside;
}

std::vector<geometry_msgs::msg::Point32> pointcloud2_merger::get_transformed_footprint () const {
    geometry_msgs::msg::PolygonStamped footprint_copy;
    {
        std::lock_guard<std::mutex> lock (mutex_);
        footprint_copy = footprint_;
    }

    std::vector<geometry_msgs::msg::Point32> footprint_points_tf;
    if (footprint_copy.polygon.points.empty ()) {
        return footprint_points_tf;
    }

    try {
        auto tf_poly = tf_buffer_->lookupTransform (frame_id_, footprint_copy.header.frame_id, footprint_copy.header.stamp);

        for (const auto &pt : footprint_copy.polygon.points) {
            geometry_msgs::msg::PointStamped p_in, p_out;
            p_in.header.frame_id = footprint_copy.header.frame_id;
            p_in.point.x         = pt.x;
            p_in.point.y         = pt.y;
            p_in.point.z         = pt.z;
            tf2::doTransform (p_in, p_out, tf_poly);

            geometry_msgs::msg::Point32 p_tf;
            p_tf.x = static_cast<float> (p_out.point.x);
            p_tf.y = static_cast<float> (p_out.point.y);
            p_tf.z = static_cast<float> (p_out.point.z);
            footprint_points_tf.push_back (p_tf);
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 1000, "TF transform for footprint failed: %s", ex.what ());
        footprint_points_tf.clear ();
    }

    return footprint_points_tf;
}

std::vector<std::array<float, 3>> pointcloud2_merger::transform_and_filter_points (const sensor_msgs::msg::PointCloud2 &pc, const std::vector<geometry_msgs::msg::Point32> &footprint_points_tf) const {
    std::vector<std::array<float, 3>> points;

    sensor_msgs::msg::PointCloud2 pc_tf;
    try {
        auto tf = tf_buffer_->lookupTransform (frame_id_, pc.header.frame_id, pc.header.stamp);
        tf2::doTransform (pc, pc_tf, tf);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 1000, "TF transform failed: %s", ex.what ());
        return points;
    }

    sensor_msgs::PointCloud2ConstIterator<float> it_x (pc_tf, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y (pc_tf, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z (pc_tf, "z");

    for (; it_x != it_x.end (); ++it_x, ++it_y, ++it_z) {
        if (check_footprint (*it_x, *it_y, footprint_points_tf)) {
            continue;
        }
        points.push_back ({*it_x, *it_y, *it_z});
    }

    return points;
}

sensor_msgs::msg::PointCloud2 pointcloud2_merger::build_pointcloud2 (const std::vector<std::array<float, 3>> &points, const std::string &frame_id, const rclcpp::Time &stamp) const {
    sensor_msgs::msg::PointCloud2 output;
    output.header.stamp    = stamp;
    output.header.frame_id = frame_id;
    output.height          = 1;
    output.width           = static_cast<uint32_t> (points.size ());
    output.is_bigendian    = false;
    output.is_dense        = true;

    sensor_msgs::PointCloud2Modifier modifier (output);
    modifier.setPointCloud2FieldsByString (1, "xyz");
    modifier.resize (points.size ());

    sensor_msgs::PointCloud2Iterator<float> it_out_x (output, "x");
    sensor_msgs::PointCloud2Iterator<float> it_out_y (output, "y");
    sensor_msgs::PointCloud2Iterator<float> it_out_z (output, "z");

    for (const auto &p : points) {
        *it_out_x = p[0];
        *it_out_y = p[1];
        *it_out_z = p[2];
        ++it_out_x;
        ++it_out_y;
        ++it_out_z;
    }

    return output;
}

void pointcloud2_merger::publish_pointcloud2 () {
    std::vector<sensor_msgs::msg::PointCloud2> pointclouds;
    {
        std::lock_guard<std::mutex> lock (mutex_);
        pointclouds = latest_pointclouds_;
    }

    bool available = false;
    for (const auto &pc : pointclouds) {
        if (!pc.data.empty ()) {
            available = true;
            break;
        }
    }
    if (!available) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 1000, "No pointclouds received yet.");
        return;
    }

    auto                              footprint_points_tf = get_transformed_footprint ();
    std::vector<std::array<float, 3>> merged_points;

    for (const auto &pc : pointclouds) {
        if (pc.data.empty ()) {
            continue;
        }

        auto filtered_points = transform_and_filter_points (pc, footprint_points_tf);
        merged_points.insert (merged_points.end (), filtered_points.begin (), filtered_points.end ());
    }

    pointcloud2_publisher_->publish (build_pointcloud2 (merged_points, frame_id_, this->get_clock ()->now ()));
}

}  // namespace pointcloud2_merger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (pointcloud2_merger::pointcloud2_merger)
