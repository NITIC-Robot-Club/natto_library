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

namespace pointcloud2_merger {

pointcloud2_merger::pointcloud2_merger (const rclcpp::NodeOptions &node_options) : Node ("pointcloud2_merger", node_options) {
    pointcloud2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2> ("merged_pointcloud2", rclcpp::SensorDataQoS ());
    footprint_subscriber_  = this->create_subscription<geometry_msgs::msg::PolygonStamped> ("footprint", 1, std::bind (&pointcloud2_merger::footprint_callback, this, std::placeholders::_1));

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    std::vector<std::string> lidar_topics = this->declare_parameter<std::vector<std::string>> ("lidar_topics", {"/lidar1_pointcloud2", "/lidar2_pointcloud2"});

    num_lidars_ = lidar_topics.size ();
    latest_pointclouds_.resize (num_lidars_);

    for (size_t i = 0; i < num_lidars_; ++i) {
        pointcloud2_subscribers_.push_back (this->create_subscription<sensor_msgs::msg::PointCloud2> (lidar_topics[i], rclcpp::SensorDataQoS (), [this, i] (const sensor_msgs::msg::PointCloud2::SharedPtr msg) { latest_pointclouds_[i] = *msg; }));
    }

    frame_id_        = this->declare_parameter<std::string> ("frame_id", "pointcloud2_frame");
    double frequency = this->declare_parameter<double> ("frequency", 40.0);

    RCLCPP_INFO (this->get_logger (), "pointcloud2_merger node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frame_id: %s", frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency);
    for (size_t i = 0; i < num_lidars_; ++i) {
        RCLCPP_INFO (this->get_logger (), "Subscribed to LIDAR topic[%zu]: %s", i, lidar_topics[i].c_str ());
    }

    publish_timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&pointcloud2_merger::publish_pointcloud2, this));
}

void pointcloud2_merger::footprint_callback (const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    footprint_ = *msg;
}
void pointcloud2_merger::publish_pointcloud2 () {
    bool available = false;
    for (auto &pc : latest_pointclouds_) {
        if (!pc.data.empty ()) {
            available = true;
            break;
        }
    }
    if (!available) {
        RCLCPP_WARN (this->get_logger (), "No pointclouds received yet.");
        return;
    }

    std::vector<std::array<float, 3>> merged_points;

    for (auto &pc : latest_pointclouds_) {
        if (pc.data.empty ()) continue;

        sensor_msgs::msg::PointCloud2 pc_tf;
        try {
            auto tf = tf_buffer_->lookupTransform (frame_id_, pc.header.frame_id, pc.header.stamp);
            tf2::doTransform (pc, pc_tf, tf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN (this->get_logger (), "TF transform failed: %s", ex.what ());
            continue;
        }

        sensor_msgs::PointCloud2ConstIterator<float> it_x (pc_tf, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y (pc_tf, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z (pc_tf, "z");

        for (; it_x != it_x.end (); ++it_x, ++it_y, ++it_z) {
            merged_points.push_back ({*it_x, *it_y, *it_z});
        }
    }

    std::vector<geometry_msgs::msg::Point32> footprint_points_tf;
    if (!footprint_.polygon.points.empty ()) {
        try {
            auto tf_poly = tf_buffer_->lookupTransform (frame_id_, footprint_.header.frame_id, footprint_.header.stamp);

            for (const auto &pt : footprint_.polygon.points) {
                geometry_msgs::msg::PointStamped p_in, p_out;
                p_in.header.frame_id = footprint_.header.frame_id;
                p_in.point.x         = pt.x;
                p_in.point.y         = pt.y;
                p_in.point.z         = pt.z;
                tf2::doTransform (p_in, p_out, tf_poly);

                geometry_msgs::msg::Point32 p_tf;
                p_tf.x = (float)p_out.point.x;
                p_tf.y = (float)p_out.point.y;
                p_tf.z = (float)p_out.point.z;
                footprint_points_tf.push_back (p_tf);
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN (this->get_logger (), "TF transform for footprint failed: %s", ex.what ());
            footprint_points_tf.clear ();
        }
    }

    auto is_inside = [&] (float x, float y) {
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
    };

    std::vector<std::array<float, 3>> filtered;
    filtered.reserve (merged_points.size ());
    for (auto &p : merged_points) {
        if (!is_inside (p[0], p[1])) filtered.push_back (p);
    }

    // --- PointCloud2生成 ---
    sensor_msgs::msg::PointCloud2 output;
    output.header.stamp    = this->get_clock ()->now ();
    output.header.frame_id = frame_id_;
    output.height          = 1;
    output.width           = static_cast<uint32_t> (filtered.size ());
    output.is_bigendian    = false;
    output.is_dense        = true;

    sensor_msgs::PointCloud2Modifier modifier (output);
    modifier.setPointCloud2FieldsByString (1, "xyz");
    modifier.resize (filtered.size ());

    sensor_msgs::PointCloud2Iterator<float> it_out_x (output, "x");
    sensor_msgs::PointCloud2Iterator<float> it_out_y (output, "y");
    sensor_msgs::PointCloud2Iterator<float> it_out_z (output, "z");

    for (auto &p : filtered) {
        *it_out_x = p[0];
        *it_out_y = p[1];
        *it_out_z = p[2];
        ++it_out_x;
        ++it_out_y;
        ++it_out_z;
    }

    pointcloud2_publisher_->publish (output);
}

}  // namespace pointcloud2_merger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (pointcloud2_merger::pointcloud2_merger)