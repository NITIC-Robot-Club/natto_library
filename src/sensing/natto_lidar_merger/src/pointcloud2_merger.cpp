
#include "natto_lidar_merger/pointcloud2_merger.hpp"

namespace pointcloud2_merger {

pointcloud2_merger::pointcloud2_merger (const rclcpp::NodeOptions &node_options) : Node ("pointcloud2_merger", node_options) {
    pointcloud2_publisher_         = this->create_publisher<sensor_msgs::msg::PointCloud2> ("merged_pointcloud2", 10);
    pointcloud2_first_subscriber_  = this->create_subscription<sensor_msgs::msg::PointCloud2> ("pointcloud2_first", 10, std::bind (&pointcloud2_merger::pointcloud2_callback_first, this, std::placeholders::_1));
    pointcloud2_second_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2> ("pointcloud2_second", 10, std::bind (&pointcloud2_merger::pointcloud2_callback_second, this, std::placeholders::_1));
    footprint_subscriber_          = this->create_subscription<geometry_msgs::msg::PolygonStamped> ("footprint", 10, std::bind (&pointcloud2_merger::footprint_callback, this, std::placeholders::_1));

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    frame_id_ = this->declare_parameter<std::string> ("frame_id", "pointcloud2_frame");

    RCLCPP_INFO (this->get_logger (), "pointcloud2_merger node has been started.");
    RCLCPP_INFO (this->get_logger (), "frame id: %s", frame_id_.c_str ());
}

void pointcloud2_merger::pointcloud2_callback_first (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    first_pointcloud2_ = *msg;
    publish_pointcloud2 ();
}

void pointcloud2_merger::pointcloud2_callback_second (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    second_pointcloud2_ = *msg;
    publish_pointcloud2 ();
}

void pointcloud2_merger::footprint_callback (const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    footprint_ = *msg;
}

void pointcloud2_merger::publish_pointcloud2 () {
    if (first_pointcloud2_.data.empty () || second_pointcloud2_.data.empty ()) return;

    sensor_msgs::msg::PointCloud2 first_tf, second_tf;
    try {
        auto tf_first = tf_buffer_->lookupTransform (frame_id_, first_pointcloud2_.header.frame_id, first_pointcloud2_.header.stamp);
        tf2::doTransform (first_pointcloud2_, first_tf, tf_first);

        // 2つ目のLiDAR
        auto tf_second = tf_buffer_->lookupTransform (frame_id_, second_pointcloud2_.header.frame_id, second_pointcloud2_.header.stamp);
        tf2::doTransform (second_pointcloud2_, second_tf, tf_second);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN (this->get_logger (), "TF transform failed: %s", ex.what ());
        return;
    }
    std::vector<std::array<float, 3>> merged_points;

    auto extract_points = [&] (const sensor_msgs::msg::PointCloud2 &cloud) {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x (cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y (cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z (cloud, "z");
        for (; iter_x != iter_x.end (); ++iter_x, ++iter_y, ++iter_z) {
            merged_points.push_back ({*iter_x, *iter_y, *iter_z});
        }
    };

    extract_points (first_tf);
    extract_points (second_tf);

    std::vector<geometry_msgs::msg::Point32> footprint_points_tf;

    if (!footprint_.polygon.points.empty ()) {
        try {
            geometry_msgs::msg::TransformStamped tf_poly = tf_buffer_->lookupTransform (frame_id_, footprint_.header.frame_id, footprint_.header.stamp);

            for (const auto &pt : footprint_.polygon.points) {
                geometry_msgs::msg::PointStamped p_in, p_out;
                p_in.header.frame_id = footprint_.header.frame_id;
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

    sensor_msgs::msg::PointCloud2 output;
    output.header.stamp    = this->get_clock ()->now ();
    output.header.frame_id = frame_id_;
    output.height          = 1;
    output.width           = filtered.size ();
    output.is_bigendian    = false;
    output.is_dense        = true;

    sensor_msgs::PointCloud2Modifier modifier (output);
    modifier.setPointCloud2FieldsByString (1, "xyz");
    modifier.resize (filtered.size ());

    sensor_msgs::PointCloud2Iterator<float> out_x (output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y (output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z (output, "z");

    for (auto &p : filtered) {
        *out_x = p[0];
        *out_y = p[1];
        *out_z = p[2];
        ++out_x;
        ++out_y;
        ++out_z;
    }

    // --- publish ---
    pointcloud2_publisher_->publish (output);
}

}  // namespace pointcloud2_merger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (pointcloud2_merger::pointcloud2_merger)