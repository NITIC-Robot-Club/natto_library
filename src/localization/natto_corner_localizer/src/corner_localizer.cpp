#include "natto_corner_localizer/corner_localizer.hpp"

namespace corner_localizer {
corner_localizer::corner_localizer (const rclcpp::NodeOptions &node_options) : Node ("corner_localizer", node_options) {
    pose_publisher_        = this->create_publisher<geometry_msgs::msg::PoseStamped> ("pose", 10);
    debug_coner_publisher_ = this->create_publisher<natto_msgs::msg::CornerArray> ("debug_corners", 10);
    corner_subscriber_     = this->create_subscription<natto_msgs::msg::CornerArray> ("corners", 10, std::bind (&corner_localizer::corner_callback, this, std::placeholders::_1));
    map_subscriber_        = this->create_subscription<natto_msgs::msg::Map> ("map", 10, std::bind (&corner_localizer::map_callback, this, std::placeholders::_1));

    tf_buffer_      = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_    = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster> (this);

    odom_frame_id_ = this->declare_parameter<std::string> ("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter<std::string> ("base_frame_id", "base_link");
    map_frame_id_  = this->declare_parameter<std::string> ("map_frame_id", "map");

    double initial_x   = this->declare_parameter<double> ("initial_x", 1.0);
    double initial_y   = this->declare_parameter<double> ("initial_y", 1.0);
    double initial_yaw = this->declare_parameter<double> ("initial_yaw", 0.0);

    RCLCPP_INFO (this->get_logger (), "Corner Localizer Node has been started.");
    RCLCPP_INFO (this->get_logger (), "odom_frame_id: %s", odom_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "base_frame_id: %s", base_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "map_frame_id: %s", map_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "initial_x: %f", initial_x);
    RCLCPP_INFO (this->get_logger (), "initial_y: %f", initial_y);
    RCLCPP_INFO (this->get_logger (), "initial_yaw: %f", initial_yaw);

    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, initial_yaw);
    last_map_to_odom_.header.frame_id         = map_frame_id_;
    last_map_to_odom_.child_frame_id          = odom_frame_id_;
    last_map_to_odom_.header.stamp            = this->now ();
    last_map_to_odom_.transform.translation.x = initial_x;
    last_map_to_odom_.transform.translation.y = initial_y;
    last_map_to_odom_.transform.translation.z = 0.0;
    last_map_to_odom_.transform.rotation.x    = q.x ();
    last_map_to_odom_.transform.rotation.y    = q.y ();
    last_map_to_odom_.transform.rotation.z    = q.z ();
    last_map_to_odom_.transform.rotation.w    = q.w ();
}

void corner_localizer::corner_callback (const natto_msgs::msg::CornerArray::SharedPtr msg) {
    if (map_corners_.corners.empty ()) {
        RCLCPP_WARN (this->get_logger (), "Map data is not received yet.");
        return;
    }

    natto_msgs::msg::CornerArray         transformed_map_corners;
    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
        odom_to_base = tf_buffer_->lookupTransform (base_frame_id_, odom_frame_id_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN (this->get_logger (), "Transform error: %s", ex.what ());
        return;
    }

    for (const auto &map_corner : map_corners_.corners) {
        geometry_msgs::msg::PointStamped corner_in_map;
        corner_in_map.header.frame_id = map_frame_id_;
        corner_in_map.header.stamp    = this->now ();
        corner_in_map.point           = map_corner.position;

        geometry_msgs::msg::PointStamped corner_in_odom;
        corner_in_odom.header.frame_id = odom_frame_id_;
        corner_in_odom.header.stamp    = this->now ();
        tf2::doTransform (corner_in_map, corner_in_odom, last_map_to_odom_);
        RCLCPP_INFO (this->get_logger (), "map : %.2f , odom %.2f, tf: %.2f", corner_in_map.point.x, corner_in_odom.point.x, last_map_to_odom_.transform.translation.x);

        geometry_msgs::msg::PointStamped corner_in_base;
        corner_in_base.header.frame_id = base_frame_id_;
        corner_in_base.header.stamp    = this->now ();
        tf2::doTransform (corner_in_odom, corner_in_base, odom_to_base);
        RCLCPP_INFO (this->get_logger (), "odom : %.2f , base : %.2f, tf: %.2f", corner_in_odom.point.x, corner_in_base.point.x, odom_to_base.transform.translation.x);

        return;
        natto_msgs::msg::Corner transformed_corner;
        transformed_corner.position = corner_in_base.point;

        double base_yaw         = tf2::getYaw (odom_to_base.transform.rotation) + tf2::getYaw (last_map_to_odom_.transform.rotation);
        transformed_corner.yaw1 = map_corner.yaw1 + base_yaw;
        transformed_corner.yaw2 = map_corner.yaw2 + base_yaw;

        transformed_map_corners.corners.push_back (transformed_corner);
    }
    debug_coner_publisher_->publish (transformed_map_corners);

    last_map_to_odom_.header.stamp = this->now ();
    tf_broadcaster_->sendTransform (last_map_to_odom_);

    std::vector<std::pair<natto_msgs::msg::Corner, natto_msgs::msg::Corner>> matched_corners;
    for (const auto &observed_corner : msg->corners) {
        for (const auto &map_corner : transformed_map_corners.corners) {
            if (is_same_corner (observed_corner, map_corner)) {
                matched_corners.push_back (std::make_pair (observed_corner, map_corner));
                break;
            }
        }
    }
    if (matched_corners.size () < 2) {
        // RCLCPP_WARN (this->get_logger (), "Not enough matched corners for localization.");
        return;
    }

    return;

    double mean_x_obs = 0.0, mean_y_obs = 0.0;
    double mean_x_map = 0.0, mean_y_map = 0.0;

    for (auto &pair : matched_corners) {
        mean_x_obs += pair.first.position.x;
        mean_y_obs += pair.first.position.y;
        mean_x_map += pair.second.position.x;
        mean_y_map += pair.second.position.y;
    }
    mean_x_obs /= matched_corners.size ();
    mean_y_obs /= matched_corners.size ();
    mean_x_map /= matched_corners.size ();
    mean_y_map /= matched_corners.size ();

    double Sxx = 0.0, Sxy = 0.0, Syx = 0.0, Syy = 0.0;
    for (auto &pair : matched_corners) {
        double xo = pair.first.position.x - mean_x_obs;
        double yo = pair.first.position.y - mean_y_obs;
        double xm = pair.second.position.x - mean_x_map;
        double ym = pair.second.position.y - mean_y_map;

        Sxx += xo * xm + yo * ym;
        Sxy += xo * ym - yo * xm;
    }

    double theta = std::atan2 (Sxy, Sxx);
    double cos_t = std::cos (theta);
    double sin_t = std::sin (theta);

    double tx = mean_x_map - (cos_t * mean_x_obs - sin_t * mean_y_obs);
    double ty = mean_y_map - (sin_t * mean_x_obs + cos_t * mean_y_obs);

    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, tf2::getYaw (last_map_to_odom_.transform.rotation) - theta);
    last_map_to_odom_.header.stamp = this->now ();
    last_map_to_odom_.transform.translation.x -= tx;
    last_map_to_odom_.transform.translation.y -= ty;
    last_map_to_odom_.transform.rotation.x = q.x ();
    last_map_to_odom_.transform.rotation.y = q.y ();
    last_map_to_odom_.transform.rotation.z = q.z ();
    last_map_to_odom_.transform.rotation.w = q.w ();

    tf_broadcaster_->sendTransform (last_map_to_odom_);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp       = this->now ();
    pose_msg.header.frame_id    = map_frame_id_;
    pose_msg.pose.position.x    = tx;
    pose_msg.pose.position.y    = ty;
    pose_msg.pose.position.z    = 0.0;
    pose_msg.pose.orientation.x = q.x ();
    pose_msg.pose.orientation.y = q.y ();
    pose_msg.pose.orientation.z = q.z ();
    pose_msg.pose.orientation.w = q.w ();
    pose_publisher_->publish (pose_msg);
}

void corner_localizer::map_callback (const natto_msgs::msg::Map::SharedPtr msg) {
    map_corners_.corners.clear ();
    for (size_t i = 0; i < msg->line_segments.line_segments.size (); ++i) {
        for (size_t j = i + 1; j < msg->line_segments.line_segments.size (); ++j) {
            double x1_start = msg->line_segments.line_segments[i].start.x;
            double y1_start = msg->line_segments.line_segments[i].start.y;
            double x1_end   = msg->line_segments.line_segments[i].end.x;
            double y1_end   = msg->line_segments.line_segments[i].end.y;
            double a1       = y1_end - y1_start;
            double b1       = -(x1_end - x1_start);
            double c1       = -(a1 * x1_start + b1 * y1_start);

            double x2_start = msg->line_segments.line_segments[j].start.x;
            double y2_start = msg->line_segments.line_segments[j].start.y;
            double x2_end   = msg->line_segments.line_segments[j].end.x;
            double y2_end   = msg->line_segments.line_segments[j].end.y;
            double a2       = y2_end - y2_start;
            double b2       = -(x2_end - x2_start);
            double c2       = -(a2 * x2_start + b2 * y2_start);

            double det = a1 * b2 - a2 * b1;
            if (std::fabs (det) < 1e-8) continue;  // 平行

            double x = (b1 * c2 - b2 * c1) / det;
            double y = (c1 * a2 - c2 * a1) / det;

            natto_msgs::msg::Corner corner;
            corner.position.x = x;
            corner.position.y = y;
            corner.position.z = 0.0;

            corner.yaw1 = std::atan2 (-a1, b1);
            corner.yaw2 = std::atan2 (-a2, b2);

            map_corners_.corners.push_back (corner);
        }
    }
}

bool corner_localizer::is_same_corner (natto_msgs::msg::Corner corner1, natto_msgs::msg::Corner corner2) {
    double position_threshold = 0.4;
    double angle_threshold    = 0.5;

    double dx       = corner1.position.x - corner2.position.x;
    double dy       = corner1.position.y - corner2.position.y;
    double distance = std::sqrt (dx * dx + dy * dy);
    if (distance > position_threshold) return false;

    double dyaw1 = std::fabs (corner1.yaw1 - corner2.yaw1);
    dyaw1        = std::fmod (dyaw1 + M_PI, 2.0 * M_PI) - M_PI;  // wrap to [-pi, pi]
    dyaw1        = std::fabs (dyaw1);

    double dyaw2 = std::fabs (corner1.yaw2 - corner2.yaw2);
    dyaw2        = std::fmod (dyaw2 + M_PI, 2.0 * M_PI) - M_PI;  // wrap to [-pi, pi]
    dyaw2        = std::fabs (dyaw2);

    // if (dyaw1 > angle_threshold || dyaw2 > angle_threshold) return false;
    return true;
}

}  // namespace corner_localizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (corner_localizer::corner_localizer)