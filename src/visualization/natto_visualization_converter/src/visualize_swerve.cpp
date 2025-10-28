#include "natto_visualization_converter/visualize_swerve.hpp"

namespace visualize_swerve {

visualize_swerve::visualize_swerve (const rclcpp::NodeOptions &node_options) : Node ("visualize_swerve", node_options) {
    marker_publisher_     = this->create_publisher<visualization_msgs::msg::MarkerArray> ("marker_array", 10);
    swerve_subscription_  = this->create_subscription<natto_msgs::msg::Swerve> ("swerve", 10, std::bind (&visualize_swerve::swerve_callback, this, std::placeholders::_1));
    int publish_period_ms = this->declare_parameter<int> ("publish_period_ms", 10);
    timer_                = this->create_wall_timer (std::chrono::milliseconds (publish_period_ms), std::bind (&visualize_swerve::timer_callback, this));

    arrow_r        = this->declare_parameter<double> ("arrow_r", 0.0);
    arrow_g        = this->declare_parameter<double> ("arrow_g", 1.0);
    arrow_b        = this->declare_parameter<double> ("arrow_b", 0.0);
    arrow_scale    = this->declare_parameter<double> ("arrow_scale", 0.2);
    arrow_min_size = this->declare_parameter<double> ("arrow_min_size", 0.1);

    wheel_position_x = this->declare_parameter<std::vector<double>> ("wheel_position_x", {0.5, -0.5, -0.5, 0.5});
    wheel_position_y = this->declare_parameter<std::vector<double>> ("wheel_position_y", {0.5, 0.5, -0.5, -0.5});

    num_wheels_ = wheel_position_x.size ();
    if (wheel_position_y.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_position_x and wheel_position_y must have the same size.");
        throw std::runtime_error ("wheel_position_x and wheel_position_y must have the same size.");
    }

    RCLCPP_INFO (this->get_logger (), "visualize_swerve node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "Publish period (ms): %d", publish_period_ms);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %d", num_wheels_);
    RCLCPP_INFO (this->get_logger (), "Arrow color: (%.2f, %.2f, %.2f)", arrow_r, arrow_g, arrow_b);
    RCLCPP_INFO (this->get_logger (), "Arrow scale: %.2f", arrow_scale);
    for (int i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "Wheel %d position: (%.2f, %.2f)", i, wheel_position_x[i], wheel_position_y[i]);
    }
}

void visualize_swerve::swerve_callback (const natto_msgs::msg::Swerve::SharedPtr msg) {
    marker_array_.markers.clear ();
    for (int i = 0; i < num_wheels_; i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp    = this->now ();
        marker.ns              = "swerve_wheel";
        marker.id              = i;
        marker.type            = visualization_msgs::msg::Marker::ARROW;
        marker.action          = visualization_msgs::msg::Marker::ADD;

        // Set the pose of the marker
        double wheel_x     = wheel_position_x[i];
        double wheel_y     = wheel_position_y[i];
        double wheel_angle = msg->wheel_angle[i];
        double wheel_speed = msg->wheel_speed[i];

        if (std::signbit (wheel_speed)) {
            wheel_angle += M_PI;
        }

        wheel_speed = std::abs (wheel_speed);

        marker.pose.position.x = wheel_x;
        marker.pose.position.y = wheel_y;
        marker.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY (0, 0, wheel_angle);
        marker.pose.orientation.x = q.x ();
        marker.pose.orientation.y = q.y ();
        marker.pose.orientation.z = q.z ();
        marker.pose.orientation.w = q.w ();

        marker.scale.x = std::max (arrow_scale * wheel_speed, arrow_min_size);
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.r = arrow_r;
        marker.color.g = arrow_g;
        marker.color.b = arrow_b;
        marker.color.a = 1.0f;

        marker_array_.markers.push_back (marker);
    }
}

void visualize_swerve::timer_callback () {
    marker_publisher_->publish (marker_array_);
}

}  // namespace visualize_swerve

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (visualize_swerve::visualize_swerve)