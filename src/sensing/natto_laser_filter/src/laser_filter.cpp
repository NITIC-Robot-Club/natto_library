#include "natto_laser_filter/laser_filter.hpp"

#include <cmath>
#include <limits>

namespace laser_filter {

laser_filter::laser_filter (const rclcpp::NodeOptions &node_options) : Node ("laser_filter", node_options) {
    threshold_ = this->declare_parameter<double> ("threshold", 0.9);

    publisher_  = this->create_publisher<sensor_msgs::msg::LaserScan> ("output", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan> ("input", 10, std::bind (&laser_filter::scan_callback, this, std::placeholders::_1));
}

void laser_filter::scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto  out    = *msg;
    auto &ranges = out.ranges;

    const size_t n         = ranges.size ();
    const double angle_min = msg->angle_min;
    const double angle_inc = msg->angle_increment;

    for (size_t i = 0; i + 1 < n; ++i) {
        double r1 = ranges[i], r2 = ranges[i + 1];
        if (!std::isfinite (r1) || !std::isfinite (r2)) continue;

        double th1 = angle_min + i * angle_inc;
        double th2 = angle_min + (i + 1) * angle_inc;

        double Ax = r1 * std::cos (th1), Ay = r1 * std::sin (th1);
        double Bx = r2 * std::cos (th2), By = r2 * std::sin (th2);

        double vx = Bx - Ax, vy = By - Ay;
        double mx = (Ax + Bx) * 0.5, my = (Ay + By) * 0.5;

        double vnorm = std::hypot (vx, vy);
        double mnorm = std::hypot (mx, my);
        if (vnorm == 0.0 || mnorm == 0.0) continue;

        double dot = std::abs ((vx * mx + vy * my) / (vnorm * mnorm));

        if (dot > threshold_) {
            ranges[i] = std::numeric_limits<float>::quiet_NaN ();
        }
    }

    publisher_->publish (out);
}

}  // namespace laser_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (laser_filter::laser_filter)
