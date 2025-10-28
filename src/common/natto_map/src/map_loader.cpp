#include "natto_map/map_loader.hpp"

namespace map_loader {
map_loader::map_loader (const rclcpp::NodeOptions &node_options) : Node ("map_loader", node_options) {
    std::string line_segments_path = this->declare_parameter<std::string> ("line_segments_path", "");
    std::string circles_path       = this->declare_parameter<std::string> ("circles_path", "");
    int         publish_period_ms  = this->declare_parameter<int> ("publish_period_ms", 1000);

    map_publisher_ = this->create_publisher<natto_msgs::msg::Map> ("map", 10);

    if (line_segments_path != "") {
        load_line_segments (line_segments_path);
    } else {
        RCLCPP_WARN (this->get_logger (), "No line segments path provided.");
    }

    if (circles_path != "") {
        load_circles (circles_path);
    } else {
        RCLCPP_WARN (this->get_logger (), "No circles path provided.");
    }

    timer_ = this->create_wall_timer (std::chrono::milliseconds (publish_period_ms), std::bind (&map_loader::timer_callback, this));
}

void map_loader::load_line_segments (const std::string &path) {
    std::ifstream file (path);
    if (!file.is_open ()) {
        RCLCPP_ERROR (this->get_logger (), "Failed to open line segments file: %s", path.c_str ());
        return;
    }

    std::string line;
    std::getline (file, line);

    while (std::getline (file, line)) {
        std::stringstream            ss (line);
        natto_msgs::msg::LineSegment seg;
        std::string                  val;

        std::getline (ss, val, ',');
        seg.start.x = std::stod (val);
        std::getline (ss, val, ',');
        seg.start.y = std::stod (val);
        std::getline (ss, val, ',');
        seg.start.z = std::stod (val);
        std::getline (ss, val, ',');
        seg.end.x = std::stod (val);
        std::getline (ss, val, ',');
        seg.end.y = std::stod (val);
        std::getline (ss, val, ',');
        seg.end.z = std::stod (val);

        map_.line_segments.push_back (seg);
    }
    RCLCPP_INFO (this->get_logger (), "Loaded %zu line segments from %s", map_.line_segments.size (), path.c_str ());
}

void map_loader::load_circles (const std::string &path) {
    std::ifstream file (path);
    if (!file.is_open ()) {
        RCLCPP_ERROR (this->get_logger (), "Failed to open circles file: %s", path.c_str ());
        return;
    }

    std::string line;
    std::getline (file, line);

    while (std::getline (file, line)) {
        std::stringstream       ss (line);
        natto_msgs::msg::Circle c;
        std::string             val;

        std::getline (ss, val, ',');
        c.center.x = std::stod (val);
        std::getline (ss, val, ',');
        c.center.y = std::stod (val);
        std::getline (ss, val, ',');
        c.center.z = std::stod (val);
        std::getline (ss, val, ',');
        c.radius = std::stod (val);
        std::getline (ss, val, ',');
        c.start_angle = std::stod (val);
        std::getline (ss, val, ',');
        c.end_angle = std::stod (val);
        map_.circles.push_back (c);
    }
    RCLCPP_INFO (this->get_logger (), "Loaded %zu circles from %s", map_.circles.size (), path.c_str ());
}

void map_loader::timer_callback () {
    map_publisher_->publish (map_);
}

}  // namespace map_loader

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (map_loader::map_loader)
