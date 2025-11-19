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

#include "natto_map/map_converter.hpp"

#include <algorithm>
#include <cmath>

namespace map_converter {

map_converter::map_converter (const rclcpp::NodeOptions &node_options) : Node ("map_converter", node_options) {
    occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid> ("occupancy_grid", rclcpp::QoS (rclcpp::KeepLast (1)).transient_local ().reliable ());
    map_subscription_         = this->create_subscription<natto_msgs::msg::Map> ("map", rclcpp::QoS (rclcpp::KeepLast (1)).transient_local ().reliable (), std::bind (&map_converter::map_callback, this, std::placeholders::_1));

    resolution_ = this->declare_parameter<double> ("resolution", 0.05);

    RCLCPP_INFO (this->get_logger (), "map_converter node has been started.");
    RCLCPP_INFO (this->get_logger (), "resolution: %f", resolution_);
}

void map_converter::map_callback (const natto_msgs::msg::Map::SharedPtr msg) {
    double min_x = std::numeric_limits<double>::max ();
    double max_x = std::numeric_limits<double>::lowest ();
    double min_y = std::numeric_limits<double>::max ();
    double max_y = std::numeric_limits<double>::lowest ();

    for (const auto &seg : msg->line_segments.line_segments) {
        min_x = std::min ({min_x, seg.start.x, seg.end.x});
        max_x = std::max ({max_x, seg.start.x, seg.end.x});
        min_y = std::min ({min_y, seg.start.y, seg.end.y});
        max_y = std::max ({max_y, seg.start.y, seg.end.y});
    }

    for (const auto &circ : msg->circles.circles) {
        double r     = circ.radius;
        double start = circ.start_angle;
        double end   = circ.end_angle;

        const int steps = 50;
        for (int i = 0; i <= steps; i++) {
            double angle = start + (end - start) * i / steps;
            double x     = circ.center.x + r * cos (angle);
            double y     = circ.center.y + r * sin (angle);
            min_x        = std::min (min_x, x);
            max_x        = std::max (max_x, x);
            min_y        = std::min (min_y, y);
            max_y        = std::max (max_y, y);
        }
    }

    int width  = static_cast<int> ((max_x - min_x) / resolution_) + 1;
    int height = static_cast<int> ((max_y - min_y) / resolution_) + 1;

    std::vector<int8_t> grid (width * height, 0);

    auto world_to_grid = [&] (double x, double y, int &gx, int &gy) {
        gx = static_cast<int> ((x - min_x) / resolution_);
        gy = static_cast<int> ((y - min_y) / resolution_);
    };

    auto set_cell = [&] (int gx, int gy) {
        if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
            grid[gy * width + gx] = 100;
        }
    };

    for (const auto &seg : msg->line_segments.line_segments) {
        if (seg.start.z != 0.0 && seg.end.z != 0.0) continue;
        int x0, y0, x1, y1;
        world_to_grid (seg.start.x, seg.start.y, x0, y0);
        world_to_grid (seg.end.x, seg.end.y, x1, y1);

        int dx  = std::abs (x1 - x0);
        int dy  = std::abs (y1 - y0);
        int sx  = (x0 < x1) ? 1 : -1;
        int sy  = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;
        while (true) {
            set_cell (x, y);
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }

    for (const auto &circ : msg->circles.circles) {
        if (circ.center.z != 0.0) continue;
        int steps = std::max (10, static_cast<int> (circ.radius / resolution_ * 8));

        int gx_prev = 0, gy_prev = 0;
        for (int i = 0; i <= steps; i++) {
            double angle = circ.start_angle + (circ.end_angle - circ.start_angle) * i / steps;
            double x     = circ.center.x + circ.radius * cos (angle);
            double y     = circ.center.y + circ.radius * sin (angle);

            int gx, gy;
            world_to_grid (x, y, gx, gy);

            if (i > 0) {
                int dx     = std::abs (gx - gx_prev);
                int dy     = std::abs (gy - gy_prev);
                int sx     = (gx_prev < gx) ? 1 : -1;
                int sy     = (gy_prev < gy) ? 1 : -1;
                int err    = dx - dy;
                int x_line = gx_prev;
                int y_line = gy_prev;
                while (true) {
                    set_cell (x_line, y_line);
                    if (x_line == gx && y_line == gy) break;
                    int e2 = 2 * err;
                    if (e2 > -dy) {
                        err -= dy;
                        x_line += sx;
                    }
                    if (e2 < dx) {
                        err += dx;
                        y_line += sy;
                    }
                }
            }
            gx_prev = gx;
            gy_prev = gy;
        }

        const double two_pi                   = 2.0 * M_PI;
        auto         normalize_angle_positive = [&] (double angle) {
            double result = std::fmod (angle, two_pi);
            if (result < 0.0) result += two_pi;
            return result;
        };
        double span             = circ.end_angle - circ.start_angle;
        double span_abs         = std::fabs (span);
        bool   fill_full_circle = (span_abs < 1e-6) || (span_abs >= (two_pi - 1e-6));
        double min_x_circ       = circ.center.x - circ.radius;
        double max_x_circ       = circ.center.x + circ.radius;
        double min_y_circ       = circ.center.y - circ.radius;
        double max_y_circ       = circ.center.y + circ.radius;
        int    gx_min           = std::max (0, static_cast<int> (std::floor ((min_x_circ - min_x) / resolution_)));
        int    gx_max           = std::min (width - 1, static_cast<int> (std::ceil ((max_x_circ - min_x) / resolution_)));
        int    gy_min           = std::max (0, static_cast<int> (std::floor ((min_y_circ - min_y) / resolution_)));
        int    gy_max           = std::min (height - 1, static_cast<int> (std::ceil ((max_y_circ - min_y) / resolution_)));
        double radius_squared   = circ.radius * circ.radius;

        for (int gx = gx_min; gx <= gx_max; ++gx) {
            double wx = min_x + (gx + 0.5) * resolution_;
            for (int gy = gy_min; gy <= gy_max; ++gy) {
                double wy          = min_y + (gy + 0.5) * resolution_;
                double dx          = wx - circ.center.x;
                double dy          = wy - circ.center.y;
                double distance_sq = dx * dx + dy * dy;
                if (distance_sq > radius_squared + 1e-9) continue;

                bool in_sector = fill_full_circle;
                if (!in_sector) {
                    double theta = std::atan2 (dy, dx);
                    if (span > 0.0) {
                        double diff = normalize_angle_positive (theta - circ.start_angle);
                        in_sector   = diff <= span;
                    } else {
                        double diff = normalize_angle_positive (circ.start_angle - theta);
                        in_sector   = diff <= -span;
                    }
                }

                if (in_sector) {
                    set_cell (gx, gy);
                }
            }
        }
    }

    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp              = this->now ();
    occupancy_grid.header.frame_id           = "map";
    occupancy_grid.info.resolution           = resolution_;
    occupancy_grid.info.width                = width;
    occupancy_grid.info.height               = height;
    occupancy_grid.info.origin.position.x    = min_x;
    occupancy_grid.info.origin.position.y    = min_y;
    occupancy_grid.info.origin.position.z    = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.data                      = grid;

    occupancy_grid_publisher_->publish (occupancy_grid);
}

}  // namespace map_converter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (map_converter::map_converter)
