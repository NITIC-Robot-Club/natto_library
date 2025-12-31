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

#include "natto_astar_planner/astar_planner.hpp"

namespace astar_planner {

astar_planner::astar_planner (const rclcpp::NodeOptions &node_options) : Node ("astar_planner", node_options) {
    path_publisher_    = this->create_publisher<nav_msgs::msg::Path> ("path", 10);
    costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid> ("costmap", rclcpp::QoS (rclcpp::KeepLast (1)).transient_local ().reliable ());
    occupancy_grid_subscription_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid> ("occupancy_grid", rclcpp::QoS (rclcpp::KeepLast (1)).transient_local ().reliable (), std::bind (&astar_planner::occupancy_grid_callback, this, std::placeholders::_1));
    goal_pose_subscription_    = this->create_subscription<geometry_msgs::msg::PoseStamped> ("goal_pose", 10, std::bind (&astar_planner::goal_pose_callback, this, std::placeholders::_1));
    current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped> ("current_pose", 10, std::bind (&astar_planner::current_pose_callback, this, std::placeholders::_1));
    footprint_subscription_    = this->create_subscription<geometry_msgs::msg::PolygonStamped> ("footprint", 10, std::bind (&astar_planner::footprint_callback, this, std::placeholders::_1));

    theta_resolution_deg_ = static_cast<int> (this->declare_parameter<int> ("theta_resolution_deg", 5));
    xy_inflation_         = this->declare_parameter<double> ("xy_inflation", 0.5);
    xy_offset_            = this->declare_parameter<double> ("xy_offset", 0.1);
    yaw_offset_           = this->declare_parameter<double> ("yaw_offset", 0.1);
    grad_alpha_           = this->declare_parameter<double> ("grad_alpha", 1.0);
    grad_beta_            = this->declare_parameter<double> ("grad_beta", 8.0);
    grad_gamma_           = this->declare_parameter<double> ("grad_gamma", 0.0);
    grad_step_size_       = this->declare_parameter<double> ("grad_step_size", 0.1);
    replan_distance_threshold_ = this->declare_parameter<double> ("replan_distance_threshold", 0.5);

    // Create timer for periodic path validation (100ms = 10Hz)
    replan_timer_ = this->create_wall_timer (
        std::chrono::milliseconds (100),
        std::bind (&astar_planner::replan_timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "astar_planner node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "theta_resolution_deg: %d", theta_resolution_deg_);
    RCLCPP_INFO (this->get_logger (), "xy_inflation: %.2f", xy_inflation_);
    RCLCPP_INFO (this->get_logger (), "xy_offset: %.2f", xy_offset_);
    RCLCPP_INFO (this->get_logger (), "yaw_offset: %.2f", yaw_offset_);
    RCLCPP_INFO (this->get_logger (), "grad_alpha: %.2f", grad_alpha_);
    RCLCPP_INFO (this->get_logger (), "grad_beta: %.2f", grad_beta_);
    RCLCPP_INFO (this->get_logger (), "grad_gamma: %.2f", grad_gamma_);
    RCLCPP_INFO (this->get_logger (), "grad_step_size: %.2f", grad_step_size_);
    RCLCPP_INFO (this->get_logger (), "replan_distance_threshold: %.2f", replan_distance_threshold_);
}

void astar_planner::occupancy_grid_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    raw_map_ = *msg;
    create_costmap ();
    create_obstacle_costmap ();
    create_path ();
    costmap_publisher_->publish (costmap_);
}

void astar_planner::goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Check if this is the same goal as before
    if (is_same_goal (*msg, previous_goal_pose_)) {
        // Check if robot is following the path (within threshold)
        double min_dist = calculate_min_distance_to_path ();
        if (min_dist <= replan_distance_threshold_) {
            RCLCPP_DEBUG (this->get_logger (), "Same goal received and robot is following path (dist: %.3f), skipping replanning", min_dist);
            return;
        }
    }

    goal_pose_ = *msg;
    previous_goal_pose_ = *msg;
    create_path ();
}

void astar_planner::current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void astar_planner::footprint_callback (const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    if (footprint_.polygon.points == msg->polygon.points) return;
    footprint_ = *msg;
    RCLCPP_INFO (this->get_logger (), "build footprint mask");
    build_footprint_mask ();
    if (!raw_map_.data.empty ()) {
        create_costmap ();
        create_obstacle_costmap ();
        create_path ();
    }
}

void astar_planner::create_path () {
    if (raw_map_.data.empty ()) return;
    if (goal_pose_.header.frame_id.empty ()) return;
    if (current_pose_.header.frame_id.empty ()) return;

    auto start_pose = find_free_space_pose (current_pose_.pose);
    auto goal_pose  = find_free_space_pose (goal_pose_.pose);

    current_pose_.pose = start_pose;
    goal_pose_.pose    = goal_pose;

    auto linear_path = linear_astar ();
    if (linear_path.poses.empty ()) {
        RCLCPP_WARN (this->get_logger (), "linear_astar found no path");
        return;
    }
    auto linear_smoothed_path = linear_smoother (linear_path);

    auto angular_path = angular_astar (linear_smoothed_path);

    path_ = angular_smoother (angular_path);

    path_publisher_->publish (path_);
    costmap_publisher_->publish (costmap_);
}

void astar_planner::create_obstacle_costmap () {
    if (raw_map_.data.empty ()) return;
    obstacle_costmap_ = raw_map_;
    return;

    const int width  = static_cast<int> (raw_map_.info.width);
    const int height = static_cast<int> (raw_map_.info.height);

    if (width == 0 || height == 0) return;

    std::vector<bool> visited (static_cast<size_t> (width * height), false);
    std::queue<int>   frontier;

    auto enqueue = [&] (int nx, int ny) {
        if (nx < 0 || nx >= width) return;
        if (ny < 0 || ny >= height) return;
        size_t nidx = static_cast<size_t> (ny * width + nx);
        if (visited[nidx]) return;
        if (raw_map_.data[nidx] == 100) return;
        visited[nidx] = true;
        frontier.push (static_cast<int> (nidx));
    };
    for (int x = 0; x < width; ++x) {
        enqueue (x, 0);
        enqueue (x, height - 1);
    }
    for (int y = 0; y < height; ++y) {
        enqueue (0, y);
        enqueue (width - 1, y);
    }
    while (!frontier.empty ()) {
        int idx = frontier.front ();
        frontier.pop ();
        obstacle_costmap_.data[static_cast<size_t> (idx)] = 100;

        int cx = idx % width;
        int cy = idx / width;
        enqueue (cx + 1, cy);
        enqueue (cx - 1, cy);
        enqueue (cx, cy + 1);
        enqueue (cx, cy - 1);
    }
}

void astar_planner::create_costmap () {
    if (raw_map_.data.empty ()) return;
    const int   width           = static_cast<int> (raw_map_.info.width);
    const int   height          = static_cast<int> (raw_map_.info.height);
    const float resolution      = static_cast<float> (raw_map_.info.resolution);
    const float offset_m        = static_cast<float> (xy_offset_);
    const float inflation_m     = static_cast<float> (xy_inflation_);
    const int   max_radius_cell = static_cast<int> (std::ceil ((inflation_m + offset_m) / resolution));

    costmap_ = raw_map_;
    costmap_.data.assign (static_cast<size_t> (width * height), 0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (raw_map_.data[static_cast<size_t> (y * width + x)] <= 50) continue;
            for (int dy = -max_radius_cell; dy <= max_radius_cell; ++dy) {
                int ny = y + dy;
                if (ny < 0 || ny >= height) continue;
                for (int dx = -max_radius_cell; dx <= max_radius_cell; ++dx) {
                    int nx = x + dx;
                    if (nx < 0 || nx >= width) continue;

                    float dist  = std::hypotf (static_cast<float> (dx) * resolution, static_cast<float> (dy) * resolution);
                    float inner = inflation_m;
                    float outer = inner + offset_m;

                    int8_t new_cost = 0;
                    if (dist <= inflation_m) {
                        new_cost = 100;
                    } else if (dist <= outer) {
                        float ratio = (outer - dist) / (outer - inner);
                        new_cost    = static_cast<int8_t> (ratio * 100.0f);
                    } else {
                        continue;
                    }

                    size_t idx = static_cast<size_t> (ny * width + nx);
                    if (new_cost > costmap_.data[idx]) costmap_.data[idx] = new_cost;
                }
            }
        }
    }
    costmap_publisher_->publish (costmap_);
}

void astar_planner::build_footprint_mask () {
    if (footprint_.polygon.points.empty ()) return;
    float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
    for (auto &p : footprint_.polygon.points) {
        minx = std::min (minx, (float)p.x);
        maxx = std::max (maxx, (float)p.x);
        miny = std::min (miny, (float)p.y);
        maxy = std::max (maxy, (float)p.y);
    }
    footprint_mask_w_ = static_cast<size_t> ((maxx - minx) / raw_map_.info.resolution + 3);
    footprint_mask_h_ = static_cast<size_t> ((maxy - miny) / raw_map_.info.resolution + 3);
    footprint_mask_.assign (footprint_mask_w_ * footprint_mask_h_, 0);
    const size_t       N = footprint_.polygon.points.size ();
    std::vector<float> px (N), py (N);
    for (size_t i = 0; i < N; ++i) {
        px[i] = footprint_.polygon.points[i].x;
        py[i] = footprint_.polygon.points[i].y;
    }

    std::vector<float> interx;
    interx.reserve (N);

    for (size_t y = 0; y < footprint_mask_h_; ++y) {
        float wy = miny + (static_cast<float> (y) + 0.5f) * raw_map_.info.resolution;
        interx.clear ();
        for (size_t i = 0; i < N; ++i) {
            size_t j = (i + 1) % N;
            if ((py[i] <= wy && py[j] > wy) || (py[j] <= wy && py[i] > wy)) {
                float t = (wy - py[i]) / (py[j] - py[i]);
                interx.push_back (px[i] + t * (px[j] - px[i]));
            }
        }
        for (size_t i = 1; i < interx.size (); ++i) {
            float  key = interx[i];
            size_t j   = i;
            while (j > 0 && interx[j - 1] > key) {
                interx[j] = interx[j - 1];
                --j;
            }
            interx[j] = key;
        }
        for (size_t k = 0; k + 1 < interx.size (); k += 2) {
            size_t x_start = static_cast<size_t> ((interx[k] - minx) / raw_map_.info.resolution);
            size_t x_end   = static_cast<size_t> ((interx[k + 1] - minx) / raw_map_.info.resolution);
            x_start        = std::clamp<size_t> (x_start, 0, footprint_mask_w_ - 1);
            x_end          = std::clamp<size_t> (x_end, 0, footprint_mask_w_ - 1);
            for (size_t x = x_start; x <= x_end; ++x) footprint_mask_[y * footprint_mask_w_ + x] = 1;
        }
    }
}

bool astar_planner::rectangle_is_collision_free (const size_t cx, const size_t cy, const double yaw) {
    if (obstacle_costmap_.data.empty ()) return true;

    const size_t width  = static_cast<size_t> (obstacle_costmap_.info.width);
    const size_t height = static_cast<size_t> (obstacle_costmap_.info.height);
    const double res    = obstacle_costmap_.info.resolution;
    const double ox     = obstacle_costmap_.info.origin.position.x;
    const double oy     = obstacle_costmap_.info.origin.position.y;

    if (width == 0 || height == 0) return false;
    if (cx >= width || cy >= height) return false;

    auto is_blocked = [&] (size_t idx) {
        if (idx >= obstacle_costmap_.data.size ()) return true;
        return obstacle_costmap_.data[idx] == 100;
    };

    size_t center_idx = cy * width + cx;
    if (is_blocked (center_idx)) return false;

    if (footprint_.polygon.points.size () < 4) {
        return true;
    }

    double wx = ox + (static_cast<double> (cx) + 0.5) * res;
    double wy = oy + (static_cast<double> (cy) + 0.5) * res;

    double yaw_sin = sin (yaw);
    double yaw_cos = cos (yaw);

    for (const auto &vertex : footprint_.polygon.points) {
        double vx = wx + vertex.x * yaw_cos - vertex.y * yaw_sin;
        double vy = wy + vertex.x * yaw_sin + vertex.y * yaw_cos;

        size_t gx = static_cast<size_t> ((vx - ox) / res);
        size_t gy = static_cast<size_t> ((vy - oy) / res);

        if (gx >= width || gy >= height) return false;

        size_t vidx = gy * width + gx;
        if (is_blocked (vidx)) return false;
    }

    return true;
}

bool astar_planner::rectangle_is_collision_free (const geometry_msgs::msg::Pose &pose) {
    const double yaw = tf2::getYaw (pose.orientation);
    auto         c   = to_grid (pose.position.x, pose.position.y);
    return rectangle_is_collision_free (c.first, c.second, yaw);
}

nav_msgs::msg::Path astar_planner::linear_astar () {
    nav_msgs::msg::Path path;
    const size_t        width  = static_cast<size_t> (raw_map_.info.width);
    const size_t        height = static_cast<size_t> (raw_map_.info.height);
    const double        res    = raw_map_.info.resolution;
    const double        ox     = raw_map_.info.origin.position.x;
    const double        oy     = raw_map_.info.origin.position.y;

    const auto start_grid = to_grid (current_pose_.pose.position.x, current_pose_.pose.position.y);
    size_t     sx         = start_grid.first;
    size_t     sy         = start_grid.second;
    const auto goal_grid  = to_grid (goal_pose_.pose.position.x, goal_pose_.pose.position.y);
    size_t     gx         = goal_grid.first;
    size_t     gy         = goal_grid.second;

    if (sx >= width || sy >= height || gx >= width || gy >= height) return path;
    if (costmap_.data[static_cast<size_t> (gy * width + gx)] == 100) {
        RCLCPP_WARN (this->get_logger (), "goal in occupied cell");
        return path;
    }

    struct AstarNode {
        size_t x, y;
        double g, f;
    };
    struct Cmp {
        bool operator() (const AstarNode &a, const AstarNode &b) const {
            return a.f > b.f;
        }
    };
    std::priority_queue<AstarNode, std::vector<AstarNode>, Cmp> open;

    std::vector<double> gscore (static_cast<size_t> (width * height), std::numeric_limits<double>::infinity ());
    std::vector<int>    came_from (static_cast<size_t> (width * height), -1);
    auto                heur = [&] (size_t x, size_t y) {
        const double dx = static_cast<double> (gx) - static_cast<double> (x);
        const double dy = static_cast<double> (gy) - static_cast<double> (y);
        return std::hypot (dx, dy);
    };

    open.push ({sx, sy, 0.0, heur (sx, sy)});
    gscore[static_cast<size_t> (sy * width + sx)] = 0.0;

    const std::vector<std::pair<int, int>> dirs = {
        { 1,  0},
        {-1,  0},
        { 0,  1},
        { 0, -1},
        { 1,  1},
        { 1, -1},
        {-1,  1},
        {-1, -1}
    };

    while (!open.empty ()) {
        AstarNode cur = open.top ();
        open.pop ();
        if (cur.x == gx && cur.y == gy) break;
        if (cur.g > gscore[static_cast<size_t> (cur.x + cur.y * width)]) continue;

        for (auto [dx, dy] : dirs) {
            int nx_i = static_cast<int> (cur.x) + dx;
            int ny_i = static_cast<int> (cur.y) + dy;
            if (nx_i < 0 || ny_i < 0) continue;
            size_t nx = static_cast<size_t> (nx_i);
            size_t ny = static_cast<size_t> (ny_i);
            if (nx >= width || ny >= height) continue;
            if (costmap_.data[static_cast<size_t> (nx + ny * width)] == 100) continue;

            double tentative = cur.g + std::hypot (dx, dy) + static_cast<double> (costmap_.data[static_cast<size_t> (nx + ny * width)]) / 100.0;
            if (tentative + 1e-9 < gscore[static_cast<size_t> (nx + ny * width)]) {
                gscore[static_cast<size_t> (nx + ny * width)]    = tentative;
                came_from[static_cast<size_t> (nx + ny * width)] = static_cast<int> (cur.x + cur.y * width);
                open.push ({nx, ny, tentative, tentative + heur (nx, ny)});
            }
        }
    }

    size_t cur_idx = static_cast<size_t> (gx + gy * width);
    if (came_from[cur_idx] == -1) {
        return path;
    }
    std::vector<std::pair<size_t, size_t>> rev;
    while (!(cur_idx == static_cast<size_t> (sx + sy * width))) {
        size_t cx = cur_idx % width;
        size_t cy = cur_idx / width;
        rev.emplace_back (cx, cy);
        cur_idx = static_cast<size_t> (came_from[cur_idx]);
    }
    rev.emplace_back (sx, sy);
    std::reverse (rev.begin (), rev.end ());
    path.header.frame_id = raw_map_.header.frame_id;
    path.header.stamp    = this->now ();
    for (auto &p : rev) {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x = ox + (static_cast<double> (p.first) + 0.5) * res;
        ps.pose.position.y = oy + (static_cast<double> (p.second) + 0.5) * res;
        path.poses.push_back (ps);
    }

    return path;
}

nav_msgs::msg::Path astar_planner::linear_smoother (const nav_msgs::msg::Path &linear_path) {
    if (linear_path.poses.size () < 3) return linear_path;
    nav_msgs::msg::Path path     = linear_path;
    auto                get_cost = [&] (size_t gx, size_t gy) -> double {
        if (gx >= costmap_.info.width || gy >= costmap_.info.height) return 1.0;
        return static_cast<double> (costmap_.data[gy * costmap_.info.width + gx]) / 100.0;
    };
    auto get_cost_grad = [&] (double x, double y) -> std::pair<double, double> {
        auto [gx, gy] = to_grid (x, y);
        double c_x    = get_cost (gx + 1, gy) - get_cost (gx - 1, gy);
        double c_y    = get_cost (gx, gy + 1) - get_cost (gx, gy - 1);
        return {c_x / 2.0, c_y / 2.0};
    };

    const int max_iter = 300;
    for (int iter = 0; iter < max_iter; ++iter) {
        for (size_t i = 1; i + 1 < path.poses.size (); ++i) {
            auto &p_prev    = path.poses[i - 1].pose.position;
            auto &p         = path.poses[i].pose.position;
            auto &p_next    = path.poses[i + 1].pose.position;
            auto [cgx, cgy] = get_cost_grad (p.x, p.y);
            double smooth_x = p.x - 0.5 * (p_prev.x + p_next.x);
            double smooth_y = p.y - 0.5 * (p_prev.y + p_next.y);
            double anchor_x = p.x - linear_path.poses[i].pose.position.x;
            double anchor_y = p.y - linear_path.poses[i].pose.position.y;
            double total_x  = cgx * grad_alpha_ + smooth_x * grad_beta_ + anchor_x * grad_gamma_;
            double total_y  = cgy * grad_alpha_ + smooth_y * grad_beta_ + anchor_y * grad_gamma_;
            path.poses[i].pose.position.x -= total_x * grad_step_size_;
            path.poses[i].pose.position.y -= total_y * grad_step_size_;

            geometry_msgs::msg::Pose probe = path.poses[i].pose;
            probe.orientation              = path.poses[i].pose.orientation;
        }
    }
    return path;
}

nav_msgs::msg::Path astar_planner::angular_astar (const nav_msgs::msg::Path &linear_smoothed_path) {
    nav_msgs::msg::Path out;
    if (linear_smoothed_path.poses.empty ()) return out;

    const size_t N         = linear_smoothed_path.poses.size ();
    const size_t num_theta = std::max<size_t> (1, static_cast<size_t> (360.0 / theta_resolution_deg_));

    auto wrap_theta = [&] (int t) -> size_t {
        const int nt = static_cast<int> (num_theta);
        while (t < 0) t += nt;
        while (t >= nt) t -= nt;
        return static_cast<size_t> (t);
    };

    auto to_index = [&] (size_t ix, size_t th) -> size_t { return th * N + ix; };

    std::vector<std::vector<int8_t>> angle_cost (N, std::vector<int8_t> (num_theta, 0));

    for (size_t i = 0; i < N; ++i) {
        for (size_t t = 0; t < num_theta; ++t) {
            double yaw = (static_cast<double> (t) + 0.5) * theta_resolution_deg_ * M_PI / 180.0;

            geometry_msgs::msg::Pose probe = linear_smoothed_path.poses[i].pose;

            probe.orientation.z = std::sin (yaw * 0.5);
            probe.orientation.w = std::cos (yaw * 0.5);

            angle_cost[i][t] = rectangle_is_collision_free (probe) ? 0 : 100;
        }
    }

    auto deg_to_th = [&] (double yaw_rad) -> size_t {
        double deg = yaw_rad * 180.0 / M_PI;
        deg        = std::fmod (deg, 360.0);
        if (deg < 0.0) deg += 360.0;
        return static_cast<size_t> (std::round (deg / theta_resolution_deg_)) % num_theta;
    };

    const size_t start_theta = deg_to_th (tf2::getYaw (current_pose_.pose.orientation));
    const size_t goal_theta  = deg_to_th (tf2::getYaw (goal_pose_.pose.orientation));

    struct AstarNode {
        size_t ix;
        size_t th;
        double g;
        double f;
    };

    struct Cmp {
        bool operator() (const AstarNode &a, const AstarNode &b) const {
            return a.f > b.f;
        }
    };

    const double rot_cost   = 0.2;
    const double trans_cost = 1.0;

    std::priority_queue<AstarNode, std::vector<AstarNode>, Cmp> open;

    std::vector<double> gscore (N * num_theta, std::numeric_limits<double>::infinity ());
    std::vector<size_t> came_from (N * num_theta, static_cast<size_t> (-1));

    auto ang_dist = [&] (size_t a, size_t b) {
        size_t d = (a > b) ? (a - b) : (b - a);
        return std::min (d, num_theta - d);
    };

    auto heuristic = [&] (size_t ix, size_t th) {
        double h_rot   = rot_cost * static_cast<double> (ang_dist (th, goal_theta));
        double h_trans = trans_cost * static_cast<double> ((N - 1) - ix);
        return h_rot + h_trans;
    };

    const size_t start_idx = to_index (0, start_theta);
    gscore[start_idx]      = 0.0;

    open.push ({0, start_theta, 0.0, heuristic (0, start_theta)});

    const std::vector<int> rot_steps = {-2, -1, 1, 2};

    while (!open.empty ()) {
        AstarNode cur = open.top ();
        open.pop ();

        const size_t cur_idx = to_index (cur.ix, cur.th);
        if (cur.g > gscore[cur_idx]) continue;
        if (cur.ix == N - 1 && cur.th == goal_theta) break;

        for (int dth : rot_steps) {
            size_t nth = wrap_theta (static_cast<int> (cur.th) + dth);

            if (angle_cost[cur.ix][nth] > 50) continue;

            double ng = cur.g + rot_cost * std::abs (dth);
            size_t ni = to_index (cur.ix, nth);

            if (ng + 1e-9 < gscore[ni]) {
                gscore[ni]    = ng;
                came_from[ni] = cur_idx;
                open.push ({cur.ix, nth, ng, ng + heuristic (cur.ix, nth)});
            }
        }

        size_t nx = cur.ix + 1;
        if (nx < N && angle_cost[nx][cur.th] <= 50) {
            double ng = cur.g + trans_cost;
            size_t ni = to_index (nx, cur.th);

            if (ng + 1e-9 < gscore[ni]) {
                gscore[ni]    = ng;
                came_from[ni] = cur_idx;
                open.push ({nx, cur.th, ng, ng + heuristic (nx, cur.th)});
            }
        }
    }
    const size_t goal_idx = to_index (N - 1, goal_theta);

    if (came_from[goal_idx] == static_cast<size_t> (-1)) {
        RCLCPP_WARN (this->get_logger (), "angular_astar failed");
        return out;
    }

    size_t yawcur = goal_idx;

    if (came_from[goal_idx] == static_cast<size_t> (-1)) {
        RCLCPP_WARN (this->get_logger (), "angular_astar failed");
        return out;
    }

    std::vector<std::pair<size_t, size_t>> seq;

    while (yawcur != start_idx) {
        size_t ix = yawcur % N;
        size_t th = yawcur / N;
        seq.emplace_back (ix, th);
        yawcur = came_from[yawcur];
    }
    seq.emplace_back (0, start_theta);
    std::reverse (seq.begin (), seq.end ());

    out.header       = linear_smoothed_path.header;
    out.header.stamp = this->now ();

    for (auto &[ix, th] : seq) {
        geometry_msgs::msg::PoseStamped ps = linear_smoothed_path.poses[ix];

        double yaw = (static_cast<double> (th) + 0.5) * theta_resolution_deg_ * M_PI / 180.0;

        ps.pose.orientation.z = std::sin (yaw * 0.5);
        ps.pose.orientation.w = std::cos (yaw * 0.5);
        out.poses.push_back (ps);
    }

    return out;
}

nav_msgs::msg::Path astar_planner::angular_smoother (const nav_msgs::msg::Path &angular_path) {
    if (angular_path.poses.size () < 3) return angular_path;
    size_t              N = angular_path.poses.size ();
    std::vector<double> thetas (N);
    std::vector<double> orig (N);
    for (size_t i = 0; i < N; ++i) {
        double yaw = tf2::getYaw (angular_path.poses[i].pose.orientation);
        orig[i]    = yaw;
        thetas[i]  = yaw;
    }
    const int max_iter = 1000;
    for (int it = 0; it < max_iter; ++it) {
        for (size_t i = 1; i + 1 < N; ++i) {
            double a    = thetas[i - 1];
            double b    = thetas[i + 1];
            double mx   = std::cos (a) + std::cos (b);
            double my   = std::sin (a) + std::sin (b);
            double mid  = std::atan2 (my, mx);
            double diff = thetas[i] - mid;
            diff        = fix_angle (diff);

            double new_theta = thetas[i] - diff * 0.5;

            geometry_msgs::msg::Pose probe = angular_path.poses[i].pose;
            probe.orientation.z            = std::sin (new_theta / 2.0);
            probe.orientation.w            = std::cos (new_theta / 2.0);

            if (!rectangle_is_collision_free (probe)) {
                new_theta = orig[i];
            }

            thetas[i] = fix_angle (new_theta);
        }
    }
    nav_msgs::msg::Path out = angular_path;
    for (size_t i = 0; i < N; ++i) {
        out.poses[i].pose.orientation.z = std::sin (thetas[i] / 2.0);
        out.poses[i].pose.orientation.w = std::cos (thetas[i] / 2.0);
    }
    return out;
}

geometry_msgs::msg::Pose astar_planner::find_free_space_pose (const geometry_msgs::msg::Pose &pose) {
    if (obstacle_costmap_.data.empty ()) return pose;

    const size_t width  = static_cast<size_t> (obstacle_costmap_.info.width);
    const size_t height = static_cast<size_t> (obstacle_costmap_.info.height);
    const double res    = obstacle_costmap_.info.resolution;
    const double ox     = obstacle_costmap_.info.origin.position.x;
    const double oy     = obstacle_costmap_.info.origin.position.y;

    if (width <= 0 || height <= 0) return pose;

    double                   best_dist_sq = std::numeric_limits<double>::infinity ();
    geometry_msgs::msg::Pose best_pose    = pose;

    const double yaw = tf2::getYaw (pose.orientation);

    for (size_t cy = 0; cy < height; ++cy) {
        for (size_t cx = 0; cx < width; ++cx) {
            size_t idx = cy * width + cx;
            if (idx >= obstacle_costmap_.data.size ()) continue;
            if (obstacle_costmap_.data[idx] == 100) continue;

            if (!rectangle_is_collision_free (cx, cy, yaw)) continue;

            double wx = ox + (static_cast<double> (cx) + 0.5) * res;
            double wy = oy + (static_cast<double> (cy) + 0.5) * res;

            double dx      = wx - pose.position.x;
            double dy      = wy - pose.position.y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < best_dist_sq) {
                best_dist_sq         = dist_sq;
                best_pose.position.x = wx;
                best_pose.position.y = wy;
            }
        }
    }

    if (std::isfinite (best_dist_sq)) {
        return best_pose;
    }

    RCLCPP_WARN (this->get_logger (), "No collision-free space found in obstacle_costmap_ for the given pose.");
    return pose;
}

std::pair<size_t, size_t> astar_planner::to_grid (double x, double y) {
    size_t gx = static_cast<size_t> ((x - raw_map_.info.origin.position.x) / raw_map_.info.resolution);
    size_t gy = static_cast<size_t> ((y - raw_map_.info.origin.position.y) / raw_map_.info.resolution);
    return std::make_pair (gx, gy);
}

double astar_planner::fix_angle (double angle) {
    while (angle > +M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double astar_planner::calculate_min_distance_to_path () {
    // Return infinity if path is empty or current pose is not set
    if (path_.poses.empty () || current_pose_.header.frame_id.empty ()) {
        return std::numeric_limits<double>::infinity ();
    }

    double min_distance = std::numeric_limits<double>::infinity ();
    double current_x = current_pose_.pose.position.x;
    double current_y = current_pose_.pose.position.y;

    // Loop through all path poses to find minimum distance
    for (const auto &pose : path_.poses) {
        double dx = pose.pose.position.x - current_x;
        double dy = pose.pose.position.y - current_y;
        double distance = std::hypot (dx, dy);

        if (distance < min_distance) {
            min_distance = distance;
        }
    }

    return min_distance;
}

bool astar_planner::is_same_goal (const geometry_msgs::msg::PoseStamped &goal1, const geometry_msgs::msg::PoseStamped &goal2, double tolerance) {
    // If either goal is empty, they are not the same
    if (goal1.header.frame_id.empty () || goal2.header.frame_id.empty ()) {
        return false;
    }

    // Check if frame_ids match
    if (goal1.header.frame_id != goal2.header.frame_id) {
        return false;
    }

    // Calculate 2D distance between the two goals (consistent with calculate_min_distance_to_path)
    double dx = goal1.pose.position.x - goal2.pose.position.x;
    double dy = goal1.pose.position.y - goal2.pose.position.y;
    double distance = std::hypot (dx, dy);

    return distance < tolerance;
}

void astar_planner::replan_timer_callback () {
    // Only check if we have a valid goal and path
    if (goal_pose_.header.frame_id.empty () || path_.poses.empty ()) {
        return;
    }

    // Calculate minimum distance from current position to path
    double min_distance = calculate_min_distance_to_path ();

    // If distance exceeds threshold, trigger replanning
    if (min_distance > replan_distance_threshold_) {
        RCLCPP_INFO (this->get_logger (), "Distance to path (%.3f m) exceeds threshold (%.3f m), replanning...", min_distance, replan_distance_threshold_);
        create_path ();
    }
}

}  // namespace astar_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (astar_planner::astar_planner)
