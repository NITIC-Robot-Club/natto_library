#include "natto_astar_planner/astar_planner.hpp"

namespace astar_planner {

astar_planner::astar_planner (const rclcpp::NodeOptions &node_options) : Node ("astar_planner", node_options) {
    path_publisher_              = this->create_publisher<nav_msgs::msg::Path> ("path", 10);
    costmap_publisher_           = this->create_publisher<nav_msgs::msg::OccupancyGrid> ("costmap", 10);
    occupancy_grid_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid> ("map", 10, std::bind (&astar_planner::occupancy_grid_callback, this, std::placeholders::_1));
    goal_pose_subscription_      = this->create_subscription<geometry_msgs::msg::PoseStamped> ("goal_pose", 10, std::bind (&astar_planner::goal_pose_callback, this, std::placeholders::_1));
    current_pose_subscription_   = this->create_subscription<geometry_msgs::msg::PoseStamped> ("current_pose", 10, std::bind (&astar_planner::current_pose_callback, this, std::placeholders::_1));
    footprint_subscription_      = this->create_subscription<geometry_msgs::msg::PolygonStamped> ("footprint", 10, std::bind (&astar_planner::footprint_callback, this, std::placeholders::_1));

    theta_resolution_deg_ = this->declare_parameter<int> ("theta_resolution_deg", 15);
    xy_inflation_         = this->declare_parameter<double> ("xy_inflation", 0.5);
    xy_offset_            = this->declare_parameter<double> ("xy_offset", 0.1);
    yaw_offset_           = this->declare_parameter<double> ("yaw_offset", 0.1);
    grad_alpha_           = this->declare_parameter<double> ("grad_alpha", 1.0);
    grad_beta_            = this->declare_parameter<double> ("grad_beta", 8.0);
    grad_gamma_           = this->declare_parameter<double> ("grad_gamma", 0.0);
    grad_step_size_       = this->declare_parameter<double> ("grad_step_size", 0.1);

    RCLCPP_INFO (this->get_logger (), "astar_planner node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "theta_resolution_deg: %d", theta_resolution_deg_);
    RCLCPP_INFO (this->get_logger (), "xy_inflation: %.2f", xy_inflation_);
    RCLCPP_INFO (this->get_logger (), "xy_offset: %.2f", xy_offset_);
    RCLCPP_INFO (this->get_logger (), "yaw_offset: %.2f", yaw_offset_);
    RCLCPP_INFO (this->get_logger (), "grad_alpha: %.2f", grad_alpha_);
    RCLCPP_INFO (this->get_logger (), "grad_beta: %.2f", grad_beta_);
    RCLCPP_INFO (this->get_logger (), "grad_gamma: %.2f", grad_gamma_);
    RCLCPP_INFO (this->get_logger (), "grad_step_size: %.2f", grad_step_size_);
}

void astar_planner::occupancy_grid_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (is_same_map (*msg)) return;
    raw_map_ = *msg;
    create_costmap ();
    create_path ();
}

void astar_planner::goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_pose_ = *msg;
    create_path ();
}

void astar_planner::current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void astar_planner::footprint_callback (const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    if (is_same_footprint (*msg)) return;
    footprint_ = *msg;
    RCLCPP_INFO (this->get_logger (), "build footprint mask");
    build_footprint_mask ();
    if (!raw_map_.data.empty ()) {
        create_costmap ();
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

void astar_planner::create_costmap () {
    if (raw_map_.data.empty ()) return;
    const int   width           = static_cast<int> (raw_map_.info.width);
    const int   height          = static_cast<int> (raw_map_.info.height);
    const float resolution      = static_cast<float> (raw_map_.info.resolution);
    const float offset_m        = static_cast<float> (xy_offset_);
    const float inflation_m     = static_cast<float> (xy_inflation_);
    const int   max_radius_cell = static_cast<int> (std::ceil ((inflation_m + offset_m) / resolution));

    costmap_ = raw_map_;
    costmap_.data.assign (width * height, 0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (raw_map_.data[y * width + x] <= 50) continue;
            for (int dy = -max_radius_cell; dy <= max_radius_cell; ++dy) {
                int ny = y + dy;
                if (ny < 0 || ny >= height) continue;
                for (int dx = -max_radius_cell; dx <= max_radius_cell; ++dx) {
                    int nx = x + dx;
                    if (nx < 0 || nx >= width) continue;

                    float dist  = std::hypotf (dx * resolution, dy * resolution);
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

                    int idx = ny * width + nx;
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
    footprint_mask_w_ = static_cast<int> ((maxx - minx) / raw_map_.info.resolution) + 3;
    footprint_mask_h_ = static_cast<int> ((maxy - miny) / raw_map_.info.resolution) + 3;
    footprint_mask_.assign (footprint_mask_w_ * footprint_mask_h_, 0);
    const size_t       N = footprint_.polygon.points.size ();
    std::vector<float> px (N), py (N);
    for (size_t i = 0; i < N; ++i) {
        px[i] = footprint_.polygon.points[i].x;
        py[i] = footprint_.polygon.points[i].y;
    }

    std::vector<float> interx;
    interx.reserve (N);

    for (int y = 0; y < footprint_mask_h_; ++y) {
        float wy = miny + (y + 0.5f) * raw_map_.info.resolution;
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
            int x_start = static_cast<int> ((interx[k] - minx) / raw_map_.info.resolution);
            int x_end   = static_cast<int> ((interx[k + 1] - minx) / raw_map_.info.resolution);
            x_start     = std::clamp (x_start, 0, footprint_mask_w_ - 1);
            x_end       = std::clamp (x_end, 0, footprint_mask_w_ - 1);
            for (int x = x_start; x <= x_end; ++x) footprint_mask_[y * footprint_mask_w_ + x] = 1;
        }
    }
}

bool astar_planner::is_same_map (nav_msgs::msg::OccupancyGrid latest_map) {
    if (raw_map_.data.size () != latest_map.data.size ()) return false;
    for (size_t i = 0; i < raw_map_.data.size (); ++i) {
        if (raw_map_.data[i] != latest_map.data[i]) return false;
    }
    return true;
}

bool astar_planner::is_same_footprint (geometry_msgs::msg::PolygonStamped latest_footprint) {
    if (footprint_.polygon.points.size () != latest_footprint.polygon.points.size ()) return false;
    for (size_t i = 0; i < footprint_.polygon.points.size (); ++i) {
        if (footprint_.polygon.points[i].x != latest_footprint.polygon.points[i].x) return false;
        if (footprint_.polygon.points[i].y != latest_footprint.polygon.points[i].y) return false;
    }
    return true;
}

bool astar_planner::point_in_polygon (double x, double y, geometry_msgs::msg::Polygon polygon) {
    bool   c = false;
    size_t n = polygon.points.size ();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double xi = polygon.points[i].x, yi = polygon.points[i].y;
        double xj = polygon.points[j].x, yj = polygon.points[j].y;
        bool   intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);
        if (intersect) c = !c;
    }
    return c;
}

bool astar_planner::check_collision (geometry_msgs::msg::Pose pose) {
    if (footprint_mask_.empty ()) {
        auto [gx, gy] = to_grid (pose.position.x, pose.position.y);
        if (gx < 0 || gy < 0 || gx >= static_cast<int> (raw_map_.info.width) || gy >= static_cast<int> (raw_map_.info.height)) return true;
        int idx = gy * raw_map_.info.width + gx;
        return raw_map_.data[idx] > 50;
    }

    const int    width  = static_cast<int> (raw_map_.info.width);
    const int    height = static_cast<int> (raw_map_.info.height);
    const double res    = raw_map_.info.resolution;
    const double ox     = raw_map_.info.origin.position.x;
    const double oy     = raw_map_.info.origin.position.y;

    double yaw = tf2::getYaw (pose.orientation);
    double cs  = std::cos (yaw);
    double sn  = std::sin (yaw);

    int cx = static_cast<int> ((pose.position.x - ox) / res);
    int cy = static_cast<int> ((pose.position.y - oy) / res);

    int hw = footprint_mask_w_ / 2;
    int hh = footprint_mask_h_ / 2;
    for (int my = 0; my < footprint_mask_h_; ++my) {
        for (int mx = 0; mx < footprint_mask_w_; ++mx) {
            if (footprint_mask_[my * footprint_mask_w_ + mx] == 0) continue;
            double lx = (mx - hw) * raw_map_.info.resolution;
            double ly = (my - hh) * raw_map_.info.resolution;
            double wx = pose.position.x + lx * cs - ly * sn;
            double wy = pose.position.y + lx * sn + ly * cs;

            int gx = static_cast<int> ((wx - ox) / res);
            int gy = static_cast<int> ((wy - oy) / res);

            if (gx < 0 || gy < 0 || gx >= width || gy >= height) return true;

            if (raw_map_.data[gy * width + gx] > 50) return true;
        }
    }
    return false;
}

nav_msgs::msg::Path astar_planner::linear_astar () {
    nav_msgs::msg::Path path;
    const int           width  = static_cast<int> (raw_map_.info.width);
    const int           height = static_cast<int> (raw_map_.info.height);
    const double        res    = raw_map_.info.resolution;
    const double        ox     = raw_map_.info.origin.position.x;
    const double        oy     = raw_map_.info.origin.position.y;

    auto [sx, sy] = to_grid (current_pose_.pose.position.x, current_pose_.pose.position.y);
    auto [gx, gy] = to_grid (goal_pose_.pose.position.x, goal_pose_.pose.position.y);

    auto idx = [&] (int x, int y) { return y * width + x; };

    if (sx < 0 || sy < 0 || gx < 0 || gy < 0 || sx >= width || sy >= height || gx >= width || gy >= height) return path;
    if (costmap_.data[idx (gx, gy)] == 100) {
        RCLCPP_WARN (this->get_logger (), "goal in occupied cell");
        return path;
    }

    struct Node {
        int    x, y;
        double g, f;
    };
    struct Cmp {
        bool operator() (const Node &a, const Node &b) const {
            return a.f > b.f;
        }
    };
    std::priority_queue<Node, std::vector<Node>, Cmp> open;

    std::vector<double> gscore (width * height, std::numeric_limits<double>::infinity ());
    std::vector<int>    came_from (width * height, -1);
    auto                heur = [&] (int x, int y) { return std::hypot (gx - x, gy - y); };

    open.push ({sx, sy, 0.0, heur (sx, sy)});
    gscore[idx (sx, sy)] = 0.0;

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
        Node cur = open.top ();
        open.pop ();
        if (cur.x == gx && cur.y == gy) break;
        if (cur.g > gscore[idx (cur.x, cur.y)]) continue;

        for (auto [dx, dy] : dirs) {
            int nx = cur.x + dx, ny = cur.y + dy;
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            if (costmap_.data[idx (nx, ny)] == 100) continue;

            double tentative = cur.g + std::hypot (dx, dy);
            if (tentative + 1e-9 < gscore[idx (nx, ny)]) {
                geometry_msgs::msg::Pose probe;
                probe.position.x  = ox + (nx + 0.5) * res;
                probe.position.y  = oy + (ny + 0.5) * res;
                probe.orientation = current_pose_.pose.orientation;
                if (check_collision (probe)) continue;

                gscore[idx (nx, ny)]    = tentative;
                came_from[idx (nx, ny)] = idx (cur.x, cur.y);
                open.push ({nx, ny, tentative, tentative + heur (nx, ny)});
            }
        }
    }

    int cur_idx = idx (gx, gy);
    if (came_from[cur_idx] == -1) {
        return path;
    }
    std::vector<std::pair<int, int>> rev;
    while (!(cur_idx == idx (sx, sy))) {
        int cx = cur_idx % width;
        int cy = cur_idx / width;
        rev.emplace_back (cx, cy);
        cur_idx = came_from[cur_idx];
    }
    rev.emplace_back (sx, sy);
    std::reverse (rev.begin (), rev.end ());
    path.header.frame_id = raw_map_.header.frame_id;
    path.header.stamp    = this->now ();
    for (auto &p : rev) {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x  = ox + (p.first + 0.5) * res;
        ps.pose.position.y  = oy + (p.second + 0.5) * res;
        ps.pose.orientation = geometry_msgs::msg::Quaternion ();
        path.poses.push_back (ps);
    }
    for (size_t i = 0; i + 1 < path.poses.size (); ++i) {
        double dx                        = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
        double dy                        = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
        double yaw                       = std::atan2 (dy, dx);
        path.poses[i].pose.orientation.z = std::sin (yaw / 2.0);
        path.poses[i].pose.orientation.w = std::cos (yaw / 2.0);
    }
    double goal_yaw                       = tf2::getYaw (goal_pose_.pose.orientation);
    path.poses.back ().pose.orientation.z = std::sin (goal_yaw / 2.0);
    path.poses.back ().pose.orientation.w = std::cos (goal_yaw / 2.0);

    return path;
}

nav_msgs::msg::Path astar_planner::linear_smoother (nav_msgs::msg::Path linear_path) {
    if (linear_path.poses.size () < 3) return linear_path;
    nav_msgs::msg::Path path     = linear_path;
    auto                get_cost = [&] (int gx, int gy) -> double {
        if (gx < 0 || gy < 0 || gx >= static_cast<int> (costmap_.info.width) || gy >= static_cast<int> (costmap_.info.height)) return 1.0;
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
            if (check_collision (probe)) {
                path.poses[i].pose.position = linear_path.poses[i].pose.position;
            }
        }
    }
    for (size_t i = 0; i + 1 < path.poses.size (); ++i) {
        double dx                        = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
        double dy                        = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
        double yaw                       = std::atan2 (dy, dx);
        path.poses[i].pose.orientation.z = std::sin (yaw / 2.0);
        path.poses[i].pose.orientation.w = std::cos (yaw / 2.0);
    }
    if (!path.poses.empty ()) {
        double goal_yaw                       = tf2::getYaw (goal_pose_.pose.orientation);
        path.poses.back ().pose.orientation.z = std::sin (goal_yaw / 2.0);
        path.poses.back ().pose.orientation.w = std::cos (goal_yaw / 2.0);
    }
    return path;
}

nav_msgs::msg::Path astar_planner::angular_astar (const nav_msgs::msg::Path linear_smoothed_path) {
    nav_msgs::msg::Path out;
    if (linear_smoothed_path.poses.empty ()) return out;
    const int N         = static_cast<int> (linear_smoothed_path.poses.size ());
    const int num_theta = std::max (1, 360 / theta_resolution_deg_);

    std::vector<std::vector<int8_t>> angle_cost (N, std::vector<int8_t> (num_theta, 0));
    for (int i = 0; i < N; ++i) {
        for (int t = 0; t < num_theta; ++t) {
            double                   yaw   = t * theta_resolution_deg_ * M_PI / 180.0;
            geometry_msgs::msg::Pose probe = linear_smoothed_path.poses[i].pose;
            probe.orientation.z            = std::sin (yaw / 2.0);
            probe.orientation.w            = std::cos (yaw / 2.0);
            if (check_collision (probe)) {
                angle_cost[i][t] = 100;
            } else {
                angle_cost[i][t] = 0;
            }
        }
    }

    auto to_index = [&] (int ix, int th) { return th * N + ix; };
    struct Node {
        int    ix, th;
        double g, f;
    };
    struct Cmp {
        bool operator() (const Node &a, const Node &b) const {
            return a.f > b.f;
        }
    };
    std::priority_queue<Node, std::vector<Node>, Cmp> open;
    std::vector<double>                               gscore (N * num_theta, std::numeric_limits<double>::infinity ());
    std::vector<int>                                  came_from (N * num_theta, -1);

    int start_theta = static_cast<int> (std::round ((tf2::getYaw (current_pose_.pose.orientation) * 180.0 / M_PI) / theta_resolution_deg_)) % num_theta;
    if (start_theta < 0) start_theta += num_theta;
    int goal_theta = static_cast<int> (std::round ((tf2::getYaw (goal_pose_.pose.orientation) * 180.0 / M_PI) / theta_resolution_deg_)) % num_theta;
    if (goal_theta < 0) goal_theta += num_theta;

    open.push ({0, start_theta, 0.0, 0.0});
    gscore[to_index (0, start_theta)] = 0.0;

    auto theta_cost = [&] (int dx, int dth) -> double {
        int dth_abs = std::abs (dth);
        if (dth_abs > num_theta / 2) dth_abs = num_theta - dth_abs;
        double angle_weight = 0.5;
        return std::hypot (dx, angle_weight * dth_abs);
    };

    std::vector<int> rotations;
    for (int r = -2; r <= 2; ++r) rotations.push_back (r);

    while (!open.empty ()) {
        Node cur = open.top ();
        open.pop ();
        if (cur.ix == N - 1 && cur.th == goal_theta) break;
        if (cur.g > gscore[to_index (cur.ix, cur.th)]) continue;

        for (int dx : {0, 1}) {
            for (int dth : rotations) {
                if (dx == 0 && dth == 0) continue;
                int nx  = cur.ix + dx;
                int nth = cur.th + dth;
                if (nx < 0 || nx >= N) continue;
                if (nth < 0) nth += num_theta;
                if (nth >= num_theta) nth -= num_theta;
                if (angle_cost[nx][nth] > 50) continue;

                double nc   = cur.g + theta_cost (dx, dth);
                int    tidx = to_index (nx, nth);
                if (nc + 1e-9 < gscore[tidx]) {
                    gscore[tidx]    = nc;
                    double priority = nc + theta_cost (N - 1 - nx, goal_theta - nth);
                    open.push ({nx, nth, nc, priority});
                    came_from[tidx] = to_index (cur.ix, cur.th);
                }
            }
        }
    }

    int cur = to_index (N - 1, goal_theta);
    if (came_from[cur] == -1) {
        RCLCPP_WARN (this->get_logger (), "angular_astar failed to find path");
        return out;
    }
    std::vector<std::pair<int, int>> seq;
    while (cur != to_index (0, start_theta)) {
        int ix = cur % N;
        int th = cur / N;
        seq.emplace_back (ix, th);
        cur = came_from[cur];
        if (cur == -1) break;
    }
    seq.emplace_back (0, start_theta);
    std::reverse (seq.begin (), seq.end ());

    out.header.frame_id = linear_smoothed_path.header.frame_id;
    out.header.stamp    = this->now ();
    for (auto &p : seq) {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose               = linear_smoothed_path.poses[p.first].pose;
        double yaw            = p.second * theta_resolution_deg_ * M_PI / 180.0;
        ps.pose.orientation.z = std::sin (yaw / 2.0);
        ps.pose.orientation.w = std::cos (yaw / 2.0);
        out.poses.push_back (ps);
    }
    return out;
}

nav_msgs::msg::Path astar_planner::angular_smoother (const nav_msgs::msg::Path angular_path) {
    if (angular_path.poses.size () < 3) return angular_path;
    int                 N         = angular_path.poses.size ();
    int                 num_theta = std::max (1, 360 / theta_resolution_deg_);
    std::vector<double> thetas (N);
    for (int i = 0; i < N; ++i) {
        double yaw = tf2::getYaw (angular_path.poses[i].pose.orientation);
        yaw        = wrap_to_2pi (yaw);
        thetas[i]  = yaw;
    }
    const int max_iter = 50;
    for (int it = 0; it < max_iter; ++it) {
        for (int i = 1; i + 1 < N; ++i) {
            double a    = thetas[i - 1];
            double b    = thetas[i + 1];
            double mx   = std::cos (a) + std::cos (b);
            double my   = std::sin (a) + std::sin (b);
            double mid  = std::atan2 (my, mx);
            double diff = thetas[i] - mid;

            if (diff > M_PI) diff -= 2 * M_PI;
            if (diff < -M_PI) diff += 2 * M_PI;
            thetas[i] -= diff * 0.5;
            thetas[i]                      = wrap_to_2pi (thetas[i]);
            geometry_msgs::msg::Pose probe = angular_path.poses[i].pose;
            probe.orientation.z            = std::sin (thetas[i] / 2.0);
            probe.orientation.w            = std::cos (thetas[i] / 2.0);
            if (check_collision (probe)) {
                thetas[i] = angular_path.poses[i].pose.orientation.w;
            }
        }
    }
    nav_msgs::msg::Path out = angular_path;
    for (int i = 0; i < N; ++i) {
        out.poses[i].pose.orientation.z = std::sin (thetas[i] / 2.0);
        out.poses[i].pose.orientation.w = std::cos (thetas[i] / 2.0);
    }
    return out;
}

geometry_msgs::msg::Pose astar_planner::find_free_space_pose (geometry_msgs::msg::Pose pose) {
    const int    width  = static_cast<int> (raw_map_.info.width);
    const int    height = static_cast<int> (raw_map_.info.height);
    const double res    = raw_map_.info.resolution;
    const double ox     = raw_map_.info.origin.position.x;
    const double oy     = raw_map_.info.origin.position.y;

    if (raw_map_.data.empty ()) return pose;
    auto [sx, sy] = to_grid (pose.position.x, pose.position.y);

    sx = std::clamp (sx, 0, width - 1);
    sy = std::clamp (sy, 0, height - 1);

    std::vector<std::vector<bool>>  visited (height, std::vector<bool> (width, false));
    std::queue<std::pair<int, int>> q;
    q.push ({sx, sy});
    visited[sy][sx] = true;

    const std::vector<std::pair<int, int>> directions = {
        { 1,  0},
        {-1,  0},
        { 0,  1},
        { 0, -1},
        { 1,  1},
        {-1, -1},
        { 1, -1},
        {-1,  1}
    };

    while (!q.empty ()) {
        auto [cx, cy] = q.front ();
        q.pop ();
        int idx = cy * width + cx;
        if (idx < 0 || idx >= static_cast<int> (costmap_.data.size ())) continue;
        if (costmap_.data[idx] < 100 && costmap_.data[idx] >= 0) {
            geometry_msgs::msg::Pose free_pose = pose;
            free_pose.position.x               = ox + (cx + 0.5) * res;
            free_pose.position.y               = oy + (cy + 0.5) * res;
            return free_pose;
        }
        for (auto [dx, dy] : directions) {
            int nx = cx + dx;
            int ny = cy + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            if (visited[ny][nx]) continue;
            visited[ny][nx] = true;
            q.push ({nx, ny});
        }
    }
    RCLCPP_WARN (this->get_logger (), "No free space found near the given pose.");
    return pose;
}

std::pair<int, int> astar_planner::to_grid (double x, double y) {
    int gx = static_cast<int> ((x - raw_map_.info.origin.position.x) / raw_map_.info.resolution);
    int gy = static_cast<int> ((y - raw_map_.info.origin.position.y) / raw_map_.info.resolution);
    return std::make_pair (gx, gy);
}

double astar_planner::wrap_to_2pi (double angle) {
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return angle;
}

}  // namespace astar_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (astar_planner::astar_planner)