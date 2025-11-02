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

#include "natto_ransac/line_detector.hpp"

namespace line_detector {

line_detector::line_detector (const rclcpp::NodeOptions &node_options) : Node ("line_detector", node_options) {
    lines_publisher_         = this->create_publisher<natto_msgs::msg::LineArray> ("lines", 10);
    line_segments_publisher_ = this->create_publisher<natto_msgs::msg::LineSegmentArray> ("line_segments", 10);
    corners_publisher_       = this->create_publisher<geometry_msgs::msg::PoseArray> ("corners", 10);

    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2> ("pointcloud2", 10, std::bind (&line_detector::pointcloud_callback, this, std::placeholders::_1));

    max_iterations_        = this->declare_parameter<int> ("max_iterations", 100);
    max_lines_             = this->declare_parameter<int> ("max_lines", 10);
    min_inliers_           = this->declare_parameter<int> ("min_inliers", 75);
    distance_threshold_    = this->declare_parameter<double> ("distance_threshold", 0.01);
    segment_gap_threshold_ = this->declare_parameter<double> ("segment_gap_threshold", 0.1);
}

void line_detector::pointcloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    raw_data_ = *msg;
    data_     = *msg;
    lines_.lines.clear ();

    for (int i = 0; i < max_lines_; i++) process_pointcloud ();

    calculate_corner ();
    calculate_line_segment ();

    corners_.header.frame_id = "base_link";
    corners_.header.stamp    = this->now ();
    lines_publisher_->publish (lines_);
    corners_publisher_->publish (corners_);
    line_segments_publisher_->publish (line_segments_);
}

void line_detector::process_pointcloud () {
    const uint32_t POINT_STEP = data_.point_step;
    const uint32_t WIDTH      = data_.width;
    const uint32_t HEIGHT     = data_.height;
    const uint8_t *raw        = data_.data.data ();

    std::vector<float> xs, ys;
    xs.reserve (WIDTH * HEIGHT);
    ys.reserve (WIDTH * HEIGHT);

    for (size_t i = 0; i < WIDTH * HEIGHT; i++) {
        float x = *reinterpret_cast<const float *> (raw + i * POINT_STEP + 0);
        float y = *reinterpret_cast<const float *> (raw + i * POINT_STEP + 4);
        if (std::isfinite (x) && std::isfinite (y)) {
            xs.push_back (x);
            ys.push_back (y);
        }
    }

    if (xs.size () < 2) return;

    std::mt19937                    gen (std::random_device{}());
    std::uniform_int_distribution<> dist (0, static_cast<int> (xs.size () - 1));

    int    best_inliers = 0;
    double best_a = 0.0, best_b = 0.0, best_c = 0.0;

    for (int iter = 0; iter < max_iterations_; iter++) {
        int i1 = dist (gen);
        int i2 = dist (gen);
        if (i1 == i2) continue;

        double x1 = xs[i1], y1 = ys[i1];
        double x2 = xs[i2], y2 = ys[i2];

        double a = y1 - y2;
        double b = x2 - x1;
        double c = x1 * y2 - x2 * y1;

        double norm = std::sqrt (a * a + b * b);
        if (norm == 0.0) continue;
        a /= norm;
        b /= norm;
        c /= norm;

        int inliers = 0;
        for (size_t j = 0; j < xs.size (); j++) {
            double d = std::fabs (a * xs[j] + b * ys[j] + c);
            if (d < distance_threshold_) inliers++;
        }

        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_a       = a;
            best_b       = b;
            best_c       = c;
        }
    }

    if (best_inliers > min_inliers_) {
        natto_msgs::msg::Line line_msg;
        line_msg.a = best_a;
        line_msg.b = best_b;
        line_msg.c = best_c;
        lines_.lines.push_back (line_msg);
    }

    // inlier削除
    std::vector<float> new_xs, new_ys;
    new_xs.reserve (xs.size ());
    new_ys.reserve (ys.size ());
    for (size_t j = 0; j < xs.size (); j++) {
        double d = std::fabs (best_a * xs[j] + best_b * ys[j] + best_c);
        if (d >= distance_threshold_) {
            new_xs.push_back (xs[j]);
            new_ys.push_back (ys[j]);
        }
    }

    data_.width    = new_xs.size ();
    data_.height   = 1;
    data_.row_step = data_.width * POINT_STEP;
    data_.data.resize (data_.row_step);

    for (size_t i = 0; i < new_xs.size (); i++) {
        std::memcpy (data_.data.data () + i * POINT_STEP + 0, &new_xs[i], 4);
        std::memcpy (data_.data.data () + i * POINT_STEP + 4, &new_ys[i], 4);
        float z = 0.0f;
        std::memcpy (data_.data.data () + i * POINT_STEP + 8, &z, 4);
    }
}

void line_detector::calculate_corner () {
    corners_.poses.clear ();
    for (size_t i = 0; i < lines_.lines.size (); i++) {
        for (size_t j = i + 1; j < lines_.lines.size (); j++) {
            double a1 = lines_.lines[i].a;
            double b1 = lines_.lines[i].b;
            double c1 = lines_.lines[i].c;
            double a2 = lines_.lines[j].a;
            double b2 = lines_.lines[j].b;
            double c2 = lines_.lines[j].c;

            double det = a1 * b2 - a2 * b1;
            if (std::fabs (det) < 1e-8) continue;  // 平行

            double x = (b1 * c2 - b2 * c1) / det;
            double y = (c1 * a2 - c2 * a1) / det;

            geometry_msgs::msg::Pose p;
            p.position.x    = x;
            p.position.y    = y;
            p.position.z    = 0.0;
            p.orientation.w = 1.0;
            corners_.poses.push_back (p);
        }
    }
}
void line_detector::calculate_line_segment () {
    line_segments_.line_segments.clear ();

    const uint8_t *raw        = raw_data_.data.data ();
    const uint32_t POINT_STEP = raw_data_.point_step;
    const uint32_t WIDTH      = raw_data_.width;
    const uint32_t HEIGHT     = raw_data_.height;

    std::vector<float> xs, ys;
    xs.reserve (WIDTH * HEIGHT);
    ys.reserve (WIDTH * HEIGHT);

    for (size_t i = 0; i < WIDTH * HEIGHT; i++) {
        float x = *reinterpret_cast<const float *> (raw + i * POINT_STEP + 0);
        float y = *reinterpret_cast<const float *> (raw + i * POINT_STEP + 4);
        if (std::isfinite (x) && std::isfinite (y)) {
            xs.push_back (x);
            ys.push_back (y);
        }
    }

    for (auto &line : lines_.lines) {
        double a = line.a;
        double b = line.b;
        double c = line.c;

        std::vector<std::pair<float, float>> inliers;
        for (size_t i = 0; i < xs.size (); i++) {
            double d = std::fabs (a * xs[i] + b * ys[i] + c);
            if (d < distance_threshold_) {
                inliers.push_back ({xs[i], ys[i]});
            }
        }
        if (inliers.size () < 2) continue;

        // 線の方向ベクトルと基準点
        double dir_x = -b;
        double dir_y = a;
        double norm  = std::sqrt (dir_x * dir_x + dir_y * dir_y);
        dir_x /= norm;
        dir_y /= norm;
        double base_x = -a * c;
        double base_y = -b * c;

        // 点を射影して並べる
        std::vector<std::pair<double, std::pair<float, float>>> projected;
        projected.reserve (inliers.size ());
        for (auto &p : inliers) {
            double t = (p.first - base_x) * dir_x + (p.second - base_y) * dir_y;
            projected.push_back ({t, p});
        }
        std::sort (projected.begin (), projected.end (), [] (auto &a, auto &b) { return a.first < b.first; });

        // ギャップ判定しながら分割
        std::vector<std::pair<float, float>> current_cluster;
        for (size_t i = 0; i < projected.size (); i++) {
            if (current_cluster.empty ()) {
                current_cluster.push_back (projected[i].second);
                continue;
            }
            double gap = std::hypot (projected[i].second.first - current_cluster.back ().first, projected[i].second.second - current_cluster.back ().second);
            if (gap > segment_gap_threshold_) {
                if (current_cluster.size () >= 2) {
                    auto                         first = current_cluster.front ();
                    auto                         last  = current_cluster.back ();
                    natto_msgs::msg::LineSegment seg;
                    seg.start.x = first.first;
                    seg.start.y = first.second;
                    seg.start.z = 0.0;
                    seg.end.x   = last.first;
                    seg.end.y   = last.second;
                    seg.end.z   = 0.0;
                    line_segments_.line_segments.push_back (seg);
                }
                current_cluster.clear ();
            }
            current_cluster.push_back (projected[i].second);
        }

        // 最後のクラスタ
        if (current_cluster.size () >= 2) {
            auto                         first = current_cluster.front ();
            auto                         last  = current_cluster.back ();
            natto_msgs::msg::LineSegment seg;
            seg.start.x = first.first;
            seg.start.y = first.second;
            seg.start.z = 0.0;
            seg.end.x   = last.first;
            seg.end.y   = last.second;
            seg.end.z   = 0.0;
            line_segments_.line_segments.push_back (seg);
        }
    }
}

}  // namespace line_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (line_detector::line_detector)
