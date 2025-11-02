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
    lines_publisher_       = this->create_publisher<natto_msgs::msg::LineArray> ("lines", 10);
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2> ("pointcloud2", 10, std::bind (&line_detector::pointcloud_callback, this, std::placeholders::_1));

    max_iterations_     = this->declare_parameter<int> ("max_iterations", 100);
    max_lines_          = this->declare_parameter<int> ("max_lines", 10);
    min_inliers_        = this->declare_parameter<int> ("min_inliers", 75);
    distance_threshold_ = this->declare_parameter<double> ("distance_threshold", 0.01);
}

void line_detector::pointcloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    data_ = *msg;
    lines_.lines.clear ();
    for (int i = 0; i < max_lines_; i++) {
        process_pointcloud ();
    }
    lines_publisher_->publish (lines_);
}

void line_detector::process_pointcloud () {
    const uint32_t POINT_STEP = data_.point_step;
    const uint32_t WIDTH      = data_.width;
    const uint32_t HEIGHT     = data_.height;
    const uint8_t *raw        = data_.data.data ();

    std::vector<float> xs;
    std::vector<float> ys;

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

        // ax + by + c = 0 形式
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

    natto_msgs::msg::Line line_msg;
    line_msg.a = best_a;
    line_msg.b = best_b;
    line_msg.c = best_c;
    if (best_inliers > min_inliers_) {
        lines_.lines.push_back (line_msg);
    }

    std::vector<float> new_xs, new_ys;
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
}  // namespace line_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (line_detector::line_detector)
