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

#include "natto_simple_simulation/swerve_simulator.hpp"

#include <cmath>
#include <stdexcept>

namespace swerve_simulator {

swerve_simulator::swerve_simulator (const rclcpp::NodeOptions &node_options) : Node ("swerve_simulator", node_options) {
    swerve_result_publisher_   = this->create_publisher<natto_msgs::msg::Swerve> ("swerve_result", 10);
    simulation_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("simulation_pose", 10);
    swerve_command_subscriber_ = this->create_subscription<natto_msgs::msg::Swerve> ("swerve_command", 10, std::bind (&swerve_simulator::swerve_command_callback, this, std::placeholders::_1));
    tf_broadcaster_            = std::make_shared<tf2_ros::TransformBroadcaster> (this);

    wheel_radius_                = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_position_x             = this->declare_parameter<std::vector<double>> ("wheel_position_x", {0.5, -0.5, -0.5, 0.5});
    wheel_position_y             = this->declare_parameter<std::vector<double>> ("wheel_position_y", {0.5, 0.5, -0.5, -0.5});
    angle_gain_p_                = this->declare_parameter<double> ("angle_gain_p", 100.0);
    angle_gain_d_                = this->declare_parameter<double> ("angle_gain_d", 0.0);
    speed_gain_p_                = this->declare_parameter<double> ("speed_gain_p", 100.0);
    speed_gain_d_                = this->declare_parameter<double> ("speed_gain_d", 0.0);
    period_ms                    = this->declare_parameter<int> ("simulation_period_ms", 1);
    current_pose.pose.position.x = this->declare_parameter<double> ("initial_pose_x", 1.0);
    current_pose.pose.position.y = this->declare_parameter<double> ("initial_pose_y", 1.0);

    num_wheels_ = wheel_position_x.size ();
    if (wheel_position_y.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_position_x and wheel_position_y must have the same size.");
        throw std::runtime_error ("wheel_position_x and wheel_position_y must have the same size.");
    }

    RCLCPP_INFO (this->get_logger (), "swerve_simulator node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "simulation period: %d ms", period_ms);
    RCLCPP_INFO (this->get_logger (), "Wheel radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %d", num_wheels_);
    for (size_t i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "Wheel %d position: (%.2f, %.2f)", i, wheel_position_x[i], wheel_position_y[i]);
    }
    RCLCPP_INFO (this->get_logger (), "Angle gain P: %.2f, D: %.2f", angle_gain_p_, angle_gain_d_);
    RCLCPP_INFO (this->get_logger (), "Speed gain P: %.2f, D: %.2f", speed_gain_p_, speed_gain_d_);

    result.wheel_angle.resize (num_wheels_, 0.0);
    result.wheel_speed.resize (num_wheels_, 0.0);


    last_time = this->now();
    timer_ = this->create_wall_timer (std::chrono::milliseconds (period_ms), std::bind (&swerve_simulator::timer_callback, this));
}

void swerve_simulator::swerve_command_callback (const natto_msgs::msg::Swerve::SharedPtr msg) {
    received_commands.push_back (*msg);
}

natto_msgs::msg::Swerve swerve_simulator::compute_average_command (const std::vector<natto_msgs::msg::Swerve> &commands) const {
    if (commands.empty ()) {
        throw std::runtime_error ("No commands to average.");
    }

    natto_msgs::msg::Swerve command_sum;
    command_sum.wheel_angle.assign (num_wheels_, 0.0);
    command_sum.wheel_speed.assign (num_wheels_, 0.0);

    for (const auto &cmd : commands) {
        if (cmd.wheel_speed.size () != num_wheels_ || cmd.wheel_angle.size () != num_wheels_) {
            RCLCPP_FATAL (this->get_logger (), "Received command size does not match number of wheels.");
            RCLCPP_INFO (
                this->get_logger (),
                "Expected size: %d, Received wheel_angle size: %zu, wheel_speed size: %zu",
                num_wheels_,
                cmd.wheel_angle.size (),
                cmd.wheel_speed.size ()
            );
            throw std::runtime_error ("Received command size does not match number of wheels.");
        }

        for (size_t j = 0; j < num_wheels_; j++) {
            command_sum.wheel_angle[j] += cmd.wheel_angle[j];
            command_sum.wheel_speed[j] += cmd.wheel_speed[j];
        }
    }

    natto_msgs::msg::Swerve average_command;
    average_command.wheel_angle.assign (num_wheels_, 0.0);
    average_command.wheel_speed.assign (num_wheels_, 0.0);
    for (size_t j = 0; j < num_wheels_; j++) {
        average_command.wheel_angle[j] = command_sum.wheel_angle[j] / commands.size ();
        average_command.wheel_speed[j] = command_sum.wheel_speed[j] / commands.size ();
    }

    return average_command;
}

natto_msgs::msg::Swerve swerve_simulator::apply_wheel_response (
    double dt,
    const natto_msgs::msg::Swerve &reference_command,
    const natto_msgs::msg::Swerve &latest_command,
    const natto_msgs::msg::Swerve &current_state
) const {
    natto_msgs::msg::Swerve updated_state = current_state;
    for (size_t i = 0; i < num_wheels_; i++) {
        double angle_error = reference_command.wheel_angle[i] - current_state.wheel_angle[i];
        double speed_error = reference_command.wheel_speed[i] - current_state.wheel_speed[i];

        double angle_adjustment = angle_gain_p_ * angle_error - angle_gain_d_ * (current_state.wheel_angle[i] - reference_command.wheel_angle[i]);
        double speed_adjustment = speed_gain_p_ * speed_error - speed_gain_d_ * (current_state.wheel_speed[i] - reference_command.wheel_speed[i]);

        updated_state.wheel_angle[i] += angle_adjustment * dt;
        updated_state.wheel_speed[i] += speed_adjustment * dt;

        if (std::abs (latest_command.wheel_speed[i] - updated_state.wheel_speed[i]) < 0.01) {
            // +の目標から-0.0を目標にしたときなどの見た目の問題
            // 誤差が小さいときは見た目のために一致させる
            updated_state.wheel_speed[i] = latest_command.wheel_speed[i];
        }
    }

    return updated_state;
}

void swerve_simulator::publish_swerve_result (const natto_msgs::msg::Swerve &swerve_state) {
    swerve_result_publisher_->publish (swerve_state);
}

std::array<double, 3> swerve_simulator::estimate_body_velocity (const natto_msgs::msg::Swerve &wheel_state) const {
    double ATA[3][3] = {};  // A^T * A
    double ATb[3]    = {};  // A^T * b

    for (size_t i = 0; i < num_wheels_; i++) {
        double angle = wheel_state.wheel_angle[i];
        double speed = wheel_state.wheel_speed[i] * 2.0 * M_PI * wheel_radius_;

        double ax[3] = {1.0, 0.0, -wheel_position_y[i]};
        double ay[3] = {0.0, 1.0, +wheel_position_x[i]};

        double bx = speed * std::cos (angle);
        double by = speed * std::sin (angle);

        for (size_t row = 0; row < 3; row++) {
            for (size_t col = 0; col < 3; col++) {
                ATA[row][col] += ax[row] * ax[col] + ay[row] * ay[col];
            }
            ATb[row] += ax[row] * bx + ay[row] * by;
        }
    }
    double A[3][4] = {
        {ATA[0][0], ATA[0][1], ATA[0][2], ATb[0]},
        {ATA[1][0], ATA[1][1], ATA[1][2], ATb[1]},
        {ATA[2][0], ATA[2][1], ATA[2][2], ATb[2]}
    };

    for (size_t i = 0; i < 3; ++i) {
        double pivot = A[i][i];
        for (size_t j = i; j < 4; ++j) {
            A[i][j] /= pivot;
        }
        for (size_t k = 0; k < 3; ++k) {
            if (k == i) continue;
            double factor = A[k][i];
            for (int j = i; j < 4; ++j) {
                A[k][j] -= factor * A[i][j];
            }
        }
    }

    return {A[0][3], A[1][3], A[2][3]};
}

geometry_msgs::msg::Pose swerve_simulator::integrate_pose (const double vx, const double vy, const double vz, const double dt) {
    const auto yaw      = tf2::getYaw (current_pose.pose.orientation);
    const auto vx_world = vx * std::cos (yaw) - vy * std::sin (yaw);
    const auto vy_world = vx * std::sin (yaw) + vy * std::cos (yaw);

    auto new_pose = current_pose.pose;

    new_pose.position.x += vx_world * dt;
    new_pose.position.y += vy_world * dt;

    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, yaw + vz * dt);
    new_pose.orientation.x = q.x ();
    new_pose.orientation.y = q.y ();
    new_pose.orientation.z = q.z ();
    new_pose.orientation.w = q.w ();

    return new_pose;
}

void swerve_simulator::publish_pose (const geometry_msgs::msg::Pose &new_pose) {
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.stamp = this->now ();
    message.header.frame_id = "map";
    message.pose = new_pose;
    simulation_pose_publisher_->publish (message);
}

void swerve_simulator::broadcast_transform (const geometry_msgs::msg::Pose &new_pose) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = this->now ();
    tf_msg.header.frame_id         = "map";
    tf_msg.child_frame_id          = "simulation";
    tf_msg.transform.translation.x = new_pose.position.x;
    tf_msg.transform.translation.y = new_pose.position.y;
    tf_msg.transform.translation.z = new_pose.position.z;
    tf_msg.transform.rotation      = new_pose.orientation;

    tf_broadcaster_->sendTransform (tf_msg);
}

void swerve_simulator::timer_callback () {
    const rclcpp::Time now_time = this->now();
    const double dt = (now_time - last_time).seconds();
    last_time = now_time;

    // コマンド入力が空のときは最新結果で補完する
    if (received_commands.empty ()) {
        received_commands.push_back (result);
    }

    // 受信した複数コマンドを平均化する
    const auto average_command = compute_average_command (received_commands);
    // コマンドに基づいてホイールの角度と速度を追従させる
    result = apply_wheel_response (dt, average_command, received_commands.back (), result);
    // 追従結果をswerve_resultとしてPublishする
    publish_swerve_result (result);
    received_commands.clear ();

    // ホイール挙動から機体の速度を推定する
    const auto [vx, vy, vz] = estimate_body_velocity (result);
    // 推定速度を積分して現在姿勢を更新する
    const auto new_pose = integrate_pose (vx, vy, vz, dt);
    current_pose.pose = new_pose;
    // 姿勢トピックをPublsihする
    publish_pose (current_pose.pose);
    // TFを更新してbroadcastする
    broadcast_transform (current_pose.pose);
}

}  // namespace swerve_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (swerve_simulator::swerve_simulator)
