#include "natto_simple_simulation/simulate_swerve.hpp"

namespace simulate_swerve {

simulate_swerve::simulate_swerve (const rclcpp::NodeOptions &node_options) : Node ("simulate_swerve", node_options) {
    swerve_result_publisher_   = this->create_publisher<natto_msgs::msg::Swerve> ("swerve_result", 10);
    pose_publisher_            = this->create_publisher<geometry_msgs::msg::PoseStamped> ("pose", 10);
    swerve_command_subscriber_ = this->create_subscription<natto_msgs::msg::Swerve> ("swerve_command", 10, std::bind (&simulate_swerve::swerve_command_callback, this, std::placeholders::_1));

    infinite_swerve_mode_        = this->declare_parameter<bool> ("infinite_swerve_mode", false);
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

    RCLCPP_INFO (this->get_logger (), "simulate_swerve node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "simulation period: %d ms", period_ms);
    RCLCPP_INFO (this->get_logger (), "Infinite swerve mode: %s", infinite_swerve_mode_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "Wheel radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %d", num_wheels_);
    for (int i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "Wheel %d position: (%.2f, %.2f)", i, wheel_position_x[i], wheel_position_y[i]);
    }
    RCLCPP_INFO (this->get_logger (), "Angle gain P: %.2f, D: %.2f", angle_gain_p_, angle_gain_d_);
    RCLCPP_INFO (this->get_logger (), "Speed gain P: %.2f, D: %.2f", speed_gain_p_, speed_gain_d_);

    command.wheel_angle.resize (num_wheels_, 0.0);
    command.wheel_speed.resize (num_wheels_, 0.0);
    result.wheel_angle.resize (num_wheels_, 0.0);
    result.wheel_speed.resize (num_wheels_, 0.0);

    timer_ = this->create_wall_timer (std::chrono::milliseconds (period_ms), std::bind (&simulate_swerve::timer_callback, this));
}

void simulate_swerve::swerve_command_callback (const natto_msgs::msg::Swerve::SharedPtr msg) {
    received_commands.push_back (*msg);
}

void simulate_swerve::timer_callback () {
    if (received_commands.empty ()) {
        received_commands.push_back (result);
    }
    natto_msgs::msg::Swerve command_sum;
    command_sum.wheel_angle.resize (num_wheels_, 0.0);
    command_sum.wheel_speed.resize (num_wheels_, 0.0);
    for (int i = 0; i < received_commands.size (); i++) {
        for (int j = 0; j < num_wheels_; j++) {
            if (received_commands[i].wheel_speed.size () != num_wheels_ || received_commands[i].wheel_angle.size () != num_wheels_) {
                RCLCPP_FATAL (this->get_logger (), "Received command size does not match number of wheels.");
                RCLCPP_INFO (this->get_logger (), "Expected size: %d, Received wheel_angle size: %zu, wheel_speed size: %zu", num_wheels_, received_commands[i].wheel_angle.size (), received_commands[i].wheel_speed.size ());
            }
            command_sum.wheel_angle[j] += received_commands[i].wheel_angle[j];
            command_sum.wheel_speed[j] += received_commands[i].wheel_speed[j];
        }
    }
    for (int j = 0; j < num_wheels_; j++) {
        command.wheel_angle[j] = command_sum.wheel_angle[j] / received_commands.size ();
        command.wheel_speed[j] = command_sum.wheel_speed[j] / received_commands.size ();
    }

    for (int i = 0; i < num_wheels_; i++) {
        double angle_error = command.wheel_angle[i] - result.wheel_angle[i];
        double speed_error = command.wheel_speed[i] - result.wheel_speed[i];

        double angle_adjustment = angle_gain_p_ * angle_error - angle_gain_d_ * (result.wheel_angle[i] - command.wheel_angle[i]);
        double speed_adjustment = speed_gain_p_ * speed_error - speed_gain_d_ * (result.wheel_speed[i] - command.wheel_speed[i]);

        result.wheel_angle[i] += angle_adjustment * period_ms / 1000.0;
        result.wheel_speed[i] += speed_adjustment * period_ms / 1000.0;

        if (abs (received_commands.back ().wheel_speed[i] - result.wheel_speed[i]) < 0.01) {
            // +の目標から-0.0を目標にしたときなどの見た目の問題
            // 誤差が小さいときは見た目のために一致させる
            result.wheel_speed[i] = received_commands.back ().wheel_speed[i];
        }
    }
    swerve_result_publisher_->publish (result);
    received_commands.clear ();

    double ATA[3][3] = {};  // A^T * A
    double ATb[3]    = {};  // A^T * b

    for (int i = 0; i < num_wheels_; i++) {
        double angle = result.wheel_angle[i];
        double speed = result.wheel_speed[i] * 2.0 * M_PI * wheel_radius_;

        double ax[3] = {1.0, 0.0, -wheel_position_y[i]};
        double ay[3] = {0.0, 1.0, +wheel_position_x[i]};

        double bx = speed * std::cos (angle);
        double by = speed * std::sin (angle);

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
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

    for (int i = 0; i < 3; ++i) {
        double pivot = A[i][i];
        for (int j = i; j < 4; ++j) {
            A[i][j] /= pivot;
        }
        for (int k = 0; k < 3; ++k) {
            if (k == i) continue;
            double factor = A[k][i];
            for (int j = i; j < 4; ++j) {
                A[k][j] -= factor * A[i][j];
            }
        }
    }

    double vx = A[0][3];
    double vy = A[1][3];
    double vz = A[2][3];

    double yaw   = tf2::getYaw (current_pose.pose.orientation);
    double speed = std::hypot (vx, vy);
    current_pose.pose.position.x += speed * std::cos (yaw) * period_ms / 1000.0;
    current_pose.pose.position.y += speed * std::sin (yaw) * period_ms / 1000.0;
    yaw += vz * period_ms / 1000.0;
    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, yaw);
    current_pose.pose.orientation.x = q.x ();
    current_pose.pose.orientation.y = q.y ();
    current_pose.pose.orientation.z = q.z ();
    current_pose.pose.orientation.w = q.w ();
    current_pose.header.stamp       = this->now ();
    current_pose.header.frame_id    = "map";
    pose_publisher_->publish (current_pose);
}

}  // namespace simulate_swerve

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (simulate_swerve::simulate_swerve)