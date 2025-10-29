#include "natto_swerve_calculator/swerve_calculator.hpp"

namespace swerve_calculator {

swerve_calculator::swerve_calculator (const rclcpp::NodeOptions &node_options) : Node ("swerve_calculator", node_options) {
    swerve_command_publisher_ = this->create_publisher<natto_msgs::msg::Swerve> ("swerve_command", 10);
    twist_command_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped> ("command_velocity", 10, std::bind (&swerve_calculator::command_velocity_callback, this, std::placeholders::_1));
    swerve_result_subscriber_ = this->create_subscription<natto_msgs::msg::Swerve> ("swerve_result", 10, std::bind (&swerve_calculator::swerve_result_callback, this, std::placeholders::_1));

    infinite_swerve_mode_ = this->declare_parameter<bool> ("infinite_swerve_mode", false);
    wheel_radius_         = this->declare_parameter<double> ("wheel_radius", 0.05);
    wheel_position_x      = this->declare_parameter<std::vector<double>> ("wheel_position_x", {0.5, -0.5, -0.5, 0.5});
    wheel_position_y      = this->declare_parameter<std::vector<double>> ("wheel_position_y", {0.5, 0.5, -0.5, -0.5});

    num_wheels_ = wheel_position_x.size ();
    if (wheel_position_y.size () != num_wheels_) {
        RCLCPP_ERROR (this->get_logger (), "wheel_position_x and wheel_position_y must have the same size.");
        throw std::runtime_error ("wheel_position_x and wheel_position_y must have the same size.");
    }

    RCLCPP_INFO (this->get_logger (), "swerve_calculator node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "Infinite swerve mode: %s", infinite_swerve_mode_ ? "true" : "false");
    RCLCPP_INFO (this->get_logger (), "Wheel radius: %.2f m", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "Number of wheels: %d", num_wheels_);
    for (int i = 0; i < num_wheels_; i++) {
        RCLCPP_INFO (this->get_logger (), "Wheel %d position: (%.2f, %.2f)", i, wheel_position_x[i], wheel_position_y[i]);
    }
}

void swerve_calculator::command_velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    natto_msgs::msg::Swerve swerve_msg;
    swerve_msg.wheel_angle.resize (num_wheels_, 0.0);
    swerve_msg.wheel_speed.resize (num_wheels_, 0.0);

    double x = msg->twist.linear.x;
    double y = msg->twist.linear.y;
    double z = msg->twist.angular.z;

    for (int i = 0; i < num_wheels_; i++) {
        if (x == 0.0 && y == 0.0 && z == 0.0) {
            double vx = -wheel_position_y[i];
            double vy = +wheel_position_x[i];

            swerve_msg.wheel_speed[i] = 0.0;
            swerve_msg.wheel_angle[i] = std::atan2 (vy, vx);
        } else {
            double vx    = x - z * wheel_position_y[i];
            double vy    = y + z * wheel_position_x[i];
            double v     = std::hypot (vx, vy);
            double angle = v / (2.0 * M_PI * wheel_radius_);

            swerve_msg.wheel_speed[i] = angle;
            swerve_msg.wheel_angle[i] = std::atan2 (vy, vx);
        }
    }
    if (infinite_swerve_mode_) {
        for (int i = 0; i < num_wheels_; i++) {
            if(swerve_result_.wheel_angle.size()!=num_wheels_ || swerve_result_.wheel_speed.size()!=num_wheels_){
                continue;
            }
            while (swerve_msg.wheel_angle[i] - swerve_result_.wheel_angle[i] > M_PI) {
                swerve_msg.wheel_angle[i] -= 2.0 * M_PI;
            }
            while (swerve_msg.wheel_angle[i] - swerve_result_.wheel_angle[i] < -M_PI) {
                swerve_msg.wheel_angle[i] += 2.0 * M_PI;
            }
            if (std::fabs (swerve_msg.wheel_angle[i] - swerve_result_.wheel_angle[i]) > M_PI / 2.0) {
                swerve_msg.wheel_angle[i] += (swerve_msg.wheel_angle[i] > swerve_result_.wheel_angle[i]) ? -M_PI : M_PI;
                swerve_msg.wheel_speed[i] *= -1.0;
            }
        }
    }
    swerve_command_publisher_->publish (swerve_msg);
}

void swerve_calculator::swerve_result_callback (const natto_msgs::msg::Swerve::SharedPtr msg) {
    swerve_result_ = *msg;
}

}  // namespace swerve_calculator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (swerve_calculator::swerve_calculator)