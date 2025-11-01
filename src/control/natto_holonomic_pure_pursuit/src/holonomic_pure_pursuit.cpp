#include "natto_holonomic_pure_pursuit/holonomic_pure_pursuit.hpp"

namespace holonomic_pure_pursuit {

holonomic_pure_pursuit::holonomic_pure_pursuit (const rclcpp::NodeOptions &options) : Node ("holonomic_pure_pursuit", options) {
    cmd_vel_publisher_   = this->create_publisher<geometry_msgs::msg::TwistStamped> ("command_velocity", 1);
    lookahead_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("lookahead", 1);
    pose_subscriber_     = this->create_subscription<geometry_msgs::msg::PoseStamped> ("current_pose", 1, std::bind (&holonomic_pure_pursuit::pose_callback, this, std::placeholders::_1));
    path_subscriber_     = this->create_subscription<nav_msgs::msg::Path> ("path", 1, std::bind (&holonomic_pure_pursuit::path_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::milliseconds (50), std::bind (&holonomic_pure_pursuit::timer_callback, this));

    lookahead_time_               = this->declare_parameter ("lookahead_time", 1.0);
    min_lookahead_distance_       = this->declare_parameter ("min_lookahead_distance", 0.1);
    max_lookahead_distance_       = this->declare_parameter ("max_lookahead_distance", 1.0);
    angle_speed_p_                = this->declare_parameter ("angle_speed_p", 1.0);
    curvature_decceleration_p_    = this->declare_parameter ("curvature_decceleration_p", 1.0);
    min_curvature_speed_m_s_      = this->declare_parameter ("min_curvature_speed_m_s", 0.3);
    angle_decceleration_p_        = this->declare_parameter ("angle_decceleration_p", 1.0);
    max_speed_xy_m_s_             = this->declare_parameter ("max_speed_xy_m_s", 3.0);
    min_speed_xy_m_s_             = this->declare_parameter ("min_speed_xy_m_s", 0.1);
    max_speed_z_deg_s_            = this->declare_parameter ("max_speed_z_deg_s", 180.0);
    min_speed_z_deg_s_            = this->declare_parameter ("min_speed_z_deg_s", 18.0);
    max_acceleration_xy_m_s2_     = this->declare_parameter ("max_acceleration_xy_m_s2_", 10.0);
    max_acceleration_z_deg_s2_    = this->declare_parameter ("max_acceleration_z_deg_s2", 500.0);
    goal_deceleration_m_s2_       = this->declare_parameter ("goal_deceleration_m_s2", 4.0);
    goal_deceleration_distance_p_ = this->declare_parameter ("goal_deceleration_distance_p", 1.0);
    goal_position_tolerance_      = this->declare_parameter ("goal_position_tolerance_m", 0.03);
    goal_yaw_tolerance_deg_       = this->declare_parameter ("goal_yaw_tolerance_deg", 10.0);
    goal_speed_tolerance_xy_m_s_  = this->declare_parameter ("goal_speed_tolerance_xy_m_s", 0.3);
    goal_speed_tolerance_z_deg_s_ = this->declare_parameter ("goal_speed_tolerance_z_deg_s", 30.0);

    RCLCPP_INFO (this->get_logger (), "Holonomic Pure Pursuit Node has been started.");
    RCLCPP_INFO (this->get_logger (), "lookahead_time : %f", lookahead_time_);
    RCLCPP_INFO (this->get_logger (), "min_lookahead_distance : %f", min_lookahead_distance_);
    RCLCPP_INFO (this->get_logger (), "max_lookahead_distance : %f", max_lookahead_distance_);
    RCLCPP_INFO (this->get_logger (), "angle_speed_p : %f", angle_speed_p_);
    RCLCPP_INFO (this->get_logger (), "curvature_decceleration_p : %f", curvature_decceleration_p_);
    RCLCPP_INFO (this->get_logger (), "min_curvature_speed_m_s : %f", min_curvature_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "angle_decceleration_p : %f", angle_decceleration_p_);
    RCLCPP_INFO (this->get_logger (), "max_speed_xy_m_s : %f", max_speed_xy_m_s_);
    RCLCPP_INFO (this->get_logger (), "min_speed_xy_m_s : %f", min_speed_xy_m_s_);
    RCLCPP_INFO (this->get_logger (), "max_speed_z_deg_s : %f", max_speed_z_deg_s_);
    RCLCPP_INFO (this->get_logger (), "min_speed_z_deg_s : %f", min_speed_z_deg_s_);
    RCLCPP_INFO (this->get_logger (), "max_acceleration_xy_m_s2_ : %f", max_acceleration_xy_m_s2_);
    RCLCPP_INFO (this->get_logger (), "max_acceleration_z_deg_s2 : %f", max_acceleration_z_deg_s2_);
    RCLCPP_INFO (this->get_logger (), "goal_deceleration_m_s2 : %f", goal_deceleration_m_s2_);
    RCLCPP_INFO (this->get_logger (), "goal_position_tolerance_m : %f", goal_position_tolerance_);
    RCLCPP_INFO (this->get_logger (), "goal_yaw_tolerance_deg : %f", goal_yaw_tolerance_deg_);
    RCLCPP_INFO (this->get_logger (), "goal_speed_tolerance_xy_m_s : %f", goal_speed_tolerance_xy_m_s_);
    RCLCPP_INFO (this->get_logger (), "goal_speed_tolerance_z_deg_s : %f", goal_speed_tolerance_z_deg_s_);
}

void holonomic_pure_pursuit::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void holonomic_pure_pursuit::path_callback (const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = *msg;
}

void holonomic_pure_pursuit::timer_callback () {
    // パラメータ取得

    if (path_.poses.empty ()) {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp    = this->now ();
        cmd_vel.header.frame_id = "base_link";
        cmd_vel.twist.linear.x  = 0.0;
        cmd_vel.twist.linear.y  = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        cmd_vel_publisher_->publish (cmd_vel);
        return;
    }
    double position_error        = std::hypot (path_.poses.back ().pose.position.x - current_pose_.pose.position.x, path_.poses.back ().pose.position.y - current_pose_.pose.position.y);
    bool   goal_position_reached = (position_error < goal_position_tolerance_);

    double yaw_error = tf2::getYaw(path_.poses.back ().pose.orientation) - tf2::getYaw(current_pose_.pose.orientation);
    while (yaw_error > +M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
    yaw_error             = std::abs (yaw_error);
    bool goal_yaw_reached = (yaw_error < goal_yaw_tolerance_deg_ * M_PI / 180.0);

    double current_speed_xy      = std::hypot (last_cmd_vel_.twist.linear.x, last_cmd_vel_.twist.linear.y);
    bool   goal_speed_xy_reached = (current_speed_xy < goal_speed_tolerance_xy_m_s_);
    double current_speed_z       = std::abs (last_cmd_vel_.twist.angular.z);
    bool   goal_speed_z_reached  = (current_speed_z < goal_speed_tolerance_z_deg_s_ * M_PI / 180.0);

    // 最近傍点の探索
    double min_distance  = std::numeric_limits<double>::max ();
    int    closest_index = -1;
    for (int i = 0; i < path_.poses.size (); i++) {
        double dist = std::hypot (path_.poses[i].pose.position.x - current_pose_.pose.position.x, path_.poses[i].pose.position.y - current_pose_.pose.position.y);
        if (dist < min_distance) {
            min_distance  = dist;
            closest_index = i;
        }
    }

    if (closest_index == -1) return;
    if (closest_index + 1 >= path_.poses.size ()) closest_index = path_.poses.size () - 2;

    // ゴール位置
    geometry_msgs::msg::Pose goal_pose     = path_.poses.back ().pose;
    double                   goal_distance = std::hypot (goal_pose.position.x - current_pose_.pose.position.x, goal_pose.position.y - current_pose_.pose.position.y);

    double last_speed = std::hypot (last_cmd_vel_.twist.linear.x, last_cmd_vel_.twist.linear.y);
    // double predict_dt = std::min (lookahead_time_, 0.5);  // 最大0.5s先まで予測
    double predicted_speed = last_speed + max_acceleration_xy_m_s2_ * lookahead_time_;
    predicted_speed        = std::clamp (predicted_speed, 0.0, max_speed_xy_m_s_);
    lookahead_distance_    = std::clamp (lookahead_time_ * predicted_speed, min_lookahead_distance_, max_lookahead_distance_);

    int    lookahead_index = closest_index;
    double acc_dist        = 0.0;
    double prev_x          = current_pose_.pose.position.x;
    double prev_y          = current_pose_.pose.position.y;
    for (int i = closest_index; i < path_.poses.size (); i++) {
        double px  = path_.poses[i].pose.position.x;
        double py  = path_.poses[i].pose.position.y;
        double seg = std::hypot (px - prev_x, py - prev_y);
        acc_dist += seg;
        prev_x          = px;
        prev_y          = py;
        lookahead_index = i;
        if (acc_dist >= lookahead_distance_) break;
    }

    if (goal_position_reached) {
        lookahead_index = path_.poses.size () - 1;
    }

    double dx = path_.poses[lookahead_index].pose.position.x - current_pose_.pose.position.x;
    double dy = path_.poses[lookahead_index].pose.position.y - current_pose_.pose.position.y;

    double current_yaw = tf2::getYaw (current_pose_.pose.orientation);
    double angle_diff  = std::atan2 (dy, dx) - current_yaw;

    // 加速度制限付き速度推定
    double target_speed = std::hypot (dx, dy) / lookahead_time_;
    // 最大速度制限
    target_speed = std::clamp (target_speed, 0.0, max_speed_xy_m_s_);

    double d = std::max (goal_distance, 0.0);

    // // ここでさらに物理的に止まれる最大速度で上限をかける（安全側）
    double max_stop_speed = std::sqrt (2.0 * std::min (goal_deceleration_m_s2_, max_acceleration_xy_m_s2_) * std::max (0.0, d / goal_deceleration_distance_p_));
    target_speed          = std::min (target_speed, max_stop_speed);

    // 曲率に応じた速度制限
    int p1 = closest_index;
    int p2 = (lookahead_index + closest_index) / 2;
    int p3 = lookahead_index;

    double a         = std::hypot (path_.poses[p1].pose.position.x - path_.poses[p2].pose.position.x, path_.poses[p1].pose.position.y - path_.poses[p2].pose.position.y);
    double b         = std::hypot (path_.poses[p2].pose.position.x - path_.poses[p3].pose.position.x, path_.poses[p2].pose.position.y - path_.poses[p3].pose.position.y);
    double c         = std::hypot (path_.poses[p1].pose.position.x - path_.poses[p3].pose.position.x, path_.poses[p1].pose.position.y - path_.poses[p3].pose.position.y);
    double s         = (a + b + c) / 2.0;
    double area      = std::sqrt (s * (s - a) * (s - b) * (s - c));
    double curvature = 0.0;
    if (a * b * c > 1e-6) {
        curvature = 4.0 * area / (a * b * c);
    }

    double curvature_speed = target_speed / (std::abs (curvature * curvature_decceleration_p_) + 1e-6);

    target_speed = std::min (target_speed, std::max (curvature_speed, min_curvature_speed_m_s_));

    target_speed = std::max (target_speed, min_speed_xy_m_s_);

    double delta_t_s    = 0.05;
    double acceleration = (target_speed - last_speed) / delta_t_s;
    acceleration        = std::clamp (acceleration, -max_acceleration_xy_m_s2_, max_acceleration_xy_m_s2_);
    double speed        = last_speed + acceleration * delta_t_s;

    double yaw_diff = tf2::getYaw (path_.poses[lookahead_index].pose.orientation) - current_yaw;
    while (yaw_diff > +M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    double yaw_speed = yaw_diff / lookahead_time_ * angle_speed_p_;
    // 加速度を考慮
    double angle_acceleration = (yaw_speed - last_cmd_vel_.twist.angular.z) / delta_t_s;
    angle_acceleration        = std::clamp (angle_acceleration, -max_acceleration_z_deg_s2_ * M_PI / 180.0, max_acceleration_z_deg_s2_ * M_PI / 180.0);
    yaw_speed                 = last_cmd_vel_.twist.angular.z + angle_acceleration * delta_t_s;
    yaw_speed                 = std::clamp (yaw_speed, -max_speed_z_deg_s_ * M_PI / 180.0, max_speed_z_deg_s_ * M_PI / 180.0);

    // if (!goal_yaw_reached) {
    //     if (yaw_speed < 0) {
    //         yaw_speed = std::min (yaw_speed, -min_speed_z_deg_s_ * M_PI / 180.0);
    //     } else {
    //         yaw_speed = std::max (yaw_speed, min_speed_z_deg_s_ * M_PI / 180.0);
    //     }
    // }

    if (goal_position_reached && goal_speed_xy_reached) {
        speed = 0.0;
    }

    if (goal_yaw_reached && goal_speed_z_reached) {
        yaw_speed = 0.0;
    }

    // Twist 発行
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp    = this->now ();
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.twist.linear.x  = speed * std::cos (angle_diff - yaw_speed * lookahead_time_ * angle_decceleration_p_);
    cmd_vel.twist.linear.y  = speed * std::sin (angle_diff - yaw_speed * lookahead_time_ * angle_decceleration_p_);
    cmd_vel.twist.angular.z = yaw_speed;
    cmd_vel_publisher_->publish (cmd_vel);

    // lookahead可視化
    geometry_msgs::msg::PoseStamped lookahead_msg;
    lookahead_msg.header.stamp    = this->now ();
    lookahead_msg.header.frame_id = "map";
    lookahead_msg.pose            = path_.poses[lookahead_index].pose;
    lookahead_publisher_->publish (lookahead_msg);
    last_cmd_vel_ = cmd_vel;
}
}  // namespace holonomic_pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (holonomic_pure_pursuit::holonomic_pure_pursuit)