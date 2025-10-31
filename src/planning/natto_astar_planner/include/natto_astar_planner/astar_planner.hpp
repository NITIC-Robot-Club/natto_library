#ifndef __ASTAR_PLANNER_HPP__
#define __ASTAR_PLANNER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace astar_planner {

class astar_planner : public rclcpp::Node {
   public:
    astar_planner (const rclcpp::NodeOptions &node_options);

   private:
    int    theta_resolution_deg_;
    double xy_inflation_, xy_offset_, yaw_offset_;
    double grad_alpha_, grad_beta_, grad_gamma_, grad_step_size_;

    nav_msgs::msg::OccupancyGrid       raw_map_;
    nav_msgs::msg::OccupancyGrid       costmap_;
    nav_msgs::msg::OccupancyGrid       angular_debug_map_;
    geometry_msgs::msg::PoseStamped    goal_pose_;
    geometry_msgs::msg::PoseStamped    current_pose_;
    geometry_msgs::msg::PolygonStamped footprint_;
    nav_msgs::msg::Path                path_;
    std::vector<int8_t>                footprint_mask_;

    int    footprint_mask_w_      = 0;
    int    footprint_mask_h_      = 0;
    double footprint_mask_res_    = 0.05;  // m/pixel, 調整可
    double footprint_mask_radius_ = 0.0;   // footprint最大半径キャッシュ

    void map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void footprint_callback (const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

    void   create_path ();
    void   build_footprint_mask ();
    void   create_costmap ();
    bool   is_same_map (nav_msgs::msg::OccupancyGrid latest_map);
    bool   is_same_footprint (geometry_msgs::msg::PolygonStamped latest_footprint);
    bool   check_collision (geometry_msgs::msg::Pose pose);
    double wrap_to_2pi (double angle);
    bool   point_in_polygon (double x, double y, geometry_msgs::msg::Polygon polygon);

    nav_msgs::msg::Path linear_astar ();
    nav_msgs::msg::Path linear_smoother (nav_msgs::msg::Path linear_path);
    nav_msgs::msg::Path angular_astar (nav_msgs::msg::Path linear_smoothed_path);
    nav_msgs::msg::Path angular_smoother (nav_msgs::msg::Path angular_path);
    std::pair<int, int> to_grid (double x, double y);

    geometry_msgs::msg::Pose find_free_space_pose (geometry_msgs::msg::Pose pose);

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                   path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr          costmap_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr          debug_map_publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr       map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    goal_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    current_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_subscription_;
};

}  // namespace astar_planner

#endif  // __ASTAR_PLANNER_HPP__
