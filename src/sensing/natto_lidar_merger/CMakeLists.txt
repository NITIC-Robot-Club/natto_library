cmake_minimum_required(VERSION 3.8)
project(natto_lidar_merger)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "merge_pointcloud2::merge_pointcloud2"
    EXECUTABLE merge_pointcloud2
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "merge_laserscan::merge_laserscan"
    EXECUTABLE merge_laserscan
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)