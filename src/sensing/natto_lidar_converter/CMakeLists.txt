cmake_minimum_required(VERSION 3.8)
project(natto_lidar_converter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "laserscan_to_pointcloud2::laserscan_to_pointcloud2"
    EXECUTABLE laserscan_to_pointcloud2
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "pointcloud2_to_laserscan::pointcloud2_to_laserscan"
    EXECUTABLE pointcloud2_to_laserscan
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)