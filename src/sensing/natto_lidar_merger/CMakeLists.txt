cmake_minimum_required(VERSION 3.8)
project(natto_lidar_merger)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "pointcloud2_merger::pointcloud2_merger"
    EXECUTABLE pointcloud2_merger
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)