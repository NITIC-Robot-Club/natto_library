cmake_minimum_required(VERSION 3.8)
project(natto_laser_filter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "laser_filter::laser_filter"
    EXECUTABLE laser_filter
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
