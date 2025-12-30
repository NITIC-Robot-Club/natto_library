cmake_minimum_required(VERSION 3.8)
project(natto_wheel_odometry)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "omni_odometry::omni_odometry"
    EXECUTABLE omni_odometry
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "wheel_odometry::wheel_odometry"
    EXECUTABLE wheel_odometry
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)