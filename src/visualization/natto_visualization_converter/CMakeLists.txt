cmake_minimum_required(VERSION 3.8)
project(natto_visualization_converter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "visualize_map::visualize_map"
    EXECUTABLE visualize_map
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "visualize_swerve::visualize_swerve"
    EXECUTABLE visualize_swerve
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)