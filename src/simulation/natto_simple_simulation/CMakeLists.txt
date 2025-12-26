cmake_minimum_required(VERSION 3.8)
project(natto_simple_simulation)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "swerve_simulator::swerve_simulator"
    EXECUTABLE swerve_simulator
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "omni_simulator::omni_simulator"
    EXECUTABLE omni_simulator
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "lidar_simulator::lidar_simulator"
    EXECUTABLE lidar_simulator
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "mecanum_simulator::mecanum_simulator"
    EXECUTABLE mecanum_simulator
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)