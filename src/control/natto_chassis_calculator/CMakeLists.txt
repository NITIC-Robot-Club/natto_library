cmake_minimum_required(VERSION 3.8)
project(natto_chassis_calculator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "chassis_calculator::chassis_calculator"
    EXECUTABLE chassis_calculator
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "two_wheel_calculator::two_wheel_calculator"
    EXECUTABLE two_wheel_calculator
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)