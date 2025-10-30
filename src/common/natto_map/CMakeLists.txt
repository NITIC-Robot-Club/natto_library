cmake_minimum_required(VERSION 3.8)
project(natto_map)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "map_loader::map_loader"
    EXECUTABLE map_loader
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "map_converter::map_converter"
    EXECUTABLE map_converter
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)