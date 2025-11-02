cmake_minimum_required(VERSION 3.8)
project(natto_ransac)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "line_detector::line_detector"
    EXECUTABLE line_detector
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "line_segment_detector::line_segment_detector"
    EXECUTABLE line_segment_detector
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)