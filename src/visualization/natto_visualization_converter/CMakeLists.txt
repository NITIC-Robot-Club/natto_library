cmake_minimum_required(VERSION 3.8)
project(natto_visualization_converter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "corners_visualizer::corners_visualizer"
    EXECUTABLE corners_visualizer
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "footprint_publisher::footprint_publisher"
    EXECUTABLE footprint_publisher
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "line_segments_visualizer::line_segments_visualizer"
    EXECUTABLE line_segments_visualizer
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "lines_visualizer::lines_visualizer"
    EXECUTABLE lines_visualizer
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "map_visualizer::map_visualizer"
    EXECUTABLE map_visualizer
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)