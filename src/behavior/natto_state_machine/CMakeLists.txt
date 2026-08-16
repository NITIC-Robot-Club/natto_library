cmake_minimum_required(VERSION 3.8)
project(natto_state_machine)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/control_type_action.cpp
    src/default_action.cpp
    src/joint_action.cpp
    src/mermaid_loader.cpp
    src/origin_action.cpp
    src/pose_action.cpp
    src/state_machine.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "default_action::default_action"
    EXECUTABLE default_action
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "mermaid_loader::mermaid_loader"
    EXECUTABLE mermaid_loader
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "state_machine::state_machine"
    EXECUTABLE state_machine
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
