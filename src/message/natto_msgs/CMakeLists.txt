cmake_minimum_required(VERSION 3.12)
project(natto_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(natto_msgs_files "")

file(
  GLOB natto_msgs_files
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_DEPENDS
  "msg/*.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${natto_msgs_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_auto_package()