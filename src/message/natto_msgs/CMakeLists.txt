cmake_minimum_required(VERSION 3.8)
project(natto_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Can.msg"
  "msg/Circle.msg"
  "msg/CircleArray.msg"
  "msg/Line.msg"
  "msg/LineArray.msg"
  "msg/LineSegment.msg"
  "msg/LineSegmentArray.msg"
  "msg/Map.msg"
  "msg/Omni.msg"
  "msg/Swerve.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_auto_package()