cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/OccMap3d.msg"
  "msg/PolyTraj.msg"
  "msg/PositionCommand.msg"
  "msg/Px4ctrlDebug.msg"
  "msg/TakeoffLand.msg"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    nav_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
