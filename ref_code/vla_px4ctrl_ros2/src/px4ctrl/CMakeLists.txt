cmake_minimum_required(VERSION 3.8)
project(px4ctrl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uav_utils REQUIRED)

add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

target_include_directories(px4ctrl_node PRIVATE
  src
)

target_link_libraries(px4ctrl_node Eigen3::Eigen)

ament_target_dependencies(px4ctrl_node
  geometry_msgs
  mavros_msgs
  quadrotor_msgs
  rclcpp
  sensor_msgs
  std_msgs
  uav_utils
)

install(TARGETS px4ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/feetech_gripper_node.py
  scripts/fly_x_gripper_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
