cmake_minimum_required(VERSION 3.8)
project(hls_gripper)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/gripper_gravity_calibration.py
    scripts/hls_gripper_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
