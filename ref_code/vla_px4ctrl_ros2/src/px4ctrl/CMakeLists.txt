cmake_minimum_required(VERSION 3.8)
project(px4ctrl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
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
  src/physical_setpoint_protocol.cpp
)

add_executable(test_attitude_mode_node
  src/test_attitude_mode_node.cpp
)

add_executable(trajectory_planner_node
  src/trajectory_planner_node.cpp
)

target_include_directories(px4ctrl_node PRIVATE
  src
)

target_include_directories(test_attitude_mode_node PRIVATE
  src
)

target_include_directories(trajectory_planner_node PRIVATE
  src
)

target_link_libraries(px4ctrl_node Eigen3::Eigen)
target_link_libraries(test_attitude_mode_node Eigen3::Eigen)
target_link_libraries(trajectory_planner_node Eigen3::Eigen)

ament_target_dependencies(px4ctrl_node
  geometry_msgs
  mavros
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  rclcpp
  sensor_msgs
  std_msgs
  uav_utils
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(physical_setpoint_protocol_test
    test/physical_setpoint_protocol_test.cpp
    src/physical_setpoint_protocol.cpp
  )
  target_include_directories(physical_setpoint_protocol_test PRIVATE src)
  ament_add_gtest(controller_ude_test
    test/controller_ude_test.cpp
    src/controller.cpp
  )
  target_include_directories(controller_ude_test PRIVATE src)
  target_link_libraries(controller_ude_test Eigen3::Eigen)
  ament_target_dependencies(controller_ude_test
    geometry_msgs
    mavros_msgs
    nav_msgs
    quadrotor_msgs
    rclcpp
    sensor_msgs
    std_msgs
    uav_utils
  )
  ament_add_gtest(frame_transform_test
    test/frame_transform_test.cpp
  )
  target_link_libraries(frame_transform_test Eigen3::Eigen)
  ament_target_dependencies(frame_transform_test
    geometry_msgs
    mavros
    rclcpp
    sensor_msgs
  )
  ament_add_pytest_test(bspline_trajectory_test
    test/test_bspline_trajectory.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
endif()

ament_target_dependencies(test_attitude_mode_node
  geometry_msgs
  mavros_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(trajectory_planner_node
  geometry_msgs
  quadrotor_msgs
  rclcpp
  uav_utils
)

install(TARGETS
  px4ctrl_node
  test_attitude_mode_node
  trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/analyze_physical_wls_bag.py
  scripts/analyze_bspline_tracking_bag.py
  scripts/auto_grasp_place_dataset.py
  scripts/auto_hls_grasp_place.py
  scripts/bspline_tracking_test.py
  scripts/bspline_trajectory.py
  scripts/feetech_gripper_node.py
  scripts/fly_x_gripper_test.py
  scripts/gripper_soft_grasp_lib.py
  scripts/handheld_hls_grasp_test.py
  scripts/topic_liveness_check.py
  scripts/handheld_grasp_tune.py
  scripts/ude_takeoff_hover_test.py
  scripts/vrpn_to_mavros_vision_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
