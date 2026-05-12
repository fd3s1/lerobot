cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros
  nlink_parser
  message_filters
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

add_executable(odom_pub_test src/odom_pub_test.cpp)
add_executable(cmd_pub_test src/cmd_pub_test.cpp)
add_executable(usrp_splice_6 src/UsrpSplice.cpp)
add_executable(uwb_cal src/uwb_4D_RL.cpp)
#add_executable(odom_real_pub6 src/odom_sub2pub.cpp)

# add_executable(cmd_pub_test
# src/cmd_pub_test.cpp)


add_dependencies(px4ctrl_node quadrotor_msgs)

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
)
target_link_libraries(odom_pub_test ${catkin_LIBRARIES})
target_link_libraries(cmd_pub_test ${catkin_LIBRARIES})
target_link_libraries(usrp_splice_6 ${catkin_LIBRARIES})
target_link_libraries(uwb_cal ${catkin_LIBRARIES})
#target_link_libraries(odom_real_pub6 ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS 
#thrust_calibrate_scrips/thrust_calibrate.py 
scripts/Sense.py
network_io/network_io.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
