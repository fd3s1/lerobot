cmake_minimum_required(VERSION 2.6)

project(yolov5_trt)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g -fPIC")

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    sensor_msgs
    cv_bridge
    object_detection_msgs
    message_filters
    quadrotor_msgs
    Eigen3 REQUIRED
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES yolov5_trt
  CATKIN_DEPENDS roscpp sensor_msgs object_detection_msgs
  DEPENDS OpenCV EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

add_executable(human_info_pub ${PROJECT_SOURCE_DIR}/src/human_info_pub.cpp)
add_executable(qr_code_transform_node ${PROJECT_SOURCE_DIR}/src/qr_code_transform_node.cpp)

target_link_libraries(human_info_pub ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(qr_code_transform_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_definitions(-O2 -pthread)
