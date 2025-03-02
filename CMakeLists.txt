cmake_minimum_required(VERSION 2.8.3)
project(testnode)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  eigen_conversions
)

find_package(PCL REQUIRED)


find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

# ROS build
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS PCL
  INCLUDE_DIRS include
)

# eCAL / proto
set(protobuf_files
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/pcl.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/pcl_header.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/pcl_pfield.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/types.proto
)
add_executable(testnode node.cpp include/PointCloudHandler.cpp)

# compile protos
PROTOBUF_TARGET_CPP(testnode ${CMAKE_CURRENT_SOURCE_DIR}/proto/ ${protobuf_files})


target_link_libraries(testnode ${catkin_LIBRARIES} ${PCL_LIBRARIES} eCAL::core protobuf::libprotobuf)

