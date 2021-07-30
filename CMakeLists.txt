cmake_minimum_required(VERSION 2.8.3)
project(irtifayer)

find_package(catkin REQUIRED COMPONENTS
  mavros
  std_msgs
  roscpp
  cv_bridge
  image_transport
  tf
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(vekil src/vekil.cpp)
target_link_libraries(vekil ${catkin_LIBRARIES})
target_link_libraries(vekil ${OpenCV_LIBRARIES})
