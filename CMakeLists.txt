cmake_minimum_required(VERSION 2.8.3)
project(kinect_pyuarm)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}:/usr/local/include/libfreenect2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(OpenCV REQUIRED)
find_package(freenect2 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${freenect2_INCLUDE_DIRS})

add_executable(kinect_streamer src/kinect_streamer.cpp)
target_link_libraries(kinect_streamer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${freenect2_LIBS}
)