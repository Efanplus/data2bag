cmake_minimum_required(VERSION 2.8.3)
project(data2rosbag)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  rosbag
  gflags_catkin
)
# find_package( REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)
catkin_package()

add_executable(data2rosbag src/main.cc)
target_link_libraries(data2rosbag ${catkin_LIBRARIES})