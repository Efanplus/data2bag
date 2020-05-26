cmake_minimum_required(VERSION 2.8.3)
project(data2bag_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  rosbag
)
# find_package( REQUIRED)
catkin_package()
add_executable(data2bag_pkg src/data2bag_1.cpp)
target_link_libraries(data2bag_pkg ${catkin_LIBRARIES})
include_directories(
  ${catkin_INCLUDE_DIRS}
)


