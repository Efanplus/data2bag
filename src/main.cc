#include <iostream>
#include <vector>

#include "data2Rosbag/data2bag.h"
int main(int argc, char** argv) {
  ros::init(argc, argv, "data2Rosbag");
  std::string directory = argv[1];
  if (directory.empty() || directory == "/") {
    std::cerr << "directory is wrong!" << std::endl;
  }
  if (directory.back() == '/') {
    directory = directory.substr(0, directory.size() - 1);
  }
  
  std::size_t index = directory.find_last_of('/');
  std::string bag_name = "odom_tag_" + directory.substr(index + 1) + ".bag";
  std::cout << directory << "," << bag_name << std::endl;
  Data2bag data2bag(bag_name, directory);
  data2bag.AddFile();
  return 1;
}