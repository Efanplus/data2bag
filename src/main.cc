#include <iostream>
#include <vector>

#include "data2Rosbag/data2bag.h"
int main(int argc, char** argv) {
  ros::init(argc, argv, "data2Rosbag");
//   for (int i = 1; i < argc; i++) {
//     std::cout << i << argv[i] << std::endl;
//   }
  if (argc > 4) {
    const std::string bag_name = argv[1];
    Data2bag data2bag(bag_name);
    std::vector<int> index;
    for (int k = 2; k < argc; k++) {
      if (std::string(argv[k]) == "-n") {
        index.push_back(k);
      }
    }
    for (int i = 0; i < index.size(); i++) {
      int start_index = index[i];
      int end_index = i + 1 < index.size() ? index[i + 1] : argc;
      std::cout << start_index << "," << end_index << std::endl;
      if (end_index - start_index == 4 || end_index - start_index == 5) {
        const std::string type_name(argv[start_index + 1]);
        const std::string file_path(argv[start_index + 2]);
        const std::string topic_name(argv[start_index + 3]);
        std::string frame_id;
        if (end_index - start_index == 5)
          frame_id = std::string(argv[start_index + 4]);
        std::cout << file_path << "," << topic_name << "," << frame_id
                  << std::endl;
        if (type_name == "imu") {
          if (data2bag.AddFile<sensor_msgs::Imu>(file_path, topic_name,
                                                 frame_id)) {
            std::cout << "load file from " << file_path << " to topic "
                      << topic_name << " success!\n";
          } else {
            std::cout << "load file from " << file_path << " to topic "
                      << topic_name << " failed!\n";
          }
        } else if (type_name == "pose") {
          if (data2bag.AddFile<geometry_msgs::PoseStamped>(
                  file_path, topic_name, frame_id)) {
            std::cout << "load file from " << file_path << " to topic "
                      << topic_name << " success!\n";
          } else {
            std::cout << "load file from " << file_path << " to topic "
                      << topic_name << " failed!\n";
          }
        } else if (type_name == "point") {
          if (data2bag.AddFile<geometry_msgs::PointStamped>(
                  file_path, topic_name, frame_id)) {
            std::cout << "load file from " << file_path << " to topic "
                      << topic_name << " success!\n";
          } else {
            std::cout << "load file from " << file_path << " to topic "
                      << topic_name << " failed!\n";
          }
        } else if (type_name == "posec") {
          if (data2bag.AddFile<geometry_msgs::PoseWithCovarianceStamped>(
                  file_path, topic_name, frame_id)) {
            std::cout << "load file from " << file_path << " to topic "
                      << topic_name << " success!\n";
          } else {
            std::cout << "load file from " << file_path << " to topic "
                      << topic_name << " failed!\n";
          }
        } else if (type_name == "odom") {
          if (data2bag.AddFile<nav_msgs::Odometry>(file_path, topic_name,
                                                   frame_id)) {
            std::cout << "load file from " << file_path << " to topic "
                      << topic_name << " success!\n";
          } else {
            std::cout << "load file from " << file_path << " to topic "
                      << topic_name << " failed!\n";
          }
        }
      }
    }
  }
  return 1;
}