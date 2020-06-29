#include <iostream>
#include <vector>

#include "data2Rosbag/data2bag.h"
int main(int argc, char** argv) {
  ros::init(argc, argv, "data2Rosbag");
  //   for (int i = 1; i < argc; i++) {
  //     std::cout << i << argv[i] << std::endl;
  //   }
  const std::string bag_name = argv[1];
  Data2bag data2bag(bag_name);
  // std::cout << "please input the type of the quaternion(xyzw or wxyz)!\n";
  std::cout << "please input the param, 's' to make rosbag, 'b' to end\n";
  std::string order_suanz;
  std::cin >> order_suanz;
  while (order_suanz == "s") {
    std::string type_name;
    std::string file_path;
    std::string topic_name;
    std::string frame_id;
    std::string quaternion_type;
    std::cout << "please input the type_name(imu, pose,point,posec, odom)\n";
    std::cin >> type_name;
    std::cout << "please input the file_path\n";
    std::cin >> file_path;
    std::cout << "please input the topic_name\n";
    std::cin >> topic_name;
    std::cout << "please input the frame_id\n";
    std::cin >> frame_id;
    std::cout << "please select the quaternion type, J or H\n";
    std::cin >> quaternion_type;
    int quaternion_enum;
    quaternion_enum =
        quaternion_type == "J" ? Data2bag::JPL : Data2bag::Hamilton;
    if (type_name == "imu") {
      if (data2bag.AddFile<sensor_msgs::Imu>(file_path, topic_name, frame_id,
                                             quaternion_enum)) {
        std::cout << "load file from " << file_path << " to topic "
                  << topic_name << " success!\n";
      } else {
        std::cout << "load file from " << file_path << " to topic "
                  << topic_name << " failed!\n";
      }
    } else if (type_name == "pose") {
      if (data2bag.AddFile<geometry_msgs::PoseStamped>(
              file_path, topic_name, frame_id, quaternion_enum)) {
        std::cout << "load file from " << file_path << " to topic "
                  << topic_name << " success!\n";
      } else {
        std::cout << "load file from " << file_path << " to topic "
                  << topic_name << " failed!\n";
      }
    } else if (type_name == "point") {
      if (data2bag.AddFile<geometry_msgs::PointStamped>(
              file_path, topic_name, frame_id, quaternion_enum)) {
        std::cout << "load file from " << file_path << " to topic "
                  << topic_name << " success!\n";
      } else {
        std::cout << "load file from " << file_path << " to topic "
                  << topic_name << " failed!\n";
      }
    } else if (type_name == "posec") {
      if (data2bag.AddFile<geometry_msgs::PoseWithCovarianceStamped>(
              file_path, topic_name, frame_id, quaternion_enum)) {
        std::cout << "load file from " << file_path << " to topic "
                  << topic_name << " success!\n";
      } else {
        std::cout << "load file from " << file_path << " to topic "
                  << topic_name << " failed!\n";
      }
    } else if (type_name == "odom") {
      if (data2bag.AddFile<nav_msgs::Odometry>(file_path, topic_name, frame_id,
                                               quaternion_enum)) {
        std::cout << "load file from " << file_path << " to topic "
                  << topic_name << " success!\n";
      } else {
        std::cout << "load file from " << file_path << " to topic "
                  << topic_name << " failed!\n";
      }
    }
    std::cout << "please input the param, 's' to make rosbag, 'b' to end\n";
    std::cin >> order_suanz;
  }
  return 1;
}