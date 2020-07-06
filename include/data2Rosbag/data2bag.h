#ifndef DATABAG_H
#define DATABAG_H
#include "data2bagbase.h"

class Data2bag : public Data2bagBase {
 public:
  Data2bag(const std::string& bag_name, const std::string& directory)
      : Data2bagBase(bag_name), directory_(directory) {
    ROS_INFO_STREAM(__FUNCTION__);
    if (directory_.back() != '/') directory_ += "/";
  }
  bool AddFile() {
    if (!this->loadData<geometry_msgs::PoseWithCovarianceStamped>(
            directory_ + "tag.txt", "tag_info", "tagframe",
            true, JPL)) {
      ROS_INFO_STREAM("load data failure, file path:"
                      << directory_ + "tag.txt"
                      << " topic name: "
                      << "tag_info");
    }
    if (!this->loadData<nav_msgs::Odometry>(directory_ + "odom_2dpose.txt",
                                            "odom_info", "world", false, JPL)) {
      ROS_INFO_STREAM("load data failure, file path:"
                      << directory_ + "odom_2dpose.txt"
                      << " topic name: "
                      << "odom_info");
    }
    return true;
  };

 private:
  std::string directory_;
};

#endif