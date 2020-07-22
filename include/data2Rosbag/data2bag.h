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
    if (!this->loadData<geometry_msgs::PoseStamped>(
            directory_ + "slam_status_debug/relocal_filter.txt", "relocal_info",
            "relocalframe", false, JPL)) {
      ROS_INFO_STREAM("load data failure, file path:"
                      << directory_ + "relocal_filter.txt"
                      << " topic name: "
                      << "relocal_info");
    }
    if (!this->loadData<geometry_msgs::PoseWithCovarianceStamped>(
            directory_ + "amcl_result/amcl_pose_result.txt", "amcl_info",
            "amclframe", true, JPL)) {
      ROS_INFO_STREAM("load data failure, file path:"
                      << directory_ + "amcl_pose_result.txt"
                      << " topic name: "
                      << "amcl_info");
    }
    if (!this->loadData<nav_msgs::Odometry>(
            directory_ + "amcl_result/odom_pose_result.txt", "odom_info",
            "world", false, JPL)) {
      ROS_INFO_STREAM("load data failure, file path:"
                      << directory_ + "odom_pose_result.txt"
                      << " topic name: "
                      << "odom_info");
    }
    return true;
  };

 private:
  std::string directory_;
};

#endif