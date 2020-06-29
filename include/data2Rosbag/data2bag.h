#ifndef DATABAG_H
#define DATABAG_H
#include "data2bagbase.h"

class Data2bag : public Data2bagBase {
 public:
  Data2bag(const std::string& bag_name) : Data2bagBase(bag_name) {
    ROS_INFO_STREAM(__FUNCTION__);
  }
  template <typename MSG_TYPE>
  bool AddFile(const std::string& file_path, const std::string& topic_name,
               const std::string& frame_id,
               const int& quaternion_type = Hamilton) {
    if (!this->loadData<MSG_TYPE>(file_path, topic_name, frame_id, false,
                                  quaternion_type)) {
      ROS_INFO_STREAM("load data failure, file path:"
                      << file_path << " topic name: " << topic_name);
      return false;
    }
    return true;
  };
};

#endif