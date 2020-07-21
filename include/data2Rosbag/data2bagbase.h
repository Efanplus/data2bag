#ifndef DATA2BAGBASE_H
#define DATA2BAGBASE_H
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Header.h>
#include <std_msgs/Time.h>

#include <string>
#include <vector>
// #include <Eigen/Dense>
// #include <Eigen/Core>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class Data2bagBase {
 public:
  enum { JPL, Hamilton };
  Data2bagBase(const std::string& data_bag_file_name)
      : data_bag_file_name_(data_bag_file_name) {
    ROS_INFO_STREAM(__FUNCTION__ << " data_bag file name: "
                                 << data_bag_file_name_);
    data_bag_ptr_ = std::make_shared<rosbag::Bag>(
        data_bag_file_name_, rosbag::bagmode::Read | rosbag::bagmode::Write);
    cur_ts_ = ros::Time(0);
  }
  ~Data2bagBase() { data_bag_ptr_->close(); }
  std::string replace(std::string& original, char o, char n) {
    std::string res = original;
    for (int i = 0; i < res.length(); i++) {
      if (res[i] == o) res[i] = n;
    }
    return res;
  }
  ros::Time timesamp2rostime(int64_t timesamp) {
    std::string suanz = std::to_string(timesamp);
    std::string sec_string = suanz.substr(0, 10);
    std::string nsec_string = suanz.substr(10, 9);
    while (nsec_string.length() < 9) {
      nsec_string += "0";
    }
    return ros::Time(std::stoi(sec_string), std::stoi(nsec_string));
  }
  void GetIfstreamfromFile(std::istringstream& is, nav_msgs::Odometry& msg,
                           const bool covariance_flag = false,
                           const int& quaternion_type = Hamilton) {
    is >> msg.pose.pose.position.x;
    is >> msg.pose.pose.position.y;
    msg.pose.pose.position.z = 0;
    double theta;
    is >> theta;
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = std::sin(theta / 2);
    msg.pose.pose.orientation.w = std::cos(theta / 2);

    // is >> msg.twist.twist.linear.x;
    // is >> msg.twist.twist.linear.y;
    // is >> msg.twist.twist.linear.z;
    // is >> msg.twist.twist.angular.x;
    // is >> msg.twist.twist.angular.y;
    // is >> msg.twist.twist.angular.z;
  }
  void GetIfstreamfromFile(std::istringstream& is,
                           geometry_msgs::PoseStamped& msg,
                           const bool covariance_flag = false,
                           const int& quaternion_type = Hamilton) {
    is >> msg.pose.position.x;
    is >> msg.pose.position.y;
    is >> msg.pose.position.z;
    if (quaternion_type == JPL) {
      is >> msg.pose.orientation.x;
      is >> msg.pose.orientation.y;
      is >> msg.pose.orientation.z;
      is >> msg.pose.orientation.w;
    } else if (quaternion_type == Hamilton) {
      is >> msg.pose.orientation.w;
      is >> msg.pose.orientation.x;
      is >> msg.pose.orientation.y;
      is >> msg.pose.orientation.z;
    }
  }
  void GetIfstreamfromFile(std::istringstream& is,
                           geometry_msgs::PointStamped& msg,
                           const bool covariance_flag = false,
                           const int& quaternion_type = Hamilton) {
    is >> msg.point.x;
    is >> msg.point.y;
    is >> msg.point.z;
  }
  void GetIfstreamfromFile(std::istringstream& is,
                           geometry_msgs::PoseWithCovarianceStamped& msg,
                           const bool covariance_flag = false,
                           const int& quaternion_type = Hamilton) {
    is >> msg.pose.pose.position.x;
    is >> msg.pose.pose.position.y;
    msg.pose.pose.position.z = 0;
    double theta;
    is >> theta;
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = std::sin(theta / 2);
    msg.pose.pose.orientation.w = std::cos(theta / 2);
    if (true) {
      // double scale = 10;
      std::vector<double> covariance_2d_(9);
      for (int i = 0; i < 9; i++) {
        is >> covariance_2d_[i];
      }
      const double scale = 5;
      msg.pose.covariance[0] = std::pow(covariance_2d_[0], 1) * scale;
      msg.pose.covariance[1] = std::pow(covariance_2d_[1], 1) * scale;
      msg.pose.covariance[6] = std::pow(covariance_2d_[3], 1) * scale;
      msg.pose.covariance[7] = std::pow(covariance_2d_[4], 1) * scale;
      msg.pose.covariance[35] = std::pow(covariance_2d_[8], 1) * scale;

      msg.pose.covariance[5] = std::pow(covariance_2d_[2], 1) * scale;
      msg.pose.covariance[11] = std::pow(covariance_2d_[5], 1) * scale;
      msg.pose.covariance[30] = std::pow(covariance_2d_[6], 1) * scale;
      msg.pose.covariance[31] = std::pow(covariance_2d_[7], 1) * scale;

      msg.pose.covariance[14] = 0.01;
      msg.pose.covariance[21] = 0.01;
      msg.pose.covariance[28] = 0.01;
    }
  }
  void GetIfstreamfromFile(std::istringstream& is, sensor_msgs::Imu& msg,
                           const bool covariance_flag = false,
                           const int& quaternion_type = Hamilton) {
    is >> msg.angular_velocity.x;
    is >> msg.angular_velocity.y;
    is >> msg.angular_velocity.z;
    is >> msg.linear_acceleration.x;
    is >> msg.linear_acceleration.y;
    is >> msg.linear_acceleration.z;
  }
  template <typename MSG_TYPE>
  bool loadData(const std::string& list_file, const std::string& topic,
                const std::string& frame_name = "",
                const bool covariance_flag = false,
                const int& quaternion_type = Hamilton) {
    std::ifstream ifs(list_file);
    if (!ifs.is_open()) {
      std::cerr << "Failed to open PosewithCovariance file: " << list_file
                << std::endl;
      return false;
    }
    std::cout << "loaddata from file: " << list_file << ", topic: " << topic
              << ", frame_name:" << frame_name
              << ", quaternion_type:" << quaternion_type << std::endl;
    bool first_msg = true;
    ros::Time last_timestamp = ros::Time(0);
    std::string one_line;
    int c_seq = 0;
    std::ifstream infile;
    infile.open(list_file.data());
    std::string s;
    while (getline(infile, s)) {
      if (s[0] == '#') continue;
      s = replace(s, ',', ' ');
      std::size_t index_time = s.find_first_of(" ");
      std::string str_time = s.substr(0, index_time);
      if (str_time.size() > 10 && str_time[10] != '.') {
        // std::cout << str_time << std::endl;
        str_time = str_time.substr(0, 10) + "." + str_time.substr(10);
        // std::cout << str_time << std::endl;
        // sleep(1);
      }
      s = s.substr(index_time + 1);
      std::istringstream is(s);
      MSG_TYPE msg;

      s.clear();
      GetIfstreamfromFile(is, msg, covariance_flag, quaternion_type);

      if (covariance_flag) {
        // dosomething
      }
      msg.header.seq = c_seq;
      msg.header.frame_id = frame_name;
      // std::cout << str_time << std::endl;
      msg.header.stamp = ros::Time(std::stod(str_time));
      if (msg.header.stamp == cur_ts_) {
        ROS_WARN_STREAM("the time is same with last one, topic: " << topic);
      } else {
        data_bag_ptr_->write(topic, msg.header.stamp, msg);
      }
      cur_ts_ = msg.header.stamp;
      if (first_msg) {
        start_times_.push_back(cur_ts_);
        first_msg = false;
      }
      c_seq++;
    }

    end_times_.push_back(cur_ts_);

    // end_times_.push_back(cur_ts);
    ifs.close();
    if (c_seq == 0) {
      std::cout << "there is no data readed for this topic in this file\n";
    } else {
      std::cout << "load " << topic << " data from: " << start_times_.back()
                << " to " << end_times_.back() << " total " << c_seq
                << " with duration: "
                << end_times_.back().toSec() - start_times_.back().toSec()
                << " s."
                << " About: "
                << c_seq /
                       (end_times_.back().toSec() - start_times_.back().toSec())
                << "Hz" << std::endl;
    }
    return true;
  }

  inline std::shared_ptr<rosbag::Bag> getDataBagPtr() const {
    return data_bag_ptr_;
  }

 protected:
  std::shared_ptr<rosbag::Bag> data_bag_ptr_;
  std::vector<ros::Time> start_times_, end_times_;
  std::string data_bag_file_name_;
  ros::Time cur_ts_;
};
#endif