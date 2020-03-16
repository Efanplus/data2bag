#include <string>
#include <vector>
#include <rosbag/bag.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
// #include <Eigen/Dense>
// #include <Eigen/Core>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cassert>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <memory>

// #include "csv.h"
using namespace std;
class SegwayRobotDatasetReader {
public:
    const std::string POSE_TOPIC = "/pose_info";
    const std::string IMU_TOPIC = "/imu_info";
    // const std::string ENCODER_TOPIC = "/encoder";


    SegwayRobotDatasetReader() {}

    void setDataPath(std::string path) {
        datasetPath_ = path;
    }

    explicit SegwayRobotDatasetReader(const std::string& datasetPath)
            : datasetPath_(datasetPath){
    }
    string replace(string original, char o, char n){
        string res = original;
        for(int i = 0; i < res.length(); i ++){
            if(res[i] == o)res[i] = n;
        }
        return res;
    }
    ros::Time timesamp2rostime(int64_t timesamp){
        std::string suanz = std::to_string(timesamp);
        std::string sec_string = suanz.substr(0,10);
        std::string nsec_string = suanz.substr(10,9);
        while(nsec_string.length() < 9){
            nsec_string += "0";
        }
        return ros::Time(std::stoi(sec_string),std::stoi(nsec_string));
    }
    bool readData(bool use_imu=true) {

        data_bag_file_name_ = datasetPath_ + "/imu_Rpose.bag";

        std::cout << "Saving bag to :" << data_bag_file_name_ << std::endl;
        data_bag_ptr_ =  std::make_shared<rosbag::Bag>(data_bag_file_name_,
                                                       rosbag::bagmode::Read|rosbag::bagmode::Write);

        /// load fisheye
        std::string PosewithCovariancePath = datasetPath_ + "/pose.txt";
        if (!loadPosewithCovariance(PosewithCovariancePath, POSE_TOPIC)) {
            std::cout << "Can Not load PosewithCovariance data!" << std::endl;
            return false;
        }

        /// load imu
        std::string imuListPath = datasetPath_ + "/imu.txt";
        if (use_imu) {
            if (!loadIMUReadings(imuListPath, IMU_TOPIC)) {
                std::cout << "Can Not load imu data!" << std::endl;
                return false;

            }
        }



        std::cout<< "data bag size: " << data_bag_ptr_->getSize() << std::endl;
        data_bag_ptr_->close();

        return true;
    }

    bool loadPosewithCovariance(const std::string list_file, 
                                const std::string topic, 
                                const bool covariance_flag = false) {
        std::ifstream ifs(list_file);
        if (!ifs.is_open()) {
            std::cerr << "Failed to open PosewithCovariance file: " << list_file
                      << std::endl;
            return false;
        }
        bool first_msg = true;
        ros::Time last_timestamp = ros::Time(0);
        ros::Time cur_ts = ros::Time(0);
        std::string one_line;
        int pose_c_seq = 0;
        std::ifstream infile;
        infile.open(list_file.data());
        std::string s;
        while(getline(infile, s)){
            if(s[0] == '#')continue;
            s = replace(s, ',', ' ');
            istringstream is(s);
            int64_t time_pose;
            geometry_msgs::PoseWithCovarianceStamped pose_c;
            
            s.clear();
            is >> time_pose;
            is >> pose_c.pose.pose.position.x ;
            is >> pose_c.pose.pose.position.y ;
            is >> pose_c.pose.pose.position.z ;
            is >> pose_c.pose.pose.orientation.w;
            is >> pose_c.pose.pose.orientation.x;
            is >> pose_c.pose.pose.orientation.y;
            is >> pose_c.pose.pose.orientation.z;
            
            if(covariance_flag){
                //dosomething
            }
            pose_c.header.seq = pose_c_seq;
            pose_c.header.stamp = timesamp2rostime(time_pose);
            data_bag_ptr_->write(topic, pose_c.header.stamp, pose_c);
            pose_c_seq ++;
        }

        end_times_.push_back(cur_ts);

        // end_times_.push_back(cur_ts);
        ifs.close();
        std::cout << "load " << topic << " data from: "
                  << start_times_.back() << " to " << end_times_.back() << " total " << pose_c_seq
                  << " with duration: " << end_times_.back().toSec() - start_times_.back().toSec()
                  << " s."<< " About: " << pose_c_seq/(end_times_.back().toSec() - start_times_.back().toSec()) << "Hz" << std::endl;;
        return true;
    }


    bool loadIMUReadings(const std::string list_file, const std::string topic) {

        bool first_msg = true;
        ros::Time last_timestamp = ros::Time(0);
        ros::Time cur_ts = ros::Time(0);
        std::string one_line;
        int imu_seq = 0;
        std::ifstream infile;
        infile.open(list_file.data());
        std::string s;
        while(getline(infile, s)){
            if(s[0] == '#')continue;
            s = replace(s, ',', ' ');
            istringstream is(s);
            int64_t time_imu;
            sensor_msgs::Imu imu;
            s.clear();
            is >> time_imu;
            is >> imu.angular_velocity.x;
            is >> imu.angular_velocity.y;
            is >> imu.angular_velocity.z;            
            is >> imu.linear_acceleration.x;
            is >> imu.linear_acceleration.y;
            is >> imu.linear_acceleration.z; 
            imu.header.seq = imu_seq;
            imu.header.stamp = timesamp2rostime(time_imu);
            data_bag_ptr_->write(topic, imu.header.stamp, imu);
            imu_seq ++;
        }

        end_times_.push_back(cur_ts);

        std::cout << "load " << topic << " data from: "
                  << start_times_.back() << " to " << end_times_.back() << " total " << imu_seq
                  << " with duration: " << end_times_.back().toSec() - start_times_.back().toSec()
                  << " s."<< " About: " << imu_seq/(end_times_.back().toSec() - start_times_.back().toSec()) << "Hz" << std::endl;
        return true;
    }


    inline std::string getDataBagFileName() {
        return data_bag_file_name_;
    }
    inline std::shared_ptr<rosbag::Bag> getDataBagPtr() const {
        return data_bag_ptr_;
    }

    inline ros::Time getBagStartTs() {
        return bag_start_ts_;
    }

    inline ros::Time getBagEndTs() {
        return bag_end_ts_;
    }





private:
    std::string datasetPath_;
    std::string intermediate_bag_out_folder_;
    std::string data_bag_file_name_;
    std::shared_ptr<rosbag::Bag> data_bag_ptr_;
    std::vector<ros::Time> start_times_, end_times_;
    ros::Time bag_start_ts_, bag_end_ts_;


};


int main (int argc, char ** argv) {

    std::string dataset_folder_path;
    std::string temp_folder;
    if (argc ==2 ) {
        dataset_folder_path = std::string(argv[1]);
        temp_folder = dataset_folder_path;
    } else {
        std::cerr << "Usage: test_data_source dataset_path [temp_folder]"  << std::endl;
        return -1;
    }

    SegwayRobotDatasetReader datasetReader(dataset_folder_path);
    datasetReader.readData(true);

    return 0;

}