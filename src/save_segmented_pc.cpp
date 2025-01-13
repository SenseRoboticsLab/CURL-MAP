//
// Created by zkc on 02/05/23.
//
#include "iostream"
#include "sensor_msgs/PointCloud2.h"
#include "types.h"
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <visualization_msgs/Marker.h>

int frame_num = 0;
bool is_save_segments;
std::string segments_dir;
std::string seq;

pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>());
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    std::cout << "callback" << std::endl;
    cloud_in->clear();
    pcl::fromROSMsg(*msg, *cloud_in);
    std::cout << cloud_in->size() << std::endl;
    Eigen::MatrixXd eigen_clouds(cloud_in->size(), 3);
    Eigen::MatrixXd semantics(cloud_in->size(), 1);
    for (int i = 0; i < cloud_in->size(); ++i) {
        eigen_clouds(i, 0) = cloud_in->points[i].x;
        eigen_clouds(i, 1) = cloud_in->points[i].y;
        eigen_clouds(i, 2) = cloud_in->points[i].z;
        semantics(i, 0) = cloud_in->points[i].intensity;
    }
    if (is_save_segments) {
        struct stat info;
        std::string scans_dir = segments_dir + "/" + seq + "/scans_txt/";
        std::string semantics_dir = segments_dir + "/" + seq + "/semantics_txt/";
        std::cout << scans_dir << std::endl;
        if (stat(scans_dir.c_str(), &info) != 0) {
            boost::filesystem::create_directories(scans_dir);
        }
        if (stat(semantics_dir.c_str(), &info) != 0) {
            boost::filesystem::create_directories(semantics_dir);
        }
        FileReaderBase::write_txt_file(scans_dir + std::to_string(frame_num) + ".txt", eigen_clouds);
        FileReaderBase::write_txt_file(semantics_dir + std::to_string(frame_num) + ".txt", semantics);
    }
    ++frame_num;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "save_segmented_pc");
    ros::NodeHandle nh;
    // get parameters
    nh.param<bool>("/save_results/is_save_segments", is_save_segments, false);
    nh.param<std::string>("/save_results/segments_dir", segments_dir,
                          "/media/zkc/Extreme/omniverse_issac/lidar_simulator/kitti");
    nh.param<std::string>("/seq", seq, "00");
    // subscriber
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/travel/segmented_pc", 10, callback);
    ros::spin();
}