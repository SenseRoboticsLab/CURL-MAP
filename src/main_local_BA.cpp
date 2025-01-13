//
// Created by zkc on 10/03/23.
//
#include "CURL_local_BA.h"
#include "ceres/ceres.h"
#include "curl_tools_light.h"
#include "iostream"
#include "load_config.h"
#include "sensor_msgs/PointCloud2.h"
#include "types.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "main_local_BA");
    std::string config_file = argv[1];
    //    Eigen::MatrixXd dirs =
    //        FileReaderBase::read_txt_file<BT>("/home/user/project/CURL_SLAM/matlab/tmp_data/dirs.txt");
    //    Eigen::VectorXd F =
    //        FileReaderBase::read_txt_file<BT>("/home/user/project/CURL_SLAM/matlab/tmp_data/F.txt");
    //    Eigen::VectorXd sph_coeff = curl::IRF_least_square(20, F, dirs, 4, 1);
    //    std::cout << sph_coeff << std::endl;
    //
    //    SH_TABLE_CONFIG SH_table_config;
    //    load_SH_table_config(config_file, SH_table_config);
    //    curl::getSH_table(SH_table_config);
    //    sph_coeff = curl::IRF_least_square_table(20, F, dirs, 4, 1, SH_table_config);
    //    std::cout << std::endl;
    //    std::cout << sph_coeff << std::endl;
    FileReader<BT, ST> file_loader = FileReader<BT, ST>(config_file);
    add_to_queue_ref<BT, ST>(file_loader);
    CURL_local_BA<BT, ST> curl_local_ba(file_loader, config_file);
    return 0;
}