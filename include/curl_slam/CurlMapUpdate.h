//
// Created by zkc on 19/07/23.
//

#ifndef SRC_CURLMAPUPDATE_H
#define SRC_CURLMAPUPDATE_H

#include "ConformalMappingSPH.h"
#include "Timer.h"
#include "light_structure.h"
#include "types.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <mutex>
#include <nav_msgs/Path.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <queue>
#include <ros/ros.h>

template <typename BasicType> class CurlMapUpdate {
  private:
    std::shared_ptr<CURL_TRACKING_CONFIG> curl_tracking_config_ptr;
    std::shared_ptr<DEBUG_CONFIG> debug_config_ptr;
    std::shared_ptr<ConformalMappingSPH<BasicType>> conformal_mapping_ptr;
    // ros
    ros::Subscriber v_local_sub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> landmark_cloud_sub;
    message_filters::Subscriber<nav_msgs::Path> landmark_T_obj_lidar_set_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> landmark_T_w_j_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Path,
                                                      geometry_msgs::PoseStamped>
        MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;

  public:
    CurlMapUpdate() = delete;
    CurlMapUpdate(ros::NodeHandle *nh, std::shared_ptr<CURL_TRACKING_CONFIG> _curl_tracking_config_ptr,
                  std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                  std::shared_ptr<ConformalMappingSPH<BasicType>> _conformal_mapping_ptr);
    void update_sph_coeff_callback(const sensor_msgs::PointCloud2::ConstPtr &v_obj_msg);
    void add_landmark_call_back(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                                const nav_msgs::Path::ConstPtr &path_msg,
                                const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
};

#endif // SRC_CURLMAPUPDATE_H
