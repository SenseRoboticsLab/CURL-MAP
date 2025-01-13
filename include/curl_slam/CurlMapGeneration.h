// Created by zkc on 18/08/23.
//

#ifndef SRC_CURLMAPGENERATION_H
#define SRC_CURLMAPGENERATION_H
#include "ConformalMappingSPH.h"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "light_structure.h"
#include "map_attributes_aabb_tree.h"
#include "open3d/Open3D.h"
#include "types.h"
#include <chrono>
#include <curl_slam/MapGenerationConfig.h>
#include <dynamic_reconfigure/server.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <omp.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sstream>
#include <thread>

template <typename BasicType> class CurlMapGeneration {
  private:
    // parameters
    MAP_GENERATION_CONFIG current_map_generation_config;
    std::shared_ptr<MAP_GENERATION_CONFIG> map_generation_config_ptr;
    std::shared_ptr<SH_TABLE_CONFIG<BT>> SH_table_config_ptr;
    std::shared_ptr<DIRECT_METHOD_CONFIG> direct_method_config_ptr;
    std::shared_ptr<DEBUG_CONFIG> debug_config_ptr;
    // dynamic parameters
    dynamic_reconfigure::Server<curl_slam::MapGenerationConfig> server;
    dynamic_reconfigure::Server<curl_slam::MapGenerationConfig>::CallbackType f;
    // conformal mapping class
    std::shared_ptr<ConformalMappingSPH<BasicType>> conformal_mapping_ptr;
    // publisher
    ros::Publisher pub_seg_cloud;
    ros::Publisher pub_ground_cloud;
    // informations
    int invalid_counter = 0;
    bool has_ground_info;
    bool has_bounding_box_info;
    bool has_data_asso_times;
    BasicType outlier_patch_thres;
    open3d::visualization::Visualizer open3d_vis;
    std::shared_ptr<open3d::geometry::PointCloud> open3d_display_pcd_ptr;
    Eigen::Vector3d open3d_display_pcd_center;

    std::mutex update_open3d_points_lock;
    std::condition_variable update_open3d_points_cv;
    // lock for parallel reconstruction
    omp_lock_t writelock;
    // strore trajectories for open3d tracking
    std::vector<Eigen::Matrix4d> T_w_j_vec;
    unsigned int trajectories_num;
    unsigned int tracking_idx;
    Eigen::Matrix4d T_w_j_1;
    bool update_flag;

  public:
    CurlMapGeneration(ros::NodeHandle *nh, std::shared_ptr<MAP_GENERATION_CONFIG> _map_generation_config_ptr,
                      std::shared_ptr<SH_TABLE_CONFIG<BT>> _SH_table_config_ptr,
                      std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                      std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                      std::shared_ptr<ConformalMappingSPH<BasicType>> _conformal_mapping_ptr);

    void run();
    void open3d_display_thread();
    void param_callback(curl_slam::MapGenerationConfig &config, uint32_t level);
    bool map_initialization();
    void save_final_coefficients_map();
    bool map_loader(const std::string &filename, bool is_aabb_map);
    void save_whole_points();
};

#endif // SRC_CURLMAPGENERATION_H
