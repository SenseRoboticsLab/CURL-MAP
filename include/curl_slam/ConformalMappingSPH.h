//
// Created by zkc on 05/05/23.
//

#ifndef SRC_CONFORMALMAPPINGSPH_H
#define SRC_CONFORMALMAPPINGSPH_H
#include "CurlGeometry.h"
#include "PatchProcession.h"
#include "Timer.h"
#include "convex_hull_2d_methods.h"
#include "curl_tools_light.h"
#include "delaunay_triangulation.h"
#include "iostream"
#include "light_structure.h"
#include "linear_interpolation_2.h"
#include "linear_interpolation_2_func.h"
#include "load_config.h"
#include "lock_manager.h"
#include "map_attributes_aabb_tree.h"
#include "sensor_msgs/PointCloud2.h"
#include "types.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/math/tools/minima.hpp>
#include <cmath>
#include <complex>
#include <deque>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <mutex>
#include <nanoflann.hpp>
#include <numeric>
#include <omp.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <thread>
#include <visualization_msgs/Marker.h>

template <typename BasicType> class ConformalMappingSPH {

  private:
    ros::Publisher pub_pt_valid_patches;
    ros::Publisher pub_landmark_cloud;
    ros::Publisher pub_landmark_patch;
    ros::Subscriber segment_pt_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> rt_segment_pt_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> rt_ground_pt_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> rt_non_ground_pt_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_ptr;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> non_ground_sync_ptr;

    std::queue<sensor_msgs::PointCloud2::ConstPtr> seg_cloud_queue;
    std::mutex segment_pt_queue_lock;
    pcl::PointCloud<pcl::PointXYZI> landmark_cloud;

    std::shared_ptr<CONFORMAL_MAPPING_CONFIG> conformal_mapping_config_ptr;
    std::shared_ptr<SH_TABLE_CONFIG<BasicType>> SH_table_config_ptr;
    std::shared_ptr<DIRECT_METHOD_CONFIG> direct_method_config_ptr;
    std::shared_ptr<DEBUG_CONFIG> debug_config_ptr;
    std::shared_ptr<AABB_CONFIG> aabb_config_ptr;
    pcl::PointCloud<PointT>::Ptr seg_cloud_ptr;
    pcl::PointCloud<PointT>::Ptr ground_cloud_ptr;
    std::vector<std::vector<int>> seg_patches_idx;

  public:
    typedef BasicType Basic_Type;

    void divide_patches(Eigen::MatrixXf &patch, std::vector<Eigen::MatrixXf> &patch_vec, double cut_threshold,
                        int minimum_size = 10);
    template <typename T>
    void radius_outlier_removal(typename pcl::PointCloud<T>::Ptr cloud_in_ptr, float radius_per_meter,
                                int min_neighbours);
    Eigen::MatrixXf radius_outlier_removal(const Eigen::MatrixXf &cloud_in, float radius_per_meter, int min_neighbours);
    BasicType IQR_thres(std::vector<BasicType> &vec, BasicType scale_factor = 1.5);
    BasicType hist_thres(std::vector<BasicType> &vec, BasicType percentage);
    std::pair<BasicType, int> hist_fall_thres(std::vector<BasicType> &vec, BasicType percentage);
    void cotangent_laplacian(std::vector<int> &f_filtered, const std::vector<int> &valid_face_idx,
                             Ubpa::HEMeshInherit &mesh_half_edge, Eigen::VectorX<BasicType> &valid_area,
                             Eigen::SparseMatrix<std::complex<BasicType>, Eigen::RowMajor> &L);
    void beltrami_coefficient(const Eigen::MatrixX<BasicType> &eigen_patch_local, std::vector<int> &f_filtered,
                              const std::vector<int> &valid_face_idx, const std::vector<int> &valid_vertex_idx,
                              Eigen::VectorX<BasicType> &disk_area,
                              const Eigen::Matrix<BasicType, 2, Eigen::Dynamic> &v,
                              Eigen::VectorX<std::complex<BasicType>> &mu);
    void generalized_laplacian(std::vector<int> &f_filtered, const std::vector<int> &valid_face_idx,
                               Eigen::VectorX<BasicType> &disk_area,
                               const Eigen::Matrix<BasicType, 2, Eigen::Dynamic> &v,
                               const Eigen::VectorX<std::complex<BasicType>> &mu,
                               Eigen::SparseMatrix<BasicType, Eigen::RowMajor> &Ax);

  public:
    // constructor for odometry
    ConformalMappingSPH(ros::NodeHandle *nh, std::shared_ptr<CONFORMAL_MAPPING_CONFIG> _conformal_mapping_config_ptr,
                        std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                        std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                        std::shared_ptr<AABB_CONFIG> _aabb_config_ptr, std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr);
    // constructor for map generation
    ConformalMappingSPH(std::shared_ptr<CONFORMAL_MAPPING_CONFIG> _conformal_mapping_config_ptr,
                        std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                        std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                        std::shared_ptr<AABB_CONFIG> _aabb_config_ptr, std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr);
    ~ConformalMappingSPH();
    void callback_point_cloud_segments(const sensor_msgs::PointCloud2::ConstPtr &msg);
    bool preprocessing();
    void conformal_mapping();

  public:
    void get_local_coordinate(Eigen::MatrixXf &eigen_patch, Eigen::Isometry3d &T_obj_lidar);
    void get_ground_local_coordinate(Eigen::MatrixXf &eigen_patch, const Eigen::Matrix3d &eigen_rot,
                                     Eigen::Isometry3d &T_obj_lidar);
    void projection(Eigen::MatrixXf &eigen_patch, Eigen::Isometry3d &T_obj_lidar,
                    Eigen::MatrixX<BasicType> &eigen_patch_local);
    void perspective_view(const Eigen::MatrixX<BasicType> &eigen_patch_local, const Eigen::Vector3<BasicType> &centroid,
                          Eigen::MatrixX<BasicType> &projection_2d_pts);
    bool triangulation_patch(const Eigen::MatrixX<BasicType> &eigen_patch_local,
                             const Eigen::MatrixX<BasicType> &projection_2d_pts,
                             Eigen::Matrix<int, Eigen::Dynamic, 3> &f, Ubpa::HEMeshInherit &mesh_half_edge,
                             std::vector<int> &valid_face_idx, std::vector<int> &valid_vertex_idx,
                             std::vector<int> &boundary_idx, std::vector<int> &f_filtered,
                             std::vector<BasicType> &boundary_length_segment, BasicType &boundary_length);
    void corner_selection(const Eigen::MatrixX<BasicType> &projection_2d_pts, std::vector<int> &valid_vertex_idx,
                          const std::vector<int> &boundary_idx, const BasicType &boundary_length,
                          Ubpa::HEMeshInherit &mesh_half_edge, std::vector<int> &corner_idx);
    bool rectangular_conformal_map(const Eigen::MatrixX<BasicType> &eigen_patch_local, std::vector<int> &boundary_idx,
                                   const std::vector<int> &corner_idx, std::vector<BasicType> &boundary_length_segment,
                                   const BasicType &boundary_length, const std::vector<int> &valid_vertex_idx,
                                   std::vector<int> &f_filtered, const std::vector<int> &valid_face_idx,
                                   Ubpa::HEMeshInherit &mesh_half_edge, Eigen::VectorX<BasicType> &valid_area,
                                   Eigen::VectorX<BasicType> &disk_area, Eigen::VectorX<BasicType> &square_x,
                                   Eigen::VectorX<BasicType> &square_y);
    int conformal_interpolation_extraction(const Eigen::VectorX<BasicType> &square_x,
                                           const Eigen::VectorX<BasicType> &square_y,
                                           const std::vector<int> &valid_vertex_idx,
                                           const Eigen::MatrixX<BasicType> &eigen_patch_local,
                                           Eigen::VectorX<BasicType> &sph_coeff, int degree, bool is_ground = false);

    /************************ real time ************************/
    /************************ real time ************************/
    /************************ real time ************************/
  public:
    // preprocessed point cloud
    std::mutex preprocessing_queue_lock;
    std::queue<std::vector<Eigen::MatrixX<BasicType>>> point_cloud_queue;
    std::queue<PointCloudInfo> point_cloud_info_queue;
    // map data
    omp_lock_t writelock;
    std::vector<Eigen::Isometry3d> keyframe_poses;
    std::vector<std::vector<std::pair<interp_func_pair, interp_func_pair>>> forward_conformal_mapping;
    std::vector<std::vector<std::pair<interp_func_pair, interp_func_pair>>> inverse_conformal_mapping;

    std::shared_ptr<MapAttributes<BasicType>> map_attributes_ptr;

    //    struct MAP_ATTRIBUTES {
    //        Eigen::Isometry3d T_lidar_w;
    //        Eigen::Isometry3d T_obj_lidar;
    //        std::vector<std::pair<interp_func_pair, interp_func_pair>> forward_conformal_mapping_vec;
    //        std::vector<std::pair<interp_func_pair, interp_func_pair>> inverse_conformal_mapping_vec;
    //    };

  public:
    void rt_callback_point_cloud_segments(const sensor_msgs::PointCloud2::ConstPtr &segment_msg,
                                          const sensor_msgs::PointCloud2::ConstPtr &ground_msg);
    void rt_conformal_mapping(std::vector<Eigen::MatrixX<BasicType>> &patches_local_vec,
                              PointCloudInfo &point_cloud_info, Eigen::Isometry3d &T_w_lidar);
    void rt_publish_landmark_cloud(const std::vector<Eigen::MatrixX<BasicType>> &landmark_cloud_vec);
    void rt_publish_landmark_patch_recons(std::shared_ptr<PatchInfo<BasicType>> &patch_info_ptr);
    visualization_msgs::Marker set_default_marker(const std::string &frame_id, const std::string &ns, const int &id,
                                                  const std::array<float, 3> &color,
                                                  int32_t type = visualization_msgs::Marker::POINTS,
                                                  const float scale[3] = (float[3]){0.1, 0.1, 0.1},
                                                  const float position[3] = (float[3]){0, 0, 0},
                                                  const std::string &text = "");
    /************************ map generation ************************/
    /************************ map generation ************************/
    /************************ map generation ************************/
    void map_gen_conformal_mapping(std::vector<Eigen::Isometry3d> T_w_lidar_vec,
                                   std::vector<std::vector<Eigen::Isometry3d>> T_obj_lidar_vec_vec,
                                   std::vector<std::vector<bool>> is_ground_cloud_vec_vec,
                                   std::vector<std::vector<Eigen::MatrixX<BasicType>>> local_point_cloud_vec_vec,
                                   BasicType outlier_patch_thres);

    bool new_landmark_add_judge(const unsigned int &key, const double &IoU,
                                const std::vector<double> &query_lower_bound,
                                const std::vector<double> &query_upper_bound,
                                std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr, const Eigen::Isometry3d T_w_j,
                                const Eigen::Isometry3d &T_obj_lidar, BasicType outlier_patch_thres);
};

#endif // SRC_CONFORMALMAPPINGSPH_H
