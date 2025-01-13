//
// Created by zkc on 13/03/23.
//

#ifndef CURL_SLAM_CURL_LOCAL_BA_H
#define CURL_SLAM_CURL_LOCAL_BA_H
#include "DirectMethod.h"
#include "FileReader.h"
#include "PatchProcession.h"
#include "RosHandler.h"
#include "Timer.h"
#include "light_structure.h"
#include "optimization_types.h"
#include "types.h"
#include <Eigen/StdVector>
#include <iostream>
#include <iterator>
#include <map>
#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <utility>
#include <vector>

template <typename BasicType, typename SemanticType> class CURL_local_BA {
  public:
    typedef typename FileReader<BasicType, SemanticType>::DynamicMatrix DynamicMatrix;
    typedef typename FileReader<BasicType, SemanticType>::DynamicVector DynamicVector;
    typedef typename FileReader<BasicType, SemanticType>::DynamicMatrixSem DynamicMatrixSem;

  public:
    std::string config_dir;
    std::shared_ptr<FileReader<BasicType, SemanticType>> file_loader_ptr;
    FileReader<BasicType, SemanticType> file_loader;
    SH_TABLE_CONFIG<double> SH_table_config;
    LOCAL_BA_CONFIG local_BA_config;
    DIRECT_METHOD_CONFIG direct_method_config;
    std::shared_ptr<DEBUG_CONFIG> debug_config_ptr;
    RosHandler ros_handler;
    Eigen::VectorXi std_idx;

  public:
    int start_frame = 0;
    int end_frame = 0;
    int obj_num = 0;

    std::vector<Eigen::Isometry3d> odometry;
    std::vector<Eigen::Isometry3d> trajectory;
    std::vector<Eigen::Isometry3d> trajectory_noised;
    // observations
    std::vector<Eigen::Isometry3d> odometry_noised;
    std::vector<std::vector<Eigen::Isometry3d>> T_obj_lidar_vec;
    std::vector<std::vector<DynamicMatrix>> conformal_map_vec;
    std::vector<std::vector<DynamicMatrix>> sph_coeff_vec;
    std::vector<std::vector<DynamicMatrix>> sph_coeff_vec_init;
    std::vector<std::vector<DynamicMatrix>> scans_in_lidar;
    std::vector<std::vector<std::pair<interp_func_pair, interp_func_pair>>> interp_conformal_vec;
    std::vector<std::vector<std::pair<interp_func_pair, interp_func_pair>>> interp_inv_conformal_vec;
    std::vector<std::vector<PatchProcession<double>>> patch_procession;
    std::vector<DynamicMatrix> scans_vec;
    std::vector<DynamicMatrixSem> semantics_vec;
    std::vector<std::vector<DynamicMatrix>> m_x_Gx;
    std::vector<std::vector<DynamicMatrix>> m_x_Gy;
    std::vector<std::vector<DynamicMatrix>> m_y_Gx;
    std::vector<std::vector<DynamicMatrix>> m_y_Gy;
    // states
    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_w_j_vec;
    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_o_w_vec;
    std::vector<Eigen::Matrix3d> R_w_j_init_vec;
    std::vector<Eigen::Vector3d> t_w_j_init_vec;
    std::vector<Eigen::Matrix3d> R_w_j_vec;
    std::vector<Eigen::Vector3d> t_w_j_vec;
    std::vector<Eigen::Matrix3d> R_o_w_vec;
    std::vector<Eigen::Vector3d> t_o_w_vec;
    std::vector<std::pair<Eigen::VectorXd, int>> residual_sum_pair;
    // visualization points and index
    std::vector<std::vector<DynamicMatrix>> points_j_valid_vec;
    std::vector<std::vector<DynamicMatrix>> points_j_transformed_recons_vec;
    std::vector<std::vector<std::vector<int>>> overlap_idx;
    std::vector<DynamicMatrix> points_j_1_valid_vec_tmp;
    std::vector<DynamicMatrix> points_j_1_transformed_recons_vec_tmp;
    std::vector<std::vector<int>> overlap_idx_tmp;
    std::vector<DynamicMatrix> points_obj_recons_reference;
    std::vector<DynamicMatrix> points_obj_recons_updated;
    std::vector<std::vector<DynamicMatrix>> points_obj;
    std::vector<DynamicMatrix> points_obj_tmp;

  public:
    void save_optimized_pose(const std::string &filename, std::vector<Eigen::Matrix3d> &R,
                             std::vector<Eigen::Vector3d> &t);
    void save_optimized_sph_coeff(const std::string &dir, std::vector<DynamicMatrix> sph_coeff);

  public:
    explicit CURL_local_BA(std::shared_ptr<FileReader<BasicType, SemanticType>> _file_loader_ptr,
                           const std::string &filename = "");
    explicit CURL_local_BA(FileReader<BasicType, SemanticType> &_file_loader, const std::string &filename = "");
    void BA();
    void processing();
};

#endif // CURL_SLAM_CURL_LOCAL_BA_H
