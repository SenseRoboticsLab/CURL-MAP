//
// Created by zkc on 19/06/23.
//

#ifndef SRC_PatchProcessing_H
#define SRC_PatchProcessing_H
#include "Timer.h"
#include "curl_tools_light.h"
#include "linear_interpolation_2.h"
#include "linear_interpolation_2_func.h"
#include "load_config.h"
#include "types.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
template <typename BasicType> class PatchProcession {
  public:
    const BasicType T_INVALID = std::numeric_limits<BasicType>::max();
    std::shared_ptr<DIRECT_METHOD_CONFIG> direct_method_config_ptr;
    std::shared_ptr<SH_TABLE_CONFIG<BasicType>> SH_table_config_ptr;
    std::shared_ptr<DEBUG_CONFIG> debug_config_ptr;

    // these two information can be abondoned after generating the image
    std::pair<interp_func_pair, interp_func_pair> conformal_mapping;
    std::pair<interp_func_pair, interp_func_pair> inv_conformal_mapping;

    interp_func_pair I_interp;
    BasicType min_X, max_X, min_Y, max_Y;
    BasicType min_x = 0, max_x, min_y = 0, max_y;
    Eigen::VectorX<BasicType> sph_coeff;
    // images
    int x_size, y_size;
    int X_size, Y_size;
    Eigen::MatrixX<BasicType> recons_v_local;
    std::vector<int> valid_recons_idx;
    Eigen::MatrixX<BasicType> I, I_Gx, I_Gy;
    Eigen::MatrixX<BasicType> Gamma_x, Gamma_x_Gx, Gamma_x_Gy;
    Eigen::MatrixX<BasicType> Gamma_y, Gamma_y_Gx, Gamma_y_Gy;
    // for map generation
    Eigen::MatrixX<BasicType> inv_Gamma_x;
    Eigen::MatrixX<BasicType> inv_Gamma_y;
    // lock for image I relative write and read
    mutable std::mutex I_lock;
    // residual
    Eigen::VectorX<BasicType> sph_residuals;
    Eigen::VectorX<BasicType> tmp_sph_residual;
    int residual_counter;
    Eigen::MatrixX<BasicType> sph_dirs;
    // status for this patch; decide whether icp or direct method is goind to be use
    int status;
    // degree
    int SH_degree;

    PatchProcession(const std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                    const std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                    std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                    const std::pair<interp_func_pair, interp_func_pair> &_conformal_mapping,
                    const Eigen::MatrixX<BasicType> &_v_local, const Eigen::VectorX<BasicType> &_map_x,
                    const Eigen::VectorX<BasicType> &_map_y, const Eigen::VectorX<BasicType> &_sph_coeff,
                    int _SH_degree);
    // constructor for odometry
    PatchProcession(const std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                    const std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                    std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                    const std::pair<interp_func_pair, interp_func_pair> &_conformal_mapping,
                    const std::pair<interp_func_pair, interp_func_pair> &_inv_conformal_mapping,
                    const Eigen::MatrixX<BasicType> &_v_local, const Eigen::VectorX<BasicType> &_map_x,
                    const Eigen::VectorX<BasicType> &_map_y, const Eigen::VectorX<BasicType> &_sph_coeff,
                    int _SH_degree, int _status = curl::GOOD);

    PatchProcession(const std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                    std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                    std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                    const std::pair<interp_func_pair, interp_func_pair> &_conformal_mapping,
                    const std::pair<interp_func_pair, interp_func_pair> &_inv_conformal_mapping,
                    interp_func_pair &_I_interp, const Eigen::MatrixX<BasicType> &_v_local,
                    const Eigen::VectorX<BasicType> &_map_x, const Eigen::VectorX<BasicType> &_map_y,
                    const Eigen::VectorX<BasicType> &_sph_coeff, int _SH_degree);

    // constructor for map generation
    PatchProcession(const std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                    const std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                    std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                    const std::pair<interp_func_pair, interp_func_pair> &_conformal_mapping,
                    const std::pair<interp_func_pair, interp_func_pair> &_inv_conformal_mapping,
                    const Eigen::VectorX<BasicType> &_sph_coeff, int _SH_degree);

    void calculate_I();
    void calculate_Gamma(const std::pair<interp_func_pair, interp_func_pair> &_conformal_mapping);
    void calculate_inv_Gamma(const std::pair<interp_func_pair, interp_func_pair> &_inv_conformal_mapping);
    void calculate_delta();
    void calculate_recons_v_local(const std::pair<interp_func_pair, interp_func_pair> &_inv_conformal_mapping);

    BasicType fetch_I(const Eigen::MatrixX<BasicType> &I_matrix, const BasicType x, const BasicType y) const;
    BasicType fetch_Gamma(const Eigen::MatrixX<BasicType> &Gamma_matrix, const BasicType X, const BasicType Y) const;
    BasicType fetch_inv_Gamma(const Eigen::MatrixX<BasicType> &inv_Gamma_matrix, const BasicType x,
                              const BasicType y) const;

    BasicType fetch_I_interp(const BasicType x, const BasicType y) const;
    BasicType fetch_I_Gx_interp(const BasicType x, const BasicType y) const;
    BasicType fetch_I_Gy_interp(const BasicType x, const BasicType y) const;

    BasicType fetch_Gamma_x_interp(const BasicType X, const BasicType Y) const;
    BasicType fetch_Gamma_y_interp(const BasicType X, const BasicType Y) const;

    BasicType fetch_Gamma_x_Gx_interp(const BasicType X, const BasicType Y) const;
    BasicType fetch_Gamma_x_Gy_interp(const BasicType X, const BasicType Y) const;

    BasicType fetch_Gamma_y_Gx_interp(const BasicType X, const BasicType Y) const;
    BasicType fetch_Gamma_y_Gy_interp(const BasicType X, const BasicType Y) const;

    BasicType fetch_inv_Gamma_x_interp(const BasicType x, const BasicType y) const;
    BasicType fetch_inv_Gamma_y_interp(const BasicType x, const BasicType y) const;

    bool update_sph_residuals(const Eigen::MatrixX<BasicType> &v_obj);
    void update_sph_coeff();
    void update_I();
    void update_recons();
    Eigen::MatrixX<BasicType> reconstruction(int minimum_rso, int degree, BasicType IQR_thres,
                                             const std::string &method = "bilinear");
};

#endif // SRC_PatchProcessing_H