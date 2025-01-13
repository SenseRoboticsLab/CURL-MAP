// Created by zkc on 04/04/23.
//

#ifndef SRC_DIRECTMETHOD_H
#define SRC_DIRECTMETHOD_H
#include "Timer.h"
#include "curl_tools_light.h"
#include "linear_interpolation_2.h"
#include "linear_interpolation_2_func.h"
#include "load_config.h"
#include <algorithm>
#include <cmath>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

class DirectMethod {
  public:
    std::string config_dir;
    DIRECT_METHOD_CONFIG direct_method_config;
    const SH_TABLE_CONFIG<double> *SH_table_config_ptr;
    int display_scale;
    Eigen::Vector2d conformal_map_boundary;

  public:
    Eigen::MatrixXd img_recons;
    Eigen::MatrixXd img_interp_valid; // template (target)
    cv::Mat img_recons_mat;
    cv::Mat img_interp_valid_mat;
    Eigen::Matrix<double, 3, Eigen::Dynamic> img_interp_valid_indices;
    Eigen::Matrix<double, 3, Eigen::Dynamic> img_interp_indices;
    std::vector<int> interp_valid_idx;
    Eigen::MatrixXd img_interp;                  // I
    Eigen::Vector2d pixel_interp_displace;       // [0]: x, [1]: y (from img_interp to img_recons)
    Eigen::Vector2d pixel_interp_valid_displace; // [0]: x, [1]: y //
                                                 // (from img_interp_valid to img_recons)
    Eigen::VectorXd warp_vec; // inverse with pixel_interp_valid_displace [0]: theta, [1]: x, [2]: y
                              // (from img_recons to img_interp_valid)
    double x_pixel_interval;
    double y_pixel_interval;
    Eigen::Matrix3d initial_warp_matrix;
    Eigen::Matrix2d boundary_of_recons;
    Eigen::Matrix2d boundary_of_interp;

    Eigen::MatrixXd transformed_conformal_coord;
    Eigen::MatrixXd transformed_conformal_sph_dirs;
    std::vector<int> transformed_conformal_valid_idx;

  public:
    DirectMethod(const DIRECT_METHOD_CONFIG &_direct_method_config,
                 const SH_TABLE_CONFIG<double> *_SH_table_config_ptr,
                 const Eigen::Vector2d &_conformal_map_boundary);
    ~DirectMethod() = default;
    void display_img(const std::string &img_name, const cv::Mat img, bool is_block = false,
                     int x_scale = 10, int y_scale = 10);
    void display_img_bb(const std::string &img_name, const cv::Mat img,
                        const Eigen::MatrixXd &indices, bool is_block = false, int x_scale = 10,
                        int y_scale = 10);
    bool height_img_generation(const Eigen::VectorXd &sph_coeff,
                               const std::pair<std::vector<double>, std::vector<int>> &x0,
                               const std::pair<std::vector<double>, std::vector<int>> &x1,
                               const Eigen::VectorXd &height_value);
    void compute_jacobian(const cv::Mat &img, Eigen::MatrixXd &jacobian);

    Eigen::MatrixXd get_warp_matrix();

    void
    get_transformed_conformal_coord(const std::pair<std::vector<double>, std::vector<int>> &x0,
                                    const std::pair<std::vector<double>, std::vector<int>> &x1);

    bool track(const Eigen::VectorXd &sph_coeff, const cv::Mat &img_template,
               const Eigen::Matrix<double, 3, Eigen::Dynamic> &template_indices, bool is_pyramid,
               double threshold = 1e-6);
    bool pyramid_track(const Eigen::VectorXd &sph_coeff, double SH_scale, double threshold = 1e-6);

    void update_points(const Eigen::VectorXd &sph_coeff,
                       const std::pair<std::vector<double>, std::vector<int>> &x0,
                       const std::pair<std::vector<double>, std::vector<int>> &x1,
                       const std::pair<interp_func_pair, interp_func_pair> &interp_2d_plane,
                       const Eigen::MatrixXd &points_transformed,
                       std::pair<std::vector<double>, std::vector<int>> &x_2d,
                       std::pair<std::vector<double>, std::vector<int>> &y_2d,
                       Eigen::Matrix<double, 3, Eigen::Dynamic> &points_transformed_recons);

    Eigen::VectorXd update_sph_coeff(const Eigen::VectorXd &sph_coeff,
                                     const std::pair<std::vector<double>, std::vector<int>> &x0,
                                     const std::pair<std::vector<double>, std::vector<int>> &x1,
                                     const Eigen::VectorXd &height_value);

    void update_sph_coeff_residual(const Eigen::VectorXd &sph_coeff,
                                   const std::pair<std::vector<double>, std::vector<int>> &x0,
                                   const std::pair<std::vector<double>, std::vector<int>> &x1,
                                   const Eigen::VectorXd &height_value,
                                   std::pair<Eigen::VectorXd, int> &residual_sum_pair);
};

#endif // SRC_DIRECTMETHOD_H