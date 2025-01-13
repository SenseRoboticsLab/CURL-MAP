//
// Created by zkc on 04/04/23.
//

#include "DirectMethod.h"

DirectMethod::DirectMethod(const DIRECT_METHOD_CONFIG &_direct_method_config,
                           const SH_TABLE_CONFIG<double> *_SH_table_config_ptr,
                           const Eigen::Vector2d &_conformal_map_boundary)
    : direct_method_config(_direct_method_config), SH_table_config_ptr(_SH_table_config_ptr),
      conformal_map_boundary(_conformal_map_boundary) {
    display_scale = ceil(20.0 / direct_method_config.minimum_img_rso * 10.0);
}

void DirectMethod::display_img(const std::string &img_name, const cv::Mat img, bool is_block, int x_scale,
                               int y_scale) {
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
    cv::Mat img_resize;
    cv::resize(img, img_resize, cv::Size(), x_scale, y_scale, cv::INTER_LINEAR);
    img_resize /= maxVal;
    cv::imshow(img_name, img_resize);
    if (is_block) {
        cv::waitKey(0);
    } else {
        cv::waitKey(1);
    }
}

void DirectMethod::display_img_bb(const std::string &img_name, const cv::Mat img, const Eigen::MatrixXd &indices,
                                  bool is_block, int x_scale, int y_scale) {
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;

    cv::minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);

    cv::Mat img_resize;
    cv::resize(img, img_resize, cv::Size(), x_scale, y_scale, cv::INTER_LINEAR);
    img_resize /= maxVal;
    for (int i = 0; i < indices.rows(); ++i) {
        int x = round(indices(i, 0) * x_scale);
        int y = round(indices(i, 1) * y_scale);
        if ((x >= 0) && (y >= 0) && (x < img_resize.cols) && (y < img_resize.rows)) {
            img_resize.at<double>(y, x) = 1;
        }
    }
    cv::imshow(img_name, img_resize);
    if (is_block) {
        cv::waitKey(0);
    } else {
        cv::waitKey(1);
    }
}

bool DirectMethod::height_img_generation(const Eigen::VectorXd &sph_coeff,
                                         const std::pair<std::vector<double>, std::vector<int>> &x0,
                                         const std::pair<std::vector<double>, std::vector<int>> &x1,
                                         const Eigen::VectorXd &height_value) {
    // 1 step: generate a query grid on conformal map
    int x_rso, y_rso;
    boundary_of_recons << Eigen::Vector2d::Zero(), conformal_map_boundary;
    if (conformal_map_boundary(0) <= conformal_map_boundary(1)) {
        x_rso = direct_method_config.minimum_img_rso;
        y_rso = int(round(conformal_map_boundary(1) / conformal_map_boundary(0) * x_rso));
    } else {
        y_rso = direct_method_config.minimum_img_rso;
        x_rso = int(round(conformal_map_boundary(0) / conformal_map_boundary(1) * y_rso));
    }

    Eigen::ArrayXd phis_recons = Eigen::ArrayXd::LinSpaced(x_rso, M_PI * (1 - SH_table_config_ptr->SH_scale),
                                                           2 * M_PI * SH_table_config_ptr->SH_scale +
                                                               M_PI * (1 - SH_table_config_ptr->SH_scale));
    Eigen::ArrayXd thetas_recons = Eigen::ArrayXd::LinSpaced(y_rso, M_PI / 2 * (1 - SH_table_config_ptr->SH_scale),
                                                             M_PI * SH_table_config_ptr->SH_scale +
                                                                 M_PI / 2 * (1 - SH_table_config_ptr->SH_scale));
    Eigen::MatrixXd sph_dirs(phis_recons.size() * thetas_recons.size(), 2);
    // 2 step: generate a mesh grid for azi and elev (ToDo: this place can be improved by only
    // reconstructing the interesting region)
    for (int i = 0; i < phis_recons.size(); ++i) {
        sph_dirs(Eigen::seq(i * thetas_recons.size(), (i + 1) * thetas_recons.size() - 1), 0) =
            Eigen::VectorXd::Ones(thetas_recons.size()).array() * phis_recons(i);
        sph_dirs(Eigen::seq(i * thetas_recons.size(), (i + 1) * thetas_recons.size() - 1), 1) = thetas_recons;
    }
    // 3 step: use sph_coeff to recons the height value
    Eigen::VectorXd height_recons;
    if (SH_table_config_ptr->is_speed_up) {
        height_recons = curl::invLeastSquaresSHT_table(sph_coeff, sph_dirs, *SH_table_config_ptr,
                                                       SH_table_config_ptr->warping_SH_degree);
    } else {
        height_recons = curl::invLeastSquaresSHT(sph_coeff, sph_dirs, SH_table_config_ptr->warping_SH_degree);
    }

    img_recons = Eigen::Map<Eigen::MatrixXd>(height_recons.data(), thetas_recons.size(), phis_recons.size());
    // 4 step: get the conformal map of the current object scan
    Eigen::VectorXd x0_tmp = Eigen::Map<const Eigen::VectorXd>(x0.first.data(), static_cast<long>(x0.first.size()));
    Eigen::VectorXd x1_tmp = Eigen::Map<const Eigen::VectorXd>(x1.first.data(), static_cast<long>(x1.first.size()));
    Eigen::MatrixXd x0_x1_valid(x1.second.size(), 2);
    x0_x1_valid << x0_tmp(x1.second), x1_tmp(x1.second); // this should be fine since the vector//
                                                         // length is equal to the matrix col size
    boundary_of_interp << x0_x1_valid.col(0).minCoeff(), x0_x1_valid.col(0).maxCoeff(), x0_x1_valid.col(1).minCoeff(),
        x0_x1_valid.col(1).maxCoeff();
    Eigen::VectorXd height_valid = height_value(x1.second);
    // 5 step: generate the interpolation function
    interp_func_pair F = linear_interpolation_2_func(x0_x1_valid, height_valid);
    // 6 step: generate the mesh grid and make sure the object scan height image has the same
    // resolution with the previous one
    Eigen::Vector2d object_conformal_boundary;
    x_pixel_interval = conformal_map_boundary(0) / (x_rso - 1);
    y_pixel_interval = conformal_map_boundary(1) / (y_rso - 1);
    Eigen::VectorXd x_conformal_cor =
        curl::range<double>(x0_x1_valid.col(0).minCoeff(), x0_x1_valid.col(0).maxCoeff(), x_pixel_interval);
    Eigen::VectorXd y_conformal_cor =
        curl::range<double>(x0_x1_valid.col(1).minCoeff(), x0_x1_valid.col(1).maxCoeff(), y_pixel_interval);
    Eigen::VectorXd x_idx = Eigen::VectorXd::LinSpaced(x_conformal_cor.size(), 0, x_conformal_cor.size() - 1);
    Eigen::VectorXd y_idx = Eigen::VectorXd::LinSpaced(y_conformal_cor.size(), 0, y_conformal_cor.size() - 1);
    img_interp_indices =
        Eigen::Matrix<double, 3, Eigen::Dynamic>::Ones(3, x_conformal_cor.size() * y_conformal_cor.size());
    Eigen::MatrixXd interp_dir(x_conformal_cor.size() * y_conformal_cor.size(), 2);
    for (int i = 0; i < x_conformal_cor.size(); ++i) {
        interp_dir(Eigen::seq(i * y_conformal_cor.size(), (i + 1) * y_conformal_cor.size() - 1), 0) =
            Eigen::VectorXd::Ones(y_conformal_cor.size()).array() * x_conformal_cor(i);
        interp_dir(Eigen::seq(i * y_conformal_cor.size(), (i + 1) * y_conformal_cor.size() - 1), 1) = y_conformal_cor;
        img_interp_indices(0, Eigen::seq(i * y_conformal_cor.size(), (i + 1) * y_conformal_cor.size() - 1)) =
            Eigen::VectorXd::Ones(y_conformal_cor.size()).array() * x_idx(i);
        img_interp_indices(1, Eigen::seq(i * y_conformal_cor.size(), (i + 1) * y_conformal_cor.size() - 1)) = y_idx;
    }
    // get the image displacement between img_interp and img_recons
    pixel_interp_displace << x0_x1_valid.col(0).minCoeff() / x_pixel_interval,
        x0_x1_valid.col(1).minCoeff() / y_pixel_interval;
    // 7 step: do the interpolation
    std::pair<std::vector<double>, std::vector<int>> height_interp_pair = linear_interpolation_2(interp_dir, F);
    // 8 step: give the interpolation result to the height image
    Eigen::VectorXd height_interp = Eigen::Map<Eigen::VectorXd>(height_interp_pair.first.data(),
                                                                static_cast<long>(height_interp_pair.first.size()));
    img_interp = Eigen::Map<Eigen::MatrixXd>(height_interp.data(), y_conformal_cor.size(), x_conformal_cor.size());
    // 9 step: if hole exists, cut key rectangular region as template
    bool is_valid = true;
    if (height_interp_pair.second.size() != height_interp_pair.first.size()) {
        is_valid = false;
        if (direct_method_config.valid_region_method == "cut") {
            // fetch center region without holes
            //            std::cout << "height_img_generation: exists holes in interpolated height
            //            image"
            //                      << std::endl;
            int y_center = ceil(static_cast<double>(img_interp.rows()) / 2.0);
            int x_center = ceil(static_cast<double>(img_interp.cols()) / 2.0);
            int incr = 0;
            double scale_factor = static_cast<double>(img_interp.rows()) / static_cast<double>(img_interp.cols());

            while ((!is_valid) && (img_interp(y_center, x_center) != DOUBLE_INVALID) &&
                   ((incr <= y_center) && (incr <= x_center))) {
                ++incr;
                if (scale_factor >= 1) {
                    // clock-wise shrink from four corner of the image
                    is_valid =
                        (img_interp(int(round(incr * scale_factor)), incr) != DOUBLE_INVALID) &
                        (img_interp(int(round(incr * scale_factor)), img_interp.cols() - 1 - incr) != DOUBLE_INVALID) &
                        (img_interp(img_interp.rows() - 1 - int(round(incr * scale_factor)),
                                    img_interp.cols() - 1 - incr) != DOUBLE_INVALID) &
                        (img_interp(img_interp.rows() - 1 - int(round(incr * scale_factor)), incr) != DOUBLE_INVALID);
                    if (is_valid) {
                        is_valid = (img_interp(Eigen::seq(int(round(incr * scale_factor)),
                                                          img_interp.rows() - 1 - int(round(incr * scale_factor))),
                                               Eigen::seq(incr, img_interp.cols() - 1 - incr))
                                        .maxCoeff() != DOUBLE_INVALID);
                        if (is_valid) {
                            img_interp_valid =
                                img_interp(Eigen::seq(int(round(incr * scale_factor)),
                                                      img_interp.rows() - 1 - int(round(incr * scale_factor))),
                                           Eigen::seq(incr, img_interp.cols() - 1 - incr));
                            // 10 step: get the pixel displacement of two images
                            pixel_interp_valid_displace[0] = x0_x1_valid.col(0).minCoeff() / x_pixel_interval + incr;
                            pixel_interp_valid_displace[1] =
                                x0_x1_valid.col(1).minCoeff() / y_pixel_interval + int(round(incr * scale_factor));
                        }
                    }
                } else {
                    // clock-wise shrink from four corner of the image
                    is_valid =
                        (img_interp(incr, int(round(incr * (1 / scale_factor)))) != DOUBLE_INVALID) &
                        (img_interp(incr, img_interp.cols() - int(round(incr * (1 / scale_factor)))) !=
                         DOUBLE_INVALID) &
                        (img_interp(img_interp.rows() - 1 - incr,
                                    img_interp.cols() - 1 - int(round(incr * (1 / scale_factor)))) != DOUBLE_INVALID) &
                        (img_interp(img_interp.rows() - 1 - incr, int(round(incr * (1 / scale_factor)))) !=
                         DOUBLE_INVALID);
                    if (is_valid) {
                        is_valid =
                            (img_interp(Eigen::seq(incr, img_interp.rows() - 1 - incr),
                                        Eigen::seq(int(round(incr * (1 / scale_factor))),
                                                   img_interp.cols() - 1 - int(round(incr * (1 / scale_factor)))))
                                 .maxCoeff() != DOUBLE_INVALID);
                        if (is_valid) {
                            img_interp_valid =
                                img_interp(Eigen::seq(incr, img_interp.rows() - 1 - incr),
                                           Eigen::seq(int(round(incr * (1 / scale_factor))),
                                                      img_interp.cols() - 1 - int(round(incr * (1 / scale_factor)))));
                            // 10 step: get the pixel displacement of two images
                            pixel_interp_valid_displace[0] = x0_x1_valid.col(0).minCoeff() / x_pixel_interval +
                                                             int(round(incr * (1 / scale_factor)));
                            pixel_interp_valid_displace[1] = x0_x1_valid.col(1).minCoeff() / y_pixel_interval + incr;
                        }
                    }
                }
            }

            // 10 step: get the indices of img_interp_valid
            if (is_valid) {
                img_interp_valid_indices = Eigen::Matrix<double, 3, Eigen::Dynamic>::Ones(3, img_interp_valid.size());
                for (int x = 0; x < img_interp_valid.cols(); ++x) {
                    img_interp_valid_indices(
                        0, Eigen::seq(x * img_interp_valid.rows(), (x + 1) * img_interp_valid.rows() - 1)) =
                        Eigen::VectorXd::Ones(img_interp_valid.rows()) * x;
                    img_interp_valid_indices(
                        1, Eigen::seq(x * img_interp_valid.rows(), (x + 1) * img_interp_valid.rows() - 1)) =
                        Eigen::VectorXd::LinSpaced(img_interp_valid.rows(), 0, img_interp_valid.rows() - 1);
                }
            }

        } else if (direct_method_config.valid_region_method == "interp") {
            Eigen::MatrixXd valid_indices = img_interp_indices(Eigen::seq(0, 1), height_interp_pair.second).transpose();
            pcl::PointCloud<pcl::PointXY>::Ptr coordinate_cloud(new pcl::PointCloud<pcl::PointXY>);
            curl::convert_to_point_cloud_2d(valid_indices, coordinate_cloud);
            int result_size = 1;
            pcl::KdTreeFLANN<pcl::PointXY> kdtree_pcl;
            kdtree_pcl.setInputCloud(coordinate_cloud);
            std::vector<int> pointIdxNKNSearch(result_size);
            std::vector<float> pointNKNSquaredDistance(result_size);
            pcl::PointXY search_point;
            for (int x = 0; x < img_interp.cols(); ++x) {
                for (int y = 0; y < img_interp.rows(); ++y) {
                    if (img_interp(y, x) > 100) {
                        search_point.x = x;
                        search_point.y = y;
                        kdtree_pcl.nearestKSearch(search_point, result_size, pointIdxNKNSearch,
                                                  pointNKNSquaredDistance);
                        img_interp(y, x) = img_interp(int(valid_indices(pointIdxNKNSearch[0], 1)),
                                                      int(valid_indices(pointIdxNKNSearch[0], 0)));
                    }
                }
            }
            img_interp_valid = img_interp;
            img_interp_valid_indices = img_interp_indices;
            pixel_interp_valid_displace = pixel_interp_displace;
            if (img_interp_valid.size() > 0) {
                is_valid = true;
            }
        }
    }
    if (is_valid) {
        cv::eigen2cv(img_interp_valid, img_interp_valid_mat);
        if (direct_method_config.is_gaussian_blur) {
            cv::GaussianBlur(img_interp_valid_mat, img_interp_valid_mat, cv::Size(5, 5), direct_method_config.sigma);
            //            cv::eigen2cv(img_recons, img_recons_mat);
            //            cv::GaussianBlur(img_recons_mat, img_recons_mat, cv::Size(5, 5),
            //                             direct_method_config.sigma);
            //            cv::cv2eigen(img_recons_mat, img_recons);
        }
    }
    return is_valid;
}

// this is calculating the steepest decent image
void DirectMethod::compute_jacobian(const cv::Mat &img, Eigen::MatrixXd &jacobian) {
    if (direct_method_config.method == "theta_x_y") {
        jacobian.resize(img.rows * img.cols, 3);
    } else if (direct_method_config.method == "x_y") {
        jacobian.resize(img.rows * img.cols, 2);
    } else if (direct_method_config.method == "affine") {
        jacobian.resize(img.rows * img.cols, 6);
    }
    cv::Mat grad_x, grad_y;
    cv::Sobel(img, grad_x, CV_64F, 1, 0);
    cv::Sobel(img, grad_y, CV_64F, 0, 1);
    if (direct_method_config.is_display) {
        display_img("grad_x", grad_x, direct_method_config.is_display_block, display_scale, display_scale);
        display_img("grad_y", grad_y, direct_method_config.is_display_block, display_scale, display_scale);
    }
    Eigen::MatrixXd dWdp;
    Eigen::RowVector2d delI(2);
    for (int x = 0; x < img.cols; ++x) {
        for (int y = 0; y < img.rows; ++y) {
            if (direct_method_config.method == "theta_x_y") {
                dWdp.resize(2, 3);
                dWdp << -sin(0) * x - cos(0) * y, 1, 0, cos(0) * x - sin(0) * y, 0, 1;
            } else if (direct_method_config.method == "x_y") {
                dWdp.resize(2, 2);
                dWdp << 1, 0, 0, 1;
            } else if (direct_method_config.method == "affine") {
                dWdp.resize(2, 6);
                dWdp << x, 0, y, 0, 1, 0, 0, x, 0, y, 0, 1;
            }
            double delIx = grad_x.at<double>(y, x);
            double delIy = grad_y.at<double>(y, x);
            delI << delIx, delIy;
            jacobian.row(x * img.rows + y) = delI * dWdp;
        }
    }
}

Eigen::MatrixXd DirectMethod::get_warp_matrix() {
    Eigen::Matrix3d warp_matrix;
    if (direct_method_config.method == "theta_x_y") {
        warp_matrix << cos(warp_vec(0)), -sin(warp_vec(0)), warp_vec(1), sin(warp_vec(0)), cos(warp_vec(0)),
            warp_vec(2), 0, 0, 1;

    } else if (direct_method_config.method == "x_y") {
        warp_matrix << 1, 0, warp_vec(0), 0, 1, warp_vec(1), 0, 0, 1;
    } else if (direct_method_config.method == "affine") {
        warp_matrix << 1 + warp_vec(0), warp_vec(2), warp_vec(4), warp_vec(1), 1 + warp_vec(3), warp_vec(5), 0, 0, 1;
    }
    return warp_matrix;
}

void DirectMethod::get_transformed_conformal_coord(const std::pair<std::vector<double>, std::vector<int>> &x0,
                                                   const std::pair<std::vector<double>, std::vector<int>> &x1) {
    Eigen::VectorXd x_conformal = Eigen::Map<const Eigen::VectorXd>(x0.first.data(), x0.first.size());
    Eigen::VectorXd y_conformal = Eigen::Map<const Eigen::VectorXd>(x1.first.data(), x1.first.size());
    Eigen::MatrixXd conformal_coord = Eigen::MatrixXd::Ones(x0.first.size(), 3);
    conformal_coord.col(0) = x_conformal;
    conformal_coord.col(1) = y_conformal;
    Eigen::Matrix3d conformal_T_recons_interp = get_warp_matrix().inverse();
    // The below transformation is correct because we are interested in how the transformation
    // changed after given the initial value; also can be understand as first we need to
    // transfer from recons to valid coordinate
    conformal_T_recons_interp(0, 2) =
        (conformal_T_recons_interp(0, 2) - pixel_interp_valid_displace(0)) * x_pixel_interval;
    conformal_T_recons_interp(1, 2) =
        (conformal_T_recons_interp(1, 2) - pixel_interp_valid_displace(1)) * y_pixel_interval;
    transformed_conformal_coord = (conformal_T_recons_interp * conformal_coord.transpose()).transpose();
    // valid indices
    transformed_conformal_sph_dirs.resize(transformed_conformal_coord.rows(), 2);
    transformed_conformal_sph_dirs.col(0) =
        (transformed_conformal_coord.col(0) / conformal_map_boundary(0) * 2 * M_PI * SH_table_config_ptr->SH_scale)
            .array() +
        M_PI * (1 - SH_table_config_ptr->SH_scale);
    transformed_conformal_sph_dirs.col(1) =
        (transformed_conformal_coord.col(1) / conformal_map_boundary(1) * M_PI * SH_table_config_ptr->SH_scale)
            .array() +
        M_PI_2 * (1 - SH_table_config_ptr->SH_scale);
    double azi_low = SH_table_config_ptr->get_azi_low();
    double azi_high = SH_table_config_ptr->get_azi_high();
    double elev_low = SH_table_config_ptr->get_elev_low();
    double elev_high = SH_table_config_ptr->get_elev_high();
    int counter = 0;
    for (int i = 0; i < transformed_conformal_sph_dirs.rows(); ++i) {
        if (x1.second[counter] == i) {
            if (((transformed_conformal_sph_dirs(i, 0) >= azi_low) &&
                 (transformed_conformal_sph_dirs(i, 0) <= azi_high)) &&
                ((transformed_conformal_sph_dirs(i, 1) >= elev_low) &&
                 (transformed_conformal_sph_dirs(i, 1) <= elev_high))) {
                transformed_conformal_valid_idx.push_back(i);
            }
            ++counter;
        }
    }
}

bool DirectMethod::track(const Eigen::VectorXd &sph_coeff, const cv::Mat &img_template,
                         const Eigen::Matrix<double, 3, Eigen::Dynamic> &template_indices, bool is_pyramid,
                         double threshold) {
    // Pre-compute 3: evaluate the gradient of the template image
    Eigen::MatrixXd img_template_eigen;
    cv::cv2eigen(img_template, img_template_eigen);
    // calculate the jacobian
    Eigen::MatrixXd jacobian;
    if (img_template_eigen.size() > 0) {
        compute_jacobian(img_template, jacobian);
    } else {
        std::cout << "no img_template_eigen" << std::endl;
        return false;
    }
    // calculate the hessian
    const Eigen::MatrixXd jacobian_transposed = jacobian.transpose();
    const Eigen::MatrixXd Hessian = jacobian_transposed * jacobian;
    if (direct_method_config.method == "theta_x_y") {
        warp_vec.resize(3, 1);
        warp_vec << 0, -pixel_interp_valid_displace(0),
            -pixel_interp_valid_displace(1); // inverse with pixel_interp_valid_displace [0]: theta,
                                             // [1]: x, [2]: y (from img_recons to img_template_eigen)
    } else if (direct_method_config.method == "x_y") {
        warp_vec.resize(2, 1);
        warp_vec << -pixel_interp_valid_displace(0),
            -pixel_interp_valid_displace(1); // inverse with pixel_interp_valid_displace [0]: x,
                                             // [1]: y (from img_recons to img_template_eigen)
    } else if (direct_method_config.method == "affine") {
        warp_vec.resize(6, 1);
        warp_vec << 0, 0, 0, 0, -pixel_interp_valid_displace(0),
            -pixel_interp_valid_displace(1); // inverse with pixel_interp_valid_displace [4]: x,
                                             // [4]: y (from img_recons to img_template_eigen)
    }
    Eigen::VectorXd warp_vec_last = warp_vec;
    Eigen::Map<Eigen::VectorXd> img_template_eigen_vec(img_template_eigen.data(), img_template_eigen.size());
    double error_last = DOUBLE_INVALID;
    int break_count = 0;
    if (is_pyramid) {
        break_count = 3;
    }
    for (int i = 0; i < direct_method_config.max_iters; ++i) {
        if (i == 0) {
            initial_warp_matrix = get_warp_matrix();
        }
        Eigen::Matrix3d warp_matrix = get_warp_matrix();
        // step 1, warp the interp image on the recons image (get the corresponding coordinate of
        // the recons image)
        Eigen::Matrix<double, 3, Eigen::Dynamic> template_warped_indices = warp_matrix.inverse() * template_indices;
        // check whether the indices are out of boundary
        interp_valid_idx.clear();
        bool is_out_of_bound = false;
        if ((template_warped_indices.minCoeff() < 0) ||
            (template_warped_indices.row(0).maxCoeff() > (img_recons.cols() - 1)) ||
            (template_warped_indices.row(1).maxCoeff() > (img_recons.rows() - 1))) {
            is_out_of_bound = true;
            for (int j = 0; j < template_warped_indices.cols(); ++j) {
                if ((template_warped_indices(0, j) >= 0) && (template_warped_indices(1, j) >= 0) &&
                    (template_warped_indices(0, j) <= (img_recons.cols() - 1)) &&
                    (template_warped_indices(1, j) <= (img_recons.rows() - 1))) {
                    interp_valid_idx.push_back(j);
                }
            }
            if (interp_valid_idx.size() == 0) {
                return false;
            }
        }
        // step 2, use the coordinate got from step one to sampling corresponding depth value
        // from recons image
        // ToDo: Use subpixel interpolation to test the result
        Eigen::VectorXd template_warped_height_recons = Eigen::VectorXd::Zero(template_warped_indices.cols());
        if (is_pyramid) {
            if (!is_out_of_bound) {
                Eigen::Map<const Eigen::VectorXd> img_recons_col(img_recons.data(), img_recons.size());
                Eigen::VectorXi tmp_indices =
                    (template_warped_indices.row(0) * img_recons.rows() + template_warped_indices.row(1))
                        .array()
                        .round()
                        .cast<int>();
                template_warped_height_recons = img_recons_col(tmp_indices);
            } else {
                Eigen::Map<const Eigen::VectorXd> img_recons_col(img_recons.data(), img_recons.size());
                Eigen::VectorXi tmp_indices =
                    (template_warped_indices.row(0) * img_recons.rows() + template_warped_indices.row(1))
                        .array()
                        .round()
                        .cast<int>();
                template_warped_height_recons(interp_valid_idx) = img_recons_col(tmp_indices(interp_valid_idx));
            }
        } else {
            Eigen::VectorXd phis =
                (template_warped_indices.row(0) / (img_recons.cols() - 1) * 2 * M_PI * SH_table_config_ptr->SH_scale)
                    .array() +
                M_PI * (1 - SH_table_config_ptr->SH_scale);
            Eigen::VectorXd thetas =
                (template_warped_indices.row(1) / (img_recons.rows() - 1) * M_PI * SH_table_config_ptr->SH_scale)
                    .array() +
                M_PI_2 * (1 - SH_table_config_ptr->SH_scale);
            Eigen::MatrixXd sph_dirs;
            if (!is_out_of_bound) {
                sph_dirs.resize(phis.size(), 2);
                sph_dirs << phis, thetas;
                if (SH_table_config_ptr->is_speed_up) {
                    template_warped_height_recons = curl::invLeastSquaresSHT_table(
                        sph_coeff, sph_dirs, *SH_table_config_ptr, SH_table_config_ptr->warping_SH_degree);
                } else {
                    template_warped_height_recons =
                        curl::invLeastSquaresSHT(sph_coeff, sph_dirs, SH_table_config_ptr->warping_SH_degree);
                }

            } else {
                sph_dirs.resize(interp_valid_idx.size(), 2);
                sph_dirs << phis(interp_valid_idx), thetas(interp_valid_idx);
                if (SH_table_config_ptr->is_speed_up) {
                    template_warped_height_recons(interp_valid_idx) = curl::invLeastSquaresSHT_table(
                        sph_coeff, sph_dirs, *SH_table_config_ptr, SH_table_config_ptr->warping_SH_degree);
                } else {
                    template_warped_height_recons(interp_valid_idx) =
                        curl::invLeastSquaresSHT(sph_coeff, sph_dirs, SH_table_config_ptr->warping_SH_degree);
                }
            }
        }

        // step 3, calculate the error
        Eigen::VectorXd error_vec = template_warped_height_recons - img_template_eigen_vec; // I(x) - T(W(x,p))
        Eigen::VectorXd error_vec_update = error_vec;
        //        FileReaderBase::write_txt_file(
        //            "/home/user/project/ros/slam/src/CURL_SLAM_ros/tmp_data/error_vec.txt",
        //            Eigen::MatrixXd(error_vec));
        std::vector<double> error_vec_std(error_vec_update.data(), error_vec_update.data() + error_vec_update.size());
        std::transform(error_vec_std.begin(), error_vec_std.end(), error_vec_std.begin(),
                       [](double &ele) { return abs(ele); });
        double delta = curl::IQR_thres<double>(error_vec_std, direct_method_config.IQR_delta);
        if (direct_method_config.loss_function == "linear") {
            curl::linear_loss_function(error_vec_update, delta, direct_method_config.linear_loss_param);
        } else if (direct_method_config.loss_function == "trapeziform") {
            curl::trapeziform(error_vec_update, direct_method_config.trapeziform_lower_bound,
                              direct_method_config.trapeziform_higher_bound);
        } else if (direct_method_config.loss_function == "huber_loss") {
            error_vec_update = error_vec_update.unaryExpr([&](const double &ele) {
                if (abs(ele) > delta) {
                    return (ele / abs(ele) * (2.0 * delta * sqrt(abs(ele)) - delta * delta));
                }
                return ele;
            });
        }
        // step 4, update the warp_vec
        Eigen::VectorXd delta_p = Hessian.inverse() * (jacobian_transposed * error_vec_update);
        if (direct_method_config.method == "affine") {
            Eigen::Matrix3d warp_matrix_delta;
            warp_matrix_delta << 1 + delta_p(0), delta_p(2), delta_p(4), //
                delta_p(1), 1 + delta_p(3), delta_p(5),                  //
                0, 0, 1;
            Eigen::Matrix3d warp_matrix_updated = get_warp_matrix() * warp_matrix_delta;
            warp_vec << warp_matrix_updated(0, 0) - 1, warp_matrix_updated(1, 0), warp_matrix_updated(0, 1),
                warp_matrix_updated(1, 1) - 1, warp_matrix_updated(0, 2), warp_matrix_updated(1, 2);
        } else {
            warp_vec = warp_vec + delta_p;
        }
        if (direct_method_config.is_display) {
            cv::eigen2cv(img_recons, img_recons_mat);
            display_img("template", img_template, direct_method_config.is_display_block, display_scale, display_scale);
            display_img_bb("bounding box", img_recons_mat,
                           Eigen::MatrixXd(template_warped_indices(Eigen::seq(0, 1), Eigen::all).transpose()),
                           direct_method_config.is_display_block, display_scale, display_scale);
        }
        double relative_error_change = abs(error_vec.norm() - error_last) / error_last;
        if (relative_error_change < threshold) {
            if (error_vec.cwiseAbs().mean() < direct_method_config.update_sph_coeff_threshold) {
                return true;
            } else {
                return false;
            }
        } else if (error_vec.norm() > error_last) { // ToDo: if using pyramid, when the error increasing, use higher
                                                    // resolution, if not using pyramid, then use spherical harmonics
                                                    // to update
            ++break_count;
            if (break_count > 2) {
                warp_vec = warp_vec_last;
                return false;
            }
        } else if (break_count > 0) {
            --break_count;
        } else if (warp_vec.hasNaN()) {
            warp_vec = warp_vec_last;
            return false;
        }
        if (i == (direct_method_config.max_iters - 1)) {
            return false;
        }
        if (error_vec.norm() <= error_last) {
            warp_vec_last = warp_vec;
        }
        error_last = error_vec.norm();
    }
    return true;
}

bool DirectMethod::pyramid_track(const Eigen::VectorXd &sph_coeff, double SH_scale, double threshold) {
    int lower_rso =
        img_interp_valid.rows() < img_interp_valid.cols() ? img_interp_valid.rows() : img_interp_valid.cols();
    // generate pyramid
    if (static_cast<double>(lower_rso) / 2.0 >= direct_method_config.pyramid_minimum_img_rso) {
        int pyramid_level = 2;
        while ((static_cast<double>(lower_rso) / pow(2.0, pyramid_level)) >=
               direct_method_config.pyramid_minimum_img_rso) {
            ++pyramid_level;
        }
        std::vector<cv::Mat> img_interp_valid_vec(pyramid_level);
        img_interp_valid_vec[pyramid_level - 1] = img_interp_valid_mat;
        std::vector<Eigen::MatrixXd> img_interp_valid_pyramid_eigen(pyramid_level);
        for (int i = 0; i < (pyramid_level - 1); ++i) {
            cv::pyrDown(img_interp_valid_vec[pyramid_level - 1 - i], img_interp_valid_vec[pyramid_level - 1 - i - 1]);
        }
        // give correct indices
        std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> img_interp_valid_indices_vec(pyramid_level);
        img_interp_valid_indices_vec[pyramid_level - 1] = img_interp_valid_indices;
        for (int i = 0; i < (pyramid_level - 1); ++i) {
            Eigen::VectorXd x_idx =
                curl::range<double>(0, img_interp_valid_vec[pyramid_level - 1].cols - 1, 2 * (i + 1));
            Eigen::VectorXd y_idx =
                curl::range<double>(0, img_interp_valid_vec[pyramid_level - 1].rows - 1, 2 * (i + 1));
            img_interp_valid_indices_vec[pyramid_level - 1 - i - 1] =
                Eigen::Matrix<double, 3, Eigen::Dynamic>::Ones(3, x_idx.size() * y_idx.size());
            for (int j = 0; j < x_idx.size(); ++j) {
                img_interp_valid_indices_vec[pyramid_level - 1 - i - 1](
                    0, Eigen::seq(j * y_idx.size(), (j + 1) * y_idx.size() - 1)) =
                    Eigen::VectorXd::Ones(y_idx.size()) * x_idx(j);
                img_interp_valid_indices_vec[pyramid_level - 1 - i - 1](
                    1, Eigen::seq(j * y_idx.size(), (j + 1) * y_idx.size() - 1)) = y_idx;
            }
        }
        bool result;
        bool is_pyramid = true;
        for (int i = 0; i < pyramid_level; ++i) {
            if (i == pyramid_level - 1) {
                is_pyramid = false;
            }
            result = track(sph_coeff, img_interp_valid_vec[i], img_interp_valid_indices_vec[i], is_pyramid);
        }
        return result;
    } else {
        return track(sph_coeff, img_interp_valid_mat, img_interp_valid_indices, false);
    }
}

void DirectMethod::update_points(const Eigen::VectorXd &sph_coeff,
                                 const std::pair<std::vector<double>, std::vector<int>> &x0,
                                 const std::pair<std::vector<double>, std::vector<int>> &x1,
                                 const std::pair<interp_func_pair, interp_func_pair> &interp_2d_plane,
                                 const Eigen::MatrixXd &points_transformed,
                                 std::pair<std::vector<double>, std::vector<int>> &x_2d,
                                 std::pair<std::vector<double>, std::vector<int>> &y_2d,
                                 Eigen::Matrix<double, 3, Eigen::Dynamic> &points_transformed_recons) {
    if (transformed_conformal_coord.size() == 0) {
        get_transformed_conformal_coord(x0, x1);
    }
    Eigen::Matrix<double, 3, Eigen::Dynamic> points_transformed_update;
    bool is_interpolared = true;
    bool is_conformal_back_projection = false;
    if (direct_method_config.update_points_method == "center_movement") {
        Eigen::Vector3d center_coord_pixel;
        Eigen::Vector3d center_coord_init;
        center_coord_pixel << static_cast<double>(img_interp_valid.cols()) / 2.0,
            static_cast<double>(img_interp_valid.rows()) / 2.0, 1;
        center_coord_init = initial_warp_matrix.inverse() * center_coord_pixel;
        center_coord_init(0) *= x_pixel_interval;
        center_coord_init(1) *= y_pixel_interval;

        Eigen::Vector3d center_coord_opti;
        center_coord_opti = get_warp_matrix().inverse() * center_coord_pixel;
        center_coord_opti(0) *= x_pixel_interval;
        center_coord_opti(1) *= y_pixel_interval;

        Eigen::MatrixXd center_coord(3, 2);
        center_coord << center_coord_init, center_coord_opti;
        center_coord.transposeInPlace();
        std::pair<std::vector<double>, std::vector<int>> x_center_2d =
            linear_interpolation_2(center_coord, interp_2d_plane.first);
        std::pair<std::vector<double>, std::vector<int>> y_center_2d =
            linear_interpolation_2(center_coord, interp_2d_plane.second, default_boundary, x_center_2d.first);
        if (y_center_2d.second.size() == 2) {
            Eigen::Matrix3d T_shift_2d = Eigen::Matrix3d::Identity();
            T_shift_2d(0, 2) = x_center_2d.first[1] - x_center_2d.first[0];
            T_shift_2d(1, 2) = y_center_2d.first[1] - y_center_2d.first[0];
            Eigen::MatrixXd points_transformed_homo = Eigen::MatrixXd::Ones(points_transformed.rows(), 3);
            points_transformed_homo(Eigen::all, Eigen::seq(0, 1)) = points_transformed(Eigen::all, Eigen::seq(0, 1));
            points_transformed_update = T_shift_2d * points_transformed_homo.transpose();
            is_interpolared = false;
        } else {
            is_conformal_back_projection = true;
        }
    } else if ((direct_method_config.update_points_method == "conformal_back_projection") ||
               is_conformal_back_projection) {
        x_2d = linear_interpolation_2(transformed_conformal_coord, interp_2d_plane.first, default_boundary, x1.first);
        y_2d =
            linear_interpolation_2(transformed_conformal_coord, interp_2d_plane.second, default_boundary, x_2d.first);
    }
    if (!is_interpolared) {
        y_2d.second = transformed_conformal_valid_idx; // y_2d connect with outside
    }
    points_transformed_recons.resize(3, y_2d.second.size());
    if (!y_2d.second.empty()) {
        if (SH_table_config_ptr->is_speed_up) {
            points_transformed_recons.row(2) = curl::invLeastSquaresSHT_table<double>(
                sph_coeff, transformed_conformal_sph_dirs(y_2d.second, Eigen::all), *SH_table_config_ptr);
        } else {
            points_transformed_recons.row(2) = curl::invLeastSquaresSHT<double>(
                sph_coeff, transformed_conformal_sph_dirs(y_2d.second, Eigen::all), SH_table_config_ptr->max_SH_degree);
        }
    }
    if (!is_interpolared) {
        points_transformed_recons.row(0) = points_transformed_update.row(0)(y_2d.second);
        points_transformed_recons.row(1) = points_transformed_update.row(1)(y_2d.second);
    } else {
        points_transformed_recons.row(0) =
            Eigen::Map<Eigen::VectorXd>(x_2d.first.data(), x_2d.first.size())(y_2d.second);
        points_transformed_recons.row(1) =
            Eigen::Map<Eigen::VectorXd>(y_2d.first.data(), y_2d.first.size())(y_2d.second);
    }
}

Eigen::VectorXd DirectMethod::update_sph_coeff(const Eigen::VectorXd &sph_coeff,
                                               const std::pair<std::vector<double>, std::vector<int>> &x0,
                                               const std::pair<std::vector<double>, std::vector<int>> &x1,
                                               const Eigen::VectorXd &height_value) {
    assert((SH_table_config_ptr->max_SH_degree + 1) * 2 <= SH_table_config_ptr->azi_rso);
    assert((SH_table_config_ptr->max_SH_degree + 1) * 2 <= SH_table_config_ptr->elev_rso);
    if (transformed_conformal_coord.size() == 0) {
        get_transformed_conformal_coord(x0, x1);
    }
    // ToDo: two schemes of updating the sph coefficients
    // ToDo: method 1, use the SH_TABLE resolution, project (degree+1)*2^2 points and
    // interp_valid_mat on grid
    double azi_step = (SH_table_config_ptr->azi_rso - 1) / ((SH_table_config_ptr->max_SH_degree + 1) * 2 - 1);
    azi_step = round(azi_step);
    double elev_step = (SH_table_config_ptr->elev_rso - 1) / ((SH_table_config_ptr->max_SH_degree + 1) * 2 - 1);
    elev_step = round(elev_step);
    Eigen::VectorXd std_azi_idx = curl::range<double>(0, SH_table_config_ptr->azi_rso, azi_step);
    Eigen::VectorXd std_elev_idx = curl::range<double>(0, SH_table_config_ptr->elev_rso, elev_step);
    Eigen::VectorXi std_idx(std_azi_idx.size() * std_elev_idx.size());
    for (int i = 0; i < std_azi_idx.size(); ++i) {
        std_idx(Eigen::seq(i * std_elev_idx.size(), (i + 1) * std_elev_idx.size() - 1)) =
            (std_azi_idx(i) * SH_table_config_ptr->elev_rso + std_elev_idx.array()).cast<int>();
    }
    Eigen::VectorXd std_residual = Eigen::VectorXd::Zero(std_azi_idx.size() * std_elev_idx.size());
    // get current scan on grid
    double azi_low = SH_table_config_ptr->get_azi_low();
    double azi_high = SH_table_config_ptr->get_azi_high();
    double elev_low = SH_table_config_ptr->get_elev_low();
    double elev_high = SH_table_config_ptr->get_elev_high();
    // get the index of the nearest azimuth and elevation from the table
    Eigen::VectorXi azi_idx = ((transformed_conformal_sph_dirs(transformed_conformal_valid_idx, 0).array() - azi_low) /
                               (azi_high - azi_low) * (SH_table_config_ptr->azi_rso - 1))
                                  .array()
                                  .round()
                                  .cast<int>();
    Eigen::VectorXi elev_idx =
        ((transformed_conformal_sph_dirs(transformed_conformal_valid_idx, 1).array() - elev_low) /
         (elev_high - elev_low) * (SH_table_config_ptr->elev_rso - 1))
            .array()
            .round()
            .cast<int>();
    Eigen::VectorXi scan_idx_unsorted = azi_idx * SH_table_config_ptr->elev_rso + elev_idx;
    // calculate residual
    Eigen::MatrixXd new_SH_table = SH_table_config_ptr->SH_table(
        scan_idx_unsorted, Eigen::seq(0, pow(SH_table_config_ptr->max_SH_degree + 1, 2) - 1));
    Eigen::VectorXd F = new_SH_table * sph_coeff(Eigen::seq(0, pow(SH_table_config_ptr->max_SH_degree + 1, 2) - 1));
    Eigen::VectorXd scan_residual_unsorted = height_value(transformed_conformal_valid_idx) - F;
    // detect repetition
    std::vector<int> idx_sort(scan_idx_unsorted.size(), 0);
    std::iota(idx_sort.begin(), idx_sort.end(), 0);
    std::sort(idx_sort.begin(), idx_sort.end(),
              [&](const int &a, const int &b) -> bool { return (scan_idx_unsorted(a) < scan_idx_unsorted(b)); });
    Eigen::VectorXi scan_idx = scan_idx_unsorted(idx_sort);
    Eigen::VectorXd scan_residual = scan_residual_unsorted(idx_sort);
    Eigen::VectorXd sph_coeff_update = sph_coeff;
    std::vector<int> repeat_std_idx;
    std::vector<int> repeat_scan_idx;
    std::vector<int> unrepeat_scan_idx;
    int j = 0;
    for (int i = 0; i < scan_idx.size(); ++i) {
        while (std_idx(j) < scan_idx(i) && (j < std_idx.size() - 1)) {
            ++j;
        }
        if (std_idx(j) == scan_idx(i)) {
            repeat_std_idx.push_back(j);
            repeat_scan_idx.push_back(i);
        } else {
            unrepeat_scan_idx.push_back(i);
        }
    }

    // update std_residual
    if (repeat_std_idx.size() != 0) {
        std_residual(repeat_std_idx) = scan_residual(repeat_scan_idx);
    }
    // integrate idx and residual
    Eigen::VectorXi SH_table_idx(std_idx.size() + unrepeat_scan_idx.size());
    Eigen::VectorXd residual(std_idx.size() + unrepeat_scan_idx.size());
    if (unrepeat_scan_idx.size() != 0) {
        SH_table_idx << std_idx, scan_idx(unrepeat_scan_idx);
        residual << std_residual, scan_residual(unrepeat_scan_idx);
    } else {
        SH_table_idx << std_idx;
        residual << std_residual;
    }
    // update coefficients
    curl::IRF_least_square_table(residual, SH_table_idx, *SH_table_config_ptr, 1, 1, sph_coeff_update);

    return sph_coeff_update;
    // ToDo: method 2, use the (degree+1)*2^2 resolution, replace the nearest (degree+1)*2^2 grid
    // points
}

void DirectMethod::update_sph_coeff_residual(const Eigen::VectorXd &sph_coeff,
                                             const std::pair<std::vector<double>, std::vector<int>> &x0,
                                             const std::pair<std::vector<double>, std::vector<int>> &x1,
                                             const Eigen::VectorXd &height_value,
                                             std::pair<Eigen::VectorXd, int> &residual_sum_pair) {
    assert((SH_table_config_ptr->max_SH_degree + 1) * 2 <= SH_table_config_ptr->azi_rso);
    assert((SH_table_config_ptr->max_SH_degree + 1) * 2 <= SH_table_config_ptr->elev_rso);
    if (transformed_conformal_coord.size() == 0) {
        get_transformed_conformal_coord(x0, x1);
    }
    // get current scan on grid
    double azi_low = SH_table_config_ptr->get_azi_low();
    double azi_high = SH_table_config_ptr->get_azi_high();
    double elev_low = SH_table_config_ptr->get_elev_low();
    double elev_high = SH_table_config_ptr->get_elev_high();
    // get the index of the nearest azimuth and elevation from the table
    Eigen::VectorXi azi_idx = ((transformed_conformal_sph_dirs(transformed_conformal_valid_idx, 0).array() - azi_low) /
                               (azi_high - azi_low) * (SH_table_config_ptr->azi_rso - 1))
                                  .array()
                                  .round()
                                  .cast<int>();
    Eigen::VectorXi elev_idx =
        ((transformed_conformal_sph_dirs(transformed_conformal_valid_idx, 1).array() - elev_low) /
         (elev_high - elev_low) * (SH_table_config_ptr->elev_rso - 1))
            .array()
            .round()
            .cast<int>();
    Eigen::VectorXi scan_idx_unsorted = azi_idx * SH_table_config_ptr->elev_rso + elev_idx;
    // calculate residual
    Eigen::MatrixXd new_SH_table = SH_table_config_ptr->SH_table(
        scan_idx_unsorted, Eigen::seq(0, pow(SH_table_config_ptr->max_SH_degree + 1, 2) - 1));
    Eigen::VectorXd F = new_SH_table * sph_coeff(Eigen::seq(0, pow(SH_table_config_ptr->max_SH_degree + 1, 2) - 1));
    residual_sum_pair.first(scan_idx_unsorted) += (height_value(transformed_conformal_valid_idx) - F);
    ++residual_sum_pair.second;
}