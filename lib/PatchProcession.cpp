//
// Created by zkc on 19/06/23.
//

#include "PatchProcession.h"
// this constructor is for test
template <typename BasicType>
PatchProcession<BasicType>::PatchProcession(const std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                                            const std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                                            std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                                            const std::pair<interp_func_pair, interp_func_pair> &_conformal_mapping,
                                            const Eigen::MatrixX<BasicType> &_v_local,
                                            const Eigen::VectorX<BasicType> &_map_x,
                                            const Eigen::VectorX<BasicType> &_map_y,
                                            const Eigen::VectorX<BasicType> &_sph_coeff, int _SH_degree)
    : direct_method_config_ptr(_direct_method_config_ptr), SH_table_config_ptr(_SH_table_config_ptr),
      debug_config_ptr(_debug_config_ptr), sph_coeff(_sph_coeff), SH_degree(_SH_degree) {
    residual_counter = 0;
    sph_residuals.resize(SH_table_config_ptr->update_azi_rso * SH_table_config_ptr->update_elev_rso);
    sph_residuals.setZero();
    tmp_sph_residual.resize(SH_table_config_ptr->update_azi_rso * SH_table_config_ptr->update_elev_rso);
    tmp_sph_residual.setZero();
    min_X = _v_local.col(0).minCoeff();
    max_X = _v_local.col(0).maxCoeff();
    min_Y = _v_local.col(1).minCoeff();
    max_Y = _v_local.col(1).maxCoeff();
    min_x = 0;
    min_y = 0;
    max_x = _map_x.maxCoeff();
    max_y = _map_y.maxCoeff();
    calculate_I();
    // origion of the coordinate is at the left bottom corner rather than left top
    // calculate Gamma image size
    X_size = std::round((max_X - min_X) / direct_method_config_ptr->gamma_sampling_interval);
    Y_size = std::round((max_Y - min_Y) / direct_method_config_ptr->gamma_sampling_interval);
    calculate_Gamma(_conformal_mapping);
    calculate_delta();
}
// this constructor is used in odometry
// add icp version patch procession, with kd-tree initilalized
template <typename BasicType>
PatchProcession<BasicType>::PatchProcession(const std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                                            const std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                                            std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                                            const std::pair<interp_func_pair, interp_func_pair> &_conformal_mapping,
                                            const std::pair<interp_func_pair, interp_func_pair> &_inv_conformal_mapping,
                                            const Eigen::MatrixX<BasicType> &_v_local,
                                            const Eigen::VectorX<BasicType> &_map_x,
                                            const Eigen::VectorX<BasicType> &_map_y,
                                            const Eigen::VectorX<BasicType> &_sph_coeff, int _SH_degree, int _status)
    : direct_method_config_ptr(_direct_method_config_ptr), SH_table_config_ptr(_SH_table_config_ptr),
      debug_config_ptr(_debug_config_ptr), conformal_mapping(_conformal_mapping),
      inv_conformal_mapping(_inv_conformal_mapping), sph_coeff(_sph_coeff), SH_degree(_SH_degree), status(_status) {
    residual_counter = 0;
    sph_residuals.resize(SH_table_config_ptr->update_azi_rso * SH_table_config_ptr->update_elev_rso);
    sph_residuals.setZero();
    tmp_sph_residual.resize(SH_table_config_ptr->update_azi_rso * SH_table_config_ptr->update_elev_rso);
    tmp_sph_residual.setZero();
    min_X = _v_local.col(0).minCoeff();
    max_X = _v_local.col(0).maxCoeff();
    min_Y = _v_local.col(1).minCoeff();
    max_Y = _v_local.col(1).maxCoeff();
    min_x = 0;
    min_y = 0;
    max_x = _map_x.maxCoeff();
    max_y = _map_y.maxCoeff();
    calculate_I();
    // origion of the coordinate is at the left bottom corner rather than left top
    // calculate Gamma image size
    X_size = std::round((max_X - min_X) / direct_method_config_ptr->gamma_sampling_interval);
    Y_size = std::round((max_Y - min_Y) / direct_method_config_ptr->gamma_sampling_interval);
    calculate_Gamma(_conformal_mapping);
    calculate_delta();
    calculate_recons_v_local(_inv_conformal_mapping);
    if (status == curl::MEDIUM) {
        // TODO: initlize a kd-tree for this _v_local in world frame
    }
}

template <typename BasicType>
PatchProcession<BasicType>::PatchProcession(const std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                                            std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                                            std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                                            const std::pair<interp_func_pair, interp_func_pair> &_conformal_mapping,
                                            const std::pair<interp_func_pair, interp_func_pair> &_inv_conformal_mapping,
                                            interp_func_pair &_I_interp, const Eigen::MatrixX<BasicType> &_v_local,
                                            const Eigen::VectorX<BasicType> &_map_x,
                                            const Eigen::VectorX<BasicType> &_map_y,
                                            const Eigen::VectorX<BasicType> &_sph_coeff, int _SH_degree)
    : direct_method_config_ptr(_direct_method_config_ptr), SH_table_config_ptr(_SH_table_config_ptr),
      debug_config_ptr(_debug_config_ptr), conformal_mapping(_conformal_mapping),
      inv_conformal_mapping(_inv_conformal_mapping), I_interp(_I_interp), sph_coeff(_sph_coeff), SH_degree(_SH_degree) {
    sph_residuals.resize(SH_table_config_ptr->update_azi_rso * SH_table_config_ptr->update_elev_rso);
    sph_residuals.setZero();
    tmp_sph_residual.resize(SH_table_config_ptr->update_azi_rso * SH_table_config_ptr->update_elev_rso);
    tmp_sph_residual.setZero();
    min_X = _v_local.col(0).minCoeff();
    max_X = _v_local.col(0).maxCoeff();
    min_Y = _v_local.col(1).minCoeff();
    max_Y = _v_local.col(1).maxCoeff();
    min_x = 0;
    min_y = 0;
    max_x = _map_x.maxCoeff();
    max_y = _map_y.maxCoeff();
    calculate_I();
    // origion of the coordinate is at the left bottom corner rather than left top
    // calculate Gamma image size
    X_size = std::round((max_X - min_X) / direct_method_config_ptr->gamma_sampling_interval);
    Y_size = std::round((max_Y - min_Y) / direct_method_config_ptr->gamma_sampling_interval);
    calculate_Gamma(_conformal_mapping);
    calculate_delta();
    calculate_recons_v_local(_inv_conformal_mapping);
}

// constructor for map generation
template <typename BasicType>
PatchProcession<BasicType>::PatchProcession(const std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                                            const std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                                            std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                                            const std::pair<interp_func_pair, interp_func_pair> &_conformal_mapping,
                                            const std::pair<interp_func_pair, interp_func_pair> &_inv_conformal_mapping,
                                            const Eigen::VectorX<BasicType> &_sph_coeff, int _SH_degree)
    : direct_method_config_ptr(_direct_method_config_ptr), SH_table_config_ptr(_SH_table_config_ptr),
      debug_config_ptr(_debug_config_ptr), conformal_mapping(_conformal_mapping),
      inv_conformal_mapping(_inv_conformal_mapping), sph_coeff(_sph_coeff), SH_degree(_SH_degree) {
    std::vector<BasicType> X_vec, Y_vec;
    for (auto v_it = conformal_mapping.first.first.finite_vertices_begin();
         v_it != conformal_mapping.first.first.finite_vertices_end(); ++v_it) {
        X_vec.push_back(v_it->point().x());
        Y_vec.push_back(v_it->point().y());
    }
    min_X = *(std::min_element(X_vec.begin(), X_vec.end()));
    max_X = *(std::max_element(X_vec.begin(), X_vec.end()));
    min_Y = *(std::min_element(Y_vec.begin(), Y_vec.end()));
    max_Y = *(std::max_element(Y_vec.begin(), Y_vec.end()));
    std::vector<BasicType> x_vec, y_vec;
    for (auto v_it = inv_conformal_mapping.first.first.finite_vertices_begin();
         v_it != inv_conformal_mapping.first.first.finite_vertices_end(); ++v_it) {
        x_vec.push_back(v_it->point().x());
        y_vec.push_back(v_it->point().y());
    }
    min_x = 0;
    max_x = *(std::max_element(x_vec.begin(), x_vec.end()));
    min_y = 0;
    max_y = *(std::max_element(y_vec.begin(), y_vec.end()));
    // origion of the coordinate is at the left bottom corner rather than left top
    // calculate Gamma image size
    X_size = std::round((max_X - min_X) / direct_method_config_ptr->gamma_sampling_interval);
    Y_size = std::round((max_Y - min_Y) / direct_method_config_ptr->gamma_sampling_interval);
}

// member functions
template <typename BasicType> void PatchProcession<BasicType>::calculate_I() {
    // 1 step: generate a query grid on conformal map
    if (max_x <= max_y) {
        x_size = direct_method_config_ptr->minimum_img_rso;
        y_size = int(round(max_y / max_x * x_size));
    } else {
        y_size = direct_method_config_ptr->minimum_img_rso;
        x_size = int(round(max_x / max_y * y_size));
    }

    BasicType azi_low = SH_table_config_ptr->get_azi_low();
    BasicType azi_high = SH_table_config_ptr->get_azi_high();
    BasicType elev_low = SH_table_config_ptr->get_elev_low();
    BasicType elev_high = SH_table_config_ptr->get_elev_high();
    // origion of the coordinate is at the left bottom corner rather than left top
    // pay attention that this is elev_high to elev_low
    curl::mesh_grid<BasicType>(azi_low, azi_high, x_size, elev_high, elev_low, y_size, sph_dirs);

    // 3 step: use sph_coeff to recons the height value
    Eigen::VectorX<BasicType> height_recons;
    if (SH_table_config_ptr->is_speed_up) {
        height_recons = curl::invLeastSquaresSHT_table<BasicType>(sph_coeff, sph_dirs, *SH_table_config_ptr, SH_degree);
    } else {
        height_recons = curl::invLeastSquaresSHT<BasicType>(sph_coeff, sph_dirs, SH_degree);
    }
    // origion of the coordinate is at the left bottom corner rather than left top
    //    I_lock.lock();
    I = Eigen::Map<Eigen::MatrixX<BasicType>>(height_recons.data(), y_size, x_size);
    //    I_lock.unlock();
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/hight_recons_img.txt",
                                       Eigen::MatrixXd(I.template cast<double>()));
    }
}

template <typename BasicType>
void PatchProcession<BasicType>::calculate_Gamma(
    const std::pair<interp_func_pair, interp_func_pair> &_conformal_mapping) {
    Eigen::MatrixX<BasicType> query_points;
    curl::mesh_grid<BasicType>(min_X, max_X, X_size, max_Y, min_Y, Y_size,
                               query_points); // pay attention that this is from max_Y to min_Y
    BasicType x_boundary[2] = {min_x, max_x};
    std::pair<std::vector<BasicType>, std::vector<int>> S_Gamma_x =
        linear_interpolation_2(query_points, _conformal_mapping.first, x_boundary);
    BasicType y_boundary[2] = {min_y, max_y};
    std::pair<std::vector<BasicType>, std::vector<int>> S_Gamma_y =
        linear_interpolation_2(query_points, _conformal_mapping.second, y_boundary, S_Gamma_x.first);
    Gamma_x = Eigen::Map<Eigen::MatrixX<BasicType>>(S_Gamma_x.first.data(), Y_size, X_size);
    Gamma_y = Eigen::Map<Eigen::MatrixX<BasicType>>(S_Gamma_y.first.data(), Y_size, X_size);
    //    std::cout << "Gamma size: " << Gamma_x.size() + Gamma_y.size() << std::endl;
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/Gamma_x.txt",
                                       Eigen::MatrixXd(Gamma_x.template cast<double>()));
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/Gamma_y.txt",
                                       Eigen::MatrixXd(Gamma_y.template cast<double>()));
    }
}

template <typename BasicType>
void PatchProcession<BasicType>::calculate_inv_Gamma(
    const std::pair<interp_func_pair, interp_func_pair> &_inv_conformal_mapping) {
    Eigen::MatrixX<BasicType> query_points;
    curl::mesh_grid<BasicType>(min_x, max_x, X_size, max_y, min_y, Y_size,
                               query_points); // pay attention that this is from max_Y to min_Y
    BasicType X_boundary[2] = {min_X, max_X};
    std::pair<std::vector<BasicType>, std::vector<int>> S_inv_Gamma_x =
        linear_interpolation_2(query_points, _inv_conformal_mapping.first, X_boundary);
    BasicType Y_boundary[2] = {min_Y, max_Y};
    std::pair<std::vector<BasicType>, std::vector<int>> S_inv_Gamma_y =
        linear_interpolation_2(query_points, _inv_conformal_mapping.second, Y_boundary, S_inv_Gamma_x.first);
    inv_Gamma_x = Eigen::Map<Eigen::MatrixX<BasicType>>(S_inv_Gamma_x.first.data(), Y_size, X_size);
    inv_Gamma_y = Eigen::Map<Eigen::MatrixX<BasicType>>(S_inv_Gamma_y.first.data(), Y_size, X_size);
    //    std::cout << "Gamma size: " << Gamma_x.size() + Gamma_y.size() << std::endl;
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/inv_Gamma_x.txt",
                                       Eigen::MatrixXd(inv_Gamma_x.template cast<double>()));
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/inv_Gamma_y.txt",
                                       Eigen::MatrixXd(inv_Gamma_y.template cast<double>()));
    }
}

template <typename BasicType> void PatchProcession<BasicType>::calculate_delta() {
    // calculate gradient for I
    BasicType scale_I_Gx = (max_x - min_x) / static_cast<BasicType>(x_size - 1);
    I_Gx = curl::eigen_central_gradient<BasicType>(I, "x", scale_I_Gx);
    BasicType scale_I_Gy = (max_y - min_y) / static_cast<BasicType>(y_size - 1);
    I_Gy = curl::eigen_central_gradient<BasicType>(I, "y", scale_I_Gy);
    // calculate gradient for Gamma_x
    BasicType scale_Gamma_Gx = (max_X - min_X) / static_cast<BasicType>(X_size - 1);
    Gamma_x_Gx = curl::eigen_central_gradient<BasicType>(Gamma_x, "x", scale_Gamma_Gx);
    BasicType scale_Gamma_Gy = (max_Y - min_Y) / static_cast<BasicType>(Y_size - 1);
    Gamma_x_Gy = curl::eigen_central_gradient<BasicType>(Gamma_x, "y", scale_Gamma_Gy);
    // calculate gradient for Gamma_y
    Gamma_y_Gx = curl::eigen_central_gradient<BasicType>(Gamma_y, "x", scale_Gamma_Gx);
    Gamma_y_Gy = curl::eigen_central_gradient<BasicType>(Gamma_y, "y", scale_Gamma_Gy);
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/I_Gx.txt",
                                       Eigen::MatrixXd(I_Gx.template cast<double>()));
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/I_Gy.txt",
                                       Eigen::MatrixXd(I_Gy.template cast<double>()));
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/Gamma_x_Gx.txt",
                                       Eigen::MatrixXd(Gamma_x_Gx.template cast<double>()));
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/Gamma_x_Gy.txt",
                                       Eigen::MatrixXd(Gamma_x_Gy.template cast<double>()));
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/Gamma_y_Gx.txt",
                                       Eigen::MatrixXd(Gamma_y_Gx.template cast<double>()));
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/Gamma_y_Gy.txt",
                                       Eigen::MatrixXd(Gamma_y_Gy.template cast<double>()));
    }
}

template <typename BasicType>
void PatchProcession<BasicType>::calculate_recons_v_local(
    const std::pair<interp_func_pair, interp_func_pair> &_inv_conformal_mapping) {
    Eigen::MatrixX<BasicType> xy_grid;
    curl::mesh_grid<BasicType>(min_x, max_x, x_size, max_y, min_y, y_size, xy_grid);
    BasicType X_boundary[2] = {min_X, max_X};
    std::pair<std::vector<BasicType>, std::vector<int>> v_local_X =
        linear_interpolation_2(xy_grid, _inv_conformal_mapping.first, X_boundary);
    BasicType Y_boundary[2] = {min_Y, max_Y};
    std::pair<std::vector<BasicType>, std::vector<int>> v_local_Y =
        linear_interpolation_2(xy_grid, _inv_conformal_mapping.second, Y_boundary, v_local_X.first);
    valid_recons_idx = v_local_Y.second;
    recons_v_local.resize(valid_recons_idx.size(), 3);
    recons_v_local.col(0) =
        Eigen::Map<Eigen::VectorX<BasicType>>(v_local_X.first.data(), v_local_X.first.size())(valid_recons_idx);
    recons_v_local.col(1) =
        Eigen::Map<Eigen::VectorX<BasicType>>(v_local_Y.first.data(), v_local_Y.first.size())(valid_recons_idx);
    //    I_lock.lock();
    recons_v_local.col(2) = Eigen::Map<Eigen::VectorX<BasicType>>(I.data(), I.size())(valid_recons_idx);
    //    I_lock.unlock();
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_I(const Eigen::MatrixX<BasicType> &I_matrix, const BasicType x,
                                              const BasicType y) const {
    BasicType result = T_INVALID;
    if ((x >= min_x) && (x <= max_x) && (y >= min_y) && (y <= max_y)) {
        BasicType u = (x - min_x) / (max_x - min_x) * static_cast<BasicType>(x_size - 1);
        // origin of the coordinate is at the left bottom corner
        BasicType v = (max_y - y) / (max_y - min_y) * static_cast<BasicType>(y_size - 1);
        // bilinear interpolation https://en.wikipedia.org/wiki/Bilinear_interpolation
        int ceil_v = static_cast<int>(std::ceil(v));
        int ceil_u = static_cast<int>(std::ceil(u));
        //        if ((ceil_v > (I_matrix.rows() - 1)) || (ceil_u > (I_matrix.cols() - 1))) {
        //            return result;
        //        }
        BasicType Q11 = I_matrix(ceil_v, static_cast<int>(std::floor(u)));
        BasicType Q21 = I_matrix(ceil_v, ceil_u);
        BasicType Q12 = I_matrix(static_cast<int>(std::floor(v)), static_cast<int>(std::floor(u)));
        BasicType Q22 = I_matrix(static_cast<int>(std::floor(v)), ceil_u);
        if ((Q11 != T_INVALID) && (Q21 != T_INVALID) && (Q12 != T_INVALID) && (Q22 != T_INVALID)) {
            Eigen::Vector2<BasicType> x_vec, y_vec;
            x_vec << std::ceil(u) - u, u - std::floor(u);
            y_vec << v - std::floor(v), std::ceil(v) - v;
            Eigen::Matrix2<BasicType> Q_matrix;
            Q_matrix << Q11, Q12, Q21, Q22;
            result = x_vec.transpose() * Q_matrix * y_vec;
        }
    }
    return result;
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_Gamma(const Eigen::MatrixX<BasicType> &Gamma_matrix, const BasicType X,
                                                  const BasicType Y) const {
    BasicType result = T_INVALID;
    if ((X >= min_X) && (X <= max_X) && (Y >= min_Y) && (Y <= max_Y)) {
        BasicType u = (X - min_X) / (max_X - min_X) * static_cast<BasicType>(X_size - 1);
        // origin of the coordinate is at the left bottom corner
        BasicType v = (max_Y - Y) / (max_Y - min_Y) * static_cast<BasicType>(Y_size - 1);
        int ceil_v = static_cast<int>(std::ceil(v));
        int ceil_u = static_cast<int>(std::ceil(u));
        //        if ((ceil_v > (Gamma_matrix.rows() - 1)) || (ceil_u > (Gamma_matrix.cols() - 1)))
        //        {
        //            return result;
        //        }
        // bilinear interpolation https://en.wikipedia.org/wiki/Bilinear_interpolation
        BasicType Q11 = Gamma_matrix(ceil_v, static_cast<int>(std::floor(u)));
        BasicType Q21 = Gamma_matrix(ceil_v, ceil_u);
        BasicType Q12 = Gamma_matrix(static_cast<int>(std::floor(v)), static_cast<int>(std::floor(u)));
        BasicType Q22 = Gamma_matrix(static_cast<int>(std::floor(v)), ceil_u);
        if ((Q11 != T_INVALID) && (Q21 != T_INVALID) && (Q12 != T_INVALID) && (Q22 != T_INVALID)) {
            Eigen::Vector2<BasicType> X_vec, Y_vec;
            X_vec << std::ceil(u) - u, u - std::floor(u);
            Y_vec << v - std::floor(v), std::ceil(v) - v;
            Eigen::Matrix2<BasicType> Q_matrix;
            Q_matrix << Q11, Q12, Q21, Q22;
            result = X_vec.transpose() * Q_matrix * Y_vec;
        }
    }
    return result;
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_inv_Gamma(const Eigen::MatrixX<BasicType> &inv_Gamma_matrix,
                                                      const BasicType x, const BasicType y) const {
    BasicType result = T_INVALID;
    if ((x >= min_x) && (x <= max_x) && (y >= min_y) && (y <= max_y)) {
        BasicType u = (x - min_x) / (max_x - min_x) * static_cast<BasicType>(X_size - 1);
        // origin of the coordinate is at the left bottom corner
        BasicType v = (max_y - y) / (max_y - min_y) * static_cast<BasicType>(Y_size - 1);
        int ceil_v = static_cast<int>(std::ceil(v));
        int ceil_u = static_cast<int>(std::ceil(u));
        //        if ((ceil_v > (Gamma_matrix.rows() - 1)) || (ceil_u > (Gamma_matrix.cols() - 1)))
        //        {
        //            return result;
        //        }
        // bilinear interpolation https://en.wikipedia.org/wiki/Bilinear_interpolation
        BasicType Q11 = inv_Gamma_matrix(ceil_v, static_cast<int>(std::floor(u)));
        BasicType Q21 = inv_Gamma_matrix(ceil_v, ceil_u);
        BasicType Q12 = inv_Gamma_matrix(static_cast<int>(std::floor(v)), static_cast<int>(std::floor(u)));
        BasicType Q22 = inv_Gamma_matrix(static_cast<int>(std::floor(v)), ceil_u);
        if ((Q11 != T_INVALID) && (Q21 != T_INVALID) && (Q12 != T_INVALID) && (Q22 != T_INVALID)) {
            Eigen::Vector2<BasicType> X_vec, Y_vec;
            X_vec << std::ceil(u) - u, u - std::floor(u);
            Y_vec << v - std::floor(v), std::ceil(v) - v;
            Eigen::Matrix2<BasicType> Q_matrix;
            Q_matrix << Q11, Q12, Q21, Q22;
            result = X_vec.transpose() * Q_matrix * Y_vec;
        }
    }
    return result;
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_I_interp(const BasicType x, const BasicType y) const {
    Eigen::MatrixX<BasicType> query_points(1, 2);
    query_points << x, y;
    BasicType tmp_bdy[2] = {std::numeric_limits<BasicType>::lowest(), std::numeric_limits<BasicType>::max()};
    std::pair<std::vector<BasicType>, std::vector<int>> S_I = linear_interpolation_2(query_points, I_interp, tmp_bdy);
    return S_I.first[0];
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_I_Gx_interp(const BasicType x, const BasicType y) const {
    BasicType x_left = x - 0.01;
    BasicType Z_left = fetch_I_interp(x_left, y);
    if (Z_left == T_INVALID) {
        return T_INVALID;
    }
    BasicType x_right = x + 0.01;
    BasicType Z_right = fetch_I_interp(x_right, y);
    if (Z_right == T_INVALID) {
        return T_INVALID;
    }
    BasicType gadient = (Z_right - Z_left) / (x_right - x_left);
    return gadient;
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_I_Gy_interp(const BasicType x, const BasicType y) const {
    Eigen::MatrixX<BasicType> query_points_left(1, 2);
    BasicType y_down = y - 0.01;
    BasicType Z_down = fetch_I_interp(x, y_down);
    if (Z_down == T_INVALID) {
        return T_INVALID;
    }
    BasicType y_up = y + 0.01;
    BasicType Z_up = fetch_I_interp(x, y_up);
    if (Z_up == T_INVALID) {
        return T_INVALID;
    }
    BasicType gadient = (Z_up - Z_down) / (y_up - y_down);
    return gadient;
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_Gamma_x_interp(const BasicType X, const BasicType Y) const {
    Eigen::MatrixX<BasicType> query_points(1, 2);
    query_points << X, Y;
    BasicType x_boundary[2] = {min_x, max_x};
    std::pair<std::vector<BasicType>, std::vector<int>> S_Gamma_x =
        linear_interpolation_2(query_points, conformal_mapping.first, x_boundary);
    return S_Gamma_x.first[0];
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_Gamma_y_interp(const BasicType X, const BasicType Y) const {
    Eigen::MatrixX<BasicType> query_points(1, 2);
    query_points << X, Y;
    BasicType y_boundary[2] = {min_y, max_y};
    std::pair<std::vector<BasicType>, std::vector<int>> S_Gamma_y =
        linear_interpolation_2(query_points, conformal_mapping.second, y_boundary);
    return S_Gamma_y.first[0];
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_Gamma_x_Gx_interp(const BasicType X, const BasicType Y) const {
    BasicType X_left = X - 0.01;
    BasicType x_left = fetch_Gamma_x_interp(X_left, Y);
    if (x_left == T_INVALID) {
        return T_INVALID;
    }
    BasicType X_right = X + 0.01;
    BasicType x_right = fetch_Gamma_x_interp(X_right, Y);
    if (x_right == T_INVALID) {
        return T_INVALID;
    }
    BasicType gadient = (x_right - x_left) / (X_right - X_left);
    return gadient;
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_Gamma_x_Gy_interp(const BasicType X, const BasicType Y) const {
    BasicType Y_down = Y - 0.01;
    BasicType y_down = fetch_Gamma_x_interp(X, Y_down);
    if (y_down == T_INVALID) {
        return T_INVALID;
    }
    BasicType Y_up = Y + 0.01;
    BasicType y_up = fetch_Gamma_x_interp(X, Y_up);
    if (y_up == T_INVALID) {
        return T_INVALID;
    }
    BasicType gadient = (y_up - y_down) / (Y_up - Y_down);
    return gadient;
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_Gamma_y_Gx_interp(const BasicType X, const BasicType Y) const {
    BasicType X_left = X - 0.01;
    BasicType x_left = fetch_Gamma_y_interp(X_left, Y);
    if (x_left == T_INVALID) {
        return T_INVALID;
    }
    BasicType X_right = X + 0.01;
    BasicType x_right = fetch_Gamma_y_interp(X_right, Y);
    if (x_right == T_INVALID) {
        return T_INVALID;
    }
    BasicType gadient = (x_right - x_left) / (X_right - X_left);
    return gadient;
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_Gamma_y_Gy_interp(const BasicType X, const BasicType Y) const {
    BasicType Y_down = Y - 0.01;
    BasicType y_down = fetch_Gamma_y_interp(X, Y_down);
    if (y_down == T_INVALID) {
        return T_INVALID;
    }
    BasicType Y_up = Y + 0.01;
    BasicType y_up = fetch_Gamma_y_interp(X, Y_up);
    if (y_up == T_INVALID) {
        return T_INVALID;
    }
    BasicType gadient = (y_up - y_down) / (Y_up - Y_down);
    return gadient;
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_inv_Gamma_x_interp(const BasicType x, const BasicType y) const {
    Eigen::MatrixX<BasicType> query_points(1, 2);
    query_points << x, y;
    BasicType X_boundary[2] = {min_X, max_X};
    std::pair<std::vector<BasicType>, std::vector<int>> S_Gamma_X =
        linear_interpolation_2(query_points, inv_conformal_mapping.first, X_boundary);
    return S_Gamma_X.first[0];
}

template <typename BasicType>
BasicType PatchProcession<BasicType>::fetch_inv_Gamma_y_interp(const BasicType x, const BasicType y) const {
    Eigen::MatrixX<BasicType> query_points(1, 2);
    query_points << x, y;
    BasicType Y_boundary[2] = {min_Y, max_Y};
    std::pair<std::vector<BasicType>, std::vector<int>> S_Gamma_Y =
        linear_interpolation_2(query_points, inv_conformal_mapping.second, Y_boundary);
    return S_Gamma_Y.first[0];
}

template <typename BasicType>
bool PatchProcession<BasicType>::update_sph_residuals(const Eigen::MatrixX<BasicType> &v_obj) {
    bool is_recons_updated = false;
    tmp_sph_residual.setZero();
    for (int i = 0; i < v_obj.cols(); ++i) {
        BasicType x = fetch_Gamma(Gamma_x, v_obj(0, i), v_obj(1, i));
        //        double x = patch_procession_ptr->fetch_Gamma_x_interp(p_o(0), p_o(1));
        bool is_fault = false;
        if (x == T_INVALID) {
            is_fault = true;
        }
        BasicType y = fetch_Gamma(Gamma_y, v_obj(0, i), v_obj(1, i));
        //        double y = patch_procession_ptr->fetch_Gamma_y_interp(p_o(0), p_o(1));
        if (y == T_INVALID) {
            is_fault = true;
        }
        I_lock.lock();
        BasicType I_value = fetch_I(I, x, y);
        I_lock.unlock();
        if (I_value == T_INVALID) {
            is_fault = true;
        }
        if (!is_fault) {
            int x_idx = std::round((x - min_x) / (max_x - min_x) *
                                   static_cast<BasicType>(SH_table_config_ptr->update_azi_rso - 1));
            int y_idx = std::round((y - min_y) / (max_y - min_y) *
                                   static_cast<BasicType>(SH_table_config_ptr->update_elev_rso - 1));
            BasicType x_distance = std::round(
                abs(static_cast<BasicType>(x_idx) -
                    (x - min_x) / (max_x - min_x) * static_cast<BasicType>(SH_table_config_ptr->update_azi_rso - 1)) *
                (static_cast<BasicType>(SH_table_config_ptr->azi_rso) /
                 static_cast<BasicType>(SH_table_config_ptr->update_azi_rso)));
            BasicType y_distance = std::round(
                abs(static_cast<BasicType>(y_idx) -
                    (y - min_y) / (max_y - min_y) * static_cast<BasicType>(SH_table_config_ptr->update_elev_rso - 1)) *
                (static_cast<BasicType>(SH_table_config_ptr->elev_rso) /
                 static_cast<BasicType>(SH_table_config_ptr->update_elev_rso)));
            BasicType distance = (x_distance * x_distance + y_distance * y_distance);
            BasicType weight;
            if (distance > 0) {
                weight = 1 / distance;
            } else {
                distance = 1;
            }
            BasicType res = (v_obj(2, i) - I_value) * weight;
            if (abs(res) > tmp_sph_residual(x_idx * SH_table_config_ptr->update_elev_rso + y_idx)) {
                tmp_sph_residual(x_idx * SH_table_config_ptr->update_elev_rso + y_idx) = res;
            }
        }
    }
    sph_residuals += tmp_sph_residual;
    ++residual_counter;
    if (residual_counter > SH_table_config_ptr->update_sph_counter_thres) {
        update_sph_coeff();
        // update the image
        update_I();
        update_recons();
        is_recons_updated = true;
        residual_counter = 0;
    }
    return is_recons_updated;
}

template <typename BasicType> void PatchProcession<BasicType>::update_sph_coeff() {
    sph_coeff = curl::IRF_least_square_table_update_sph<BasicType>(
        (sph_residuals / static_cast<BasicType>(residual_counter)), SH_table_config_ptr->update_idx,
        *SH_table_config_ptr, 1, 1, sph_coeff, SH_table_config_ptr->inv_method);
    sph_residuals.setZero();
}

template <typename BasicType> void PatchProcession<BasicType>::update_I() {
    // use sph_coeff to recons the height value
    Eigen::VectorX<BasicType> height_recons;
    if (SH_table_config_ptr->is_speed_up) {
        height_recons = curl::invLeastSquaresSHT_table<BasicType>(sph_coeff, sph_dirs, *SH_table_config_ptr, SH_degree);
    } else {
        height_recons = curl::invLeastSquaresSHT<BasicType>(sph_coeff, sph_dirs, SH_degree);
    }
    // origion of the coordinate is at the left bottom corner rather than left top
    // add lock here
    I_lock.lock();
    I = Eigen::Map<Eigen::MatrixX<BasicType>>(height_recons.data(), y_size, x_size);
    I_lock.unlock();
    // calculate gradient for I
    BasicType scale_I_Gx = (max_x - min_x) / static_cast<BasicType>(x_size - 1);
    I_lock.lock();
    I_Gx = curl::eigen_central_gradient<BasicType>(I, "x", scale_I_Gx);
    I_lock.unlock();
    BasicType scale_I_Gy = (max_y - min_y) / static_cast<BasicType>(y_size - 1);
    I_lock.lock();
    I_Gy = curl::eigen_central_gradient<BasicType>(I, "y", scale_I_Gy);
    I_lock.unlock();
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/hight_recons_img.txt",
                                       Eigen::MatrixXd(I.template cast<double>()));
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/I_Gx.txt",
                                       Eigen::MatrixXd(I_Gx.template cast<double>()));
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/I_Gy.txt",
                                       Eigen::MatrixXd(I_Gy.template cast<double>()));
    }
}

template <typename BasicType> void PatchProcession<BasicType>::update_recons() {
    I_lock.lock();
    recons_v_local.col(2) = Eigen::Map<Eigen::VectorX<BasicType>>(I.data(), I.size())(valid_recons_idx);
    I_lock.unlock();
}

template <typename BasicType>
Eigen::MatrixX<BasicType> PatchProcession<BasicType>::reconstruction(int minimum_rso, int degree, BasicType IQR_thres,
                                                                     const std::string &method) {
    // 1 step: generate a query grid on conformal map
    int recons_x_size, recons_y_size;
    if (max_x <= max_y) {
        recons_x_size = minimum_rso;
        recons_y_size = int(round(max_y / max_x * recons_x_size));
    } else {
        recons_y_size = minimum_rso;
        recons_x_size = int(round(max_x / max_y * recons_y_size));
    }
    BasicType azi_low = SH_table_config_ptr->get_azi_low();
    BasicType azi_high = SH_table_config_ptr->get_azi_high();
    BasicType elev_low = SH_table_config_ptr->get_elev_low();
    BasicType elev_high = SH_table_config_ptr->get_elev_high();
    // origion of the coordinate is at the left bottom corner rather than left top
    // pay attention that this is elev_high to elev_low
    Eigen::MatrixX<BasicType> recons_sph_dirs;
    curl::mesh_grid<BasicType>(azi_low, azi_high, recons_x_size, elev_high, elev_low, recons_y_size, recons_sph_dirs);
    // 3 step: use sph_coeff to recons the height value
    Eigen::VectorX<BasicType> height_recons;
    if (degree > SH_degree) {
        degree = SH_degree;
        std::cout << "Larger than the largest degree! Degree of Current Patch is: " << degree << std::endl;
    }
    if (SH_table_config_ptr->is_speed_up) {
        height_recons =
            curl::invLeastSquaresSHT_table<BasicType>(sph_coeff, recons_sph_dirs, *SH_table_config_ptr, degree);
    } else {
        height_recons = curl::invLeastSquaresSHT<BasicType>(sph_coeff, recons_sph_dirs, degree);
    }
    std::vector<BasicType> height_recons_vec(height_recons.begin(), height_recons.end());
    std::pair<BasicType, BasicType> height_recons_thres_pair;
    if (IQR_thres != 0) {
        height_recons_thres_pair = curl::IQR_thres_pair<BasicType>(height_recons_vec, IQR_thres);
    } else {
        height_recons_thres_pair.first = std::numeric_limits<BasicType>::lowest();
        height_recons_thres_pair.second = std::numeric_limits<BasicType>::max();
    }
    // generate corresponding xy grid
    Eigen::MatrixX<BasicType> recons_xy_grid;
    curl::mesh_grid<BasicType>(min_x, max_x, recons_x_size, max_y, min_y, recons_y_size, recons_xy_grid);
    // produce the reconsed point cloud
    Eigen::MatrixX<BasicType> reconsed_points(recons_x_size * recons_y_size, 3);
    if (method == "bilinear") {
        if (inv_Gamma_x.size() == 0 || inv_Gamma_y.size() == 0) {
            calculate_inv_Gamma(inv_conformal_mapping);
        }

        std::vector<int> valid_idx;
        for (int i = 0; i < recons_xy_grid.rows(); ++i) {
            BasicType X, Y;

            X = fetch_inv_Gamma(inv_Gamma_x, recons_xy_grid(i, 0), recons_xy_grid(i, 1));
            Y = fetch_inv_Gamma(inv_Gamma_y, recons_xy_grid(i, 0), recons_xy_grid(i, 1));

            if (X != T_INVALID && Y != T_INVALID && height_recons(i) > height_recons_thres_pair.first &&
                height_recons(i) < height_recons_thres_pair.second) {
                reconsed_points(i, 0) = X;
                reconsed_points(i, 1) = Y;
                reconsed_points(i, 2) = height_recons(i);
                valid_idx.push_back(i);
            }
        }
        return reconsed_points(valid_idx, Eigen::all);
    } else if (method == "nn") {
        BasicType X_boundary[2] = {min_X, max_X};
        std::pair<std::vector<BasicType>, std::vector<int>> v_local_X =
            linear_interpolation_2(recons_xy_grid, inv_conformal_mapping.first, X_boundary);
        BasicType Y_boundary[2] = {min_Y, max_Y};
        std::pair<std::vector<BasicType>, std::vector<int>> v_local_Y =
            linear_interpolation_2(recons_xy_grid, inv_conformal_mapping.second, Y_boundary, v_local_X.first);
        valid_recons_idx = v_local_Y.second;
        reconsed_points.col(0) = Eigen::Map<Eigen::VectorX<BasicType>>(v_local_X.first.data(), v_local_X.first.size());
        reconsed_points.col(1) = Eigen::Map<Eigen::VectorX<BasicType>>(v_local_Y.first.data(), v_local_Y.first.size());
        reconsed_points.col(2) = height_recons;
        return reconsed_points(valid_recons_idx, Eigen::all);
    }
}
template class PatchProcession<BT>; // this is very important