//
// Created by zkc on 23/03/23.
//

#ifndef CURL_SLAM_CURL_TOOLS_LIGHT_H
#define CURL_SLAM_CURL_TOOLS_LIGHT_H
#include "Eigen/Core"
#include "FileReaderBase.h"
#include "Timer.h"
#include "load_config.h"
#include "memory"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <random>
#include <utility>

namespace curl {
typedef Eigen::Matrix<double, Eigen::Dynamic, 3> Matrix3dCol;
typedef Eigen::Matrix<double, Eigen::Dynamic, 2> Matrix2dCol;

struct LidarConfig {
    int num_lasers = 0;
    int full_num_lasers = 0;
    int img_length = 0;
    int row_times = 0;
    int col_times = 0;
    double full_fov_up = 0;
    double fov_down = 0;
    double length_percentage = 0;
    double min_length = 0;
    double full_fov = 0;
    double u_res = 0;
    double fov = 0;
    double fov_up = 0;
    int is_density_enhance = 0;
    double max_length = 0;
};

struct PatchConfig {
    int degree_initial = 0;
    int degree_iter_step = 0;
    int degree_max = 0;
    int patch_row_rso = 0;
    int patch_col_rso = 0;
    int row_overlap_step = 0;
    int col_overlap_step = 0;
    int k = 0;
};

struct ThresholdConfig {
    double exp_error_1 = 0;
    double mask_1 = 0;
    double exp_error_2 = 0;
    double mask_2 = 0;
    double u = 0;
    double v = 0;
    double d = 0;
};

struct ReconsConfig {
    int row_times = 0;
    int col_times = 0;
    int is_density_enhance = 0;
};

struct Patches {
    Eigen::MatrixXd point_cloud_r;
    Eigen::MatrixXd point_cloud_ori_r;
    Eigen::MatrixXd training_r;
    Eigen::MatrixXd training_ori_r;
    Eigen::MatrixXd testing_r;
    Eigen::MatrixXd azi;
    Eigen::MatrixXd azi_ori;
    Eigen::MatrixXd elev;
    Eigen::MatrixXd elev_ori;
    Eigen::VectorXd point_cloud_r_col;
    Eigen::VectorXd point_cloud_ori_r_col;
    Eigen::VectorXd training_r_col;
    Eigen::VectorXd training_ori_r_col;
    Eigen::VectorXd testing_r_col;
    Eigen::MatrixXd polar;
    Eigen::MatrixXd polar_idx;
    Eigen::VectorXd polar_idx_col;
    Eigen::MatrixXd polar_ori;
    Eigen::MatrixXd polar_ori_idx;
    Eigen::VectorXd polar_ori_idx_col;
    Eigen::MatrixXd ori_idx;
    Eigen::VectorXd ori_idx_col;
};

// void atan2(Eigen::Ref<Eigen::VectorXd> y, Eigen::Ref<Eigen::VectorXd> x,
// Eigen::Ref<Eigen::VectorXd> result) {
//
////#pragma omp parallel for num_threads(16) default(none) shared(result, x, y)
//    for (int i = 0; i < y.rows(); i++) {
//        result(i) = std::atan2(y(i), x(i));
//    }
//}
// declaration
template <typename T> Eigen::VectorX<T> range(T low, T high, T step, bool with_last = false);

// definition
Eigen::VectorXd atan2(Eigen::Ref<Eigen::VectorXd> y, Eigen::Ref<Eigen::VectorXd> x) {
    Eigen::VectorXd result = Eigen::VectorXd::Zero(y.rows());
    // #pragma omp parallel for num_threads(16) default(none) shared(result, x, y)
    for (int i = 0; i < y.rows(); i++) {
        if (y(i) != 0 || x(i) > 0) {
            result(i) = 2 * atan(y(i) / (sqrt(x(i) * x(i) + y(i) * y(i)) + x(i)));
        } else if (x(i) < 0 && y(i) == 0) {
            result(i) = M_PI;
        }
    }
    return result;
}

void atan2(Eigen::Ref<Eigen::VectorXd> y, Eigen::Ref<Eigen::VectorXd> x, Eigen::Ref<Eigen::VectorXd> result) {
    // #pragma omp parallel for num_threads(16) default(none) shared(result, x, y)
    for (int i = 0; i < y.rows(); i++) {
        if (y(i) != 0 || x(i) > 0) {
            result(i) = 2 * atan(y(i) / (sqrt(x(i) * x(i) + y(i) * y(i)) + x(i)));
        } else if (x(i) < 0 && y(i) == 0) {
            result(i) = M_PI;
        }
    }
}

template <typename T> Eigen::VectorX<T> atan2(const Eigen::VectorX<T> &y, const Eigen::VectorX<T> &x) {
    Eigen::VectorX<T> result = Eigen::VectorX<T>::Zero(y.rows());
    // #pragma omp parallel for num_threads(16) default(none) shared(result, x, y)
    for (int i = 0; i < y.rows(); i++) {
        if (y(i) != 0 || x(i) > 0) {
            result(i) = 2 * atan(y(i) / (sqrt(x(i) * x(i) + y(i) * y(i)) + x(i)));
        } else if (x(i) < 0 && y(i) == 0) {
            result(i) = M_PI;
        }
    }
    return result;
}

template <typename T> Eigen::MatrixX<T> cart2sph(const Eigen::MatrixX<T> &xyz) {
    assert(xyz.cols() == 3);
    Eigen::MatrixX<T> sph(xyz.rows(), 3);
    Eigen::VectorX<T> hypotxy = (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
    sph.col(2) = (hypotxy.array().pow(2) + xyz.col(2).array().pow(2)).array().sqrt();
    sph.col(1) = atan2<T>(xyz.col(2), hypotxy);
    sph.col(0) = atan2<T>(xyz.col(1), xyz.col(0));
    return sph;
}
// output: azi elev r
Matrix3dCol cart2sph(Eigen::Ref<Matrix3dCol> xyz) {
    Matrix3dCol sph(xyz.rows(), 3);
    Eigen::VectorXd xyz_col_x = Eigen::Map<Eigen::VectorXd>(xyz.data(), xyz.rows());
    Eigen::VectorXd xyz_col_y = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows(), xyz.rows());
    Eigen::VectorXd xyz_col_z = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2, xyz.rows());
    Eigen::VectorXd hypotxy = (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
    sph.col(2) = (hypotxy.array().pow(2) + xyz.col(2).array().pow(2)).array().sqrt();
    sph.col(1) = atan2(xyz_col_z, hypotxy);
    sph.col(0) = atan2(xyz_col_y, xyz_col_x);
    return sph;
}

// output: azi elev r
// Eigen::MatrixXd cart2sph(Eigen::Ref<Eigen::MatrixXd> xyz) {
//    Eigen::MatrixXd sph(xyz.rows(), 3);
//    Eigen::VectorXd xyz_col_x = Eigen::Map<Eigen::VectorXd>(xyz.data(), xyz.rows());
//    Eigen::VectorXd xyz_col_y = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows(), xyz.rows());
//    Eigen::VectorXd xyz_col_z = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2,
//    xyz.rows()); Eigen::VectorXd hypotxy = (xyz.col(0).array().pow(2) +
//    xyz.col(1).array().pow(2)).array().sqrt(); sph.col(2) = (hypotxy.array().pow(2) +
//    xyz.col(2).array().pow(2)).array().sqrt(); sph.col(1) = atan2(xyz_col_z, hypotxy); sph.col(0)
//    = atan2(xyz_col_y, xyz_col_x); return sph;
//}

// Use Eigen::Ref can use parts as the input without const
// output: azi elev r
void cart2sph(Eigen::Ref<Matrix3dCol> xyz, Eigen::Ref<Matrix3dCol> sph) {
    Eigen::VectorXd xyz_col_x = Eigen::Map<Eigen::VectorXd>(xyz.data(), xyz.rows());
    Eigen::VectorXd xyz_col_y = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows(), xyz.rows());
    Eigen::VectorXd xyz_col_z = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2, xyz.rows());
    Eigen::VectorXd hypotxy = (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
    sph.col(2) = (hypotxy.array().pow(2) + xyz.col(2).array().pow(2)).array().sqrt();
    atan2(xyz_col_z, hypotxy, sph.col(1));
    atan2(xyz_col_y, xyz_col_x, sph.col(0));
}

template <typename T = double>
void cart2sph(Eigen::Matrix<T, Eigen::Dynamic, 3> &xyz, Eigen::Matrix<T, Eigen::Dynamic, 3> sph) {
    Eigen::VectorXd xyz_col_x = Eigen::Map<Eigen::VectorXd>(xyz.data(), xyz.rows());
    Eigen::VectorXd xyz_col_y = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows(), xyz.rows());
    Eigen::VectorXd xyz_col_z = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2, xyz.rows());
    Eigen::VectorXd hypotxy = (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
    sph.col(2) = (hypotxy.array().pow(2) + xyz.col(2).array().pow(2)).array().sqrt();
    atan2(xyz_col_z, hypotxy, sph.col(1));
    atan2(xyz_col_y, xyz_col_x, sph.col(0));
}

// Use Eigen::Ref can use parts as the input without const
// output: azi inc r
void cart2sphTable(Eigen::Ref<Matrix3dCol> xyz, Eigen::Ref<Matrix3dCol> sph) {
    Eigen::VectorXd xyz_col_x = Eigen::Map<Eigen::VectorXd>(xyz.data(), xyz.rows());
    Eigen::VectorXd xyz_col_y = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows(), xyz.rows());
    Eigen::VectorXd xyz_col_z = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2, xyz.rows());
    Eigen::VectorXd hypotxy = (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
    sph.col(2) = (hypotxy.array().pow(2) + xyz.col(2).array().pow(2)).array().sqrt();
    atan2(xyz_col_z, hypotxy, sph.col(1));
    atan2(xyz_col_y, xyz_col_x, sph.col(0));
    sph.col(0) = sph.col(0).array() + M_PI;
    sph.col(1) = M_PI_2 - sph.col(1).array();
}

void sph2cart(Eigen::VectorXd r_col, Eigen::MatrixXd sph, Eigen::Ref<Matrix3dCol> xyz) {
    xyz.col(2) = r_col.array() * sin(sph.col(1).array()).array();
    Eigen::VectorXd rcoselev = r_col.array() * cos(sph.col(1).array()).array();
    xyz.col(0) = rcoselev.array() * cos(sph.col(0).array()).array();
    xyz.col(1) = rcoselev.array() * sin(sph.col(0).array()).array();
}

template <typename T = double>
void sph2cart(Eigen::Matrix<T, Eigen::Dynamic, 1> r_col, Eigen::Matrix<T, Eigen::Dynamic, 2> sph,
              Eigen::Matrix<T, Eigen::Dynamic, 3> xyz) {
    xyz.col(2) = r_col.array() * sin(sph.col(1).array()).array();
    Eigen::VectorXd rcoselev = r_col.array() * cos(sph.col(1).array()).array();
    xyz.col(0) = rcoselev.array() * cos(sph.col(0).array()).array();
    xyz.col(1) = rcoselev.array() * sin(sph.col(0).array()).array();
}

void sph2cart_table(Eigen::VectorXd r_col, Eigen::MatrixXd sph, Eigen::Ref<Matrix3dCol> xyz) {
    sph.col(0) = sph.col(0).array() - M_PI;
    sph.col(1) = M_PI_2 - sph.col(1).array();
    xyz.col(2) = r_col.array() * sin(sph.col(1).array()).array();
    Eigen::VectorXd rcoselev = r_col.array() * cos(sph.col(1).array()).array();
    xyz.col(0) = rcoselev.array() * cos(sph.col(0).array()).array();
    xyz.col(1) = rcoselev.array() * sin(sph.col(0).array()).array();
}

template <typename T = double> void assocLegendre(int l, Eigen::VectorX<T> X, Eigen::MatrixX<T> &P) {
    P.resize(l + 1, X.size());
    // #pragma omp parallel for num_threads(16) default(none) shared(l, X, P)
    for (int i = 0; i < X.size(); i++) {
        for (int m = 0; m <= l; m++) {
            P(m, i) = pow(-1, m) * std::assoc_legendre(l, m, X(i));
        }
    }
}

template <typename T = double> Eigen::VectorX<T> assocLegendre(int l, Eigen::VectorX<T> X) {
    Eigen::VectorX<T> P;
    P.resize(l + 1, X.size());
    // #pragma omp parallel for num_threads(16) default(none) shared(l, X, P)
    for (int i = 0; i < X.size(); i++) {
        for (int m = 0; m <= l; m++) {
            P(m, i) = pow(-1, m) * std::assoc_legendre(l, m, X(i));
        }
    }
    return P;
}

template <typename T = double> T factorial(int n) {
    assert(n >= 0);
    T result = 1;
    for (int i = 1; i <= n; ++i) {
        result *= i;
    }
    return result;
}

template <typename T = double> Eigen::VectorX<T> factorial(Eigen::VectorXi n) {
    Eigen::VectorX<T> f(n.size());
    T f_mid;
    for (int i = 0; i < n.size(); i++) {
        f_mid = 1;
        assert(n(i) >= 0);
        for (int j = 1; j <= n(i); j++) {
            f_mid = f_mid * j;
        }
        f(i) = f_mid;
    }
    return f;
}

template <typename T = double> Eigen::MatrixX<T> getSH(int N, const Eigen::MatrixX<T> &dirs) {
    Eigen::MatrixX<T> Y_N;
    // initialization
    Eigen::VectorXi m;
    Eigen::MatrixX<T> Lnm_real;
    Eigen::MatrixX<T> condon;
    Y_N.resize(int(pow((N + 1), 2)), dirs.rows());
    Y_N.setZero();
    int idx_Y = 0;
    Eigen::VectorX<T> norm_real;
    Eigen::MatrixX<T> Nnm_real;
    Eigen::MatrixX<T> Nnm_real_mid;
    Eigen::MatrixX<T> CosSin;
    Eigen::MatrixX<T> Ynm;
    // start calculation
    for (int n = 0; n <= N; n++) {
        m = Eigen::VectorXi::LinSpaced(n + 1, 0, n);
        assocLegendre<T>(n, cos((Eigen::Map<const Eigen::VectorX<T>>(dirs.data() + dirs.rows(), dirs.rows())).array()),
                         Lnm_real);
        if (n != 0) {
            Eigen::MatrixXi condon_mid(m.size() + m.size() - 1, 1);
            condon_mid << m(Eigen::seq(m.size() - 1, 2 - 1, -1)), m;
            condon = Eigen::MatrixX<T>::Ones(m.size() + m.size() - 1, 1).array() * (-1);
            condon =
                condon.array().pow(condon_mid.array().cast<T>()).matrix() * Eigen::MatrixX<T>::Ones(1, dirs.rows());
            Eigen::MatrixX<T> Lnm_real_mid(Lnm_real.rows() + Lnm_real.rows() - 1, Lnm_real.cols());
            Lnm_real_mid << Lnm_real(Eigen::seq(Lnm_real.rows() - 1, 2 - 1, -1), Eigen::all), Lnm_real;
            Lnm_real = condon.array() * Lnm_real_mid.array();
        }
        // normalisations
        norm_real =
            ((2 * n + 1) * factorial<T>(n - m.array()).array() / (4 * M_PIl * factorial<T>(n + m.array())).array())
                .array()
                .sqrt();
        // convert to matrix, for direct matrix multiplication with the rest
        Nnm_real = norm_real * Eigen::MatrixX<T>::Ones(1, dirs.rows());
        if (n != 0) {
            Nnm_real_mid.resize(Nnm_real.rows() + Nnm_real.rows() - 1, Nnm_real.cols());
            Nnm_real_mid << Nnm_real(Eigen::seq(Nnm_real.rows() - 1, 2 - 1, -1), Eigen::all), Nnm_real;
            Nnm_real = Nnm_real_mid;
        }
        CosSin = Eigen::MatrixX<T>::Zero(2 * n + 1, dirs.rows());
        // zero degree
        CosSin(n, Eigen::all) = Eigen::MatrixX<T>::Ones(1, dirs.rows());
        // positive and negative degrees
        if (n != 0) {
            CosSin(m(Eigen::seq(2 - 1, m.size() - 1)).array() + n, Eigen::all) =
                sqrt(2) *
                cos((m(Eigen::seq(2 - 1, m.size() - 1)).cast<T>() * (dirs(Eigen::all, 0).transpose())).array());
            CosSin(-m(Eigen::seq(m.size() - 1, 2 - 1, -1)).array() + n, Eigen::all) =
                sqrt(2) *
                sin((m(Eigen::seq(m.size() - 1, 2 - 1, -1)).cast<T>() * (dirs(Eigen::all, 0).transpose())).array());
        }
        Ynm = Nnm_real.array() * Lnm_real.array() * CosSin.array();
        Y_N(Eigen::seq(idx_Y, idx_Y + (2 * n)), Eigen::all) = Ynm;
        idx_Y = idx_Y + 2 * n + 1;
    }
    Y_N.transposeInPlace();
    return Y_N;
}

template <typename T = double>
void get_sph_gradient_table(const Eigen::MatrixX<T> &dirs, SH_TABLE_CONFIG<T> &SH_table_config) {
    SH_table_config.SH_G_theta_table.resize(dirs.rows(), int(pow(SH_table_config.max_SH_degree + 1, 2)));
    SH_table_config.SH_G_phi_table.resize(dirs.rows(), int(pow(SH_table_config.max_SH_degree + 1, 2)));
    int idx_Y = 0;
    // lambda function for N_m
    auto Nm_func = [](const T &phi_ele, const T &m_ele) {
        T result = 0;
        if (m_ele > 0) {
            result = sqrt(2) * cos(m_ele * phi_ele);
        } else if (m_ele == 0) {
            result = 1;
        } else {
            result = sqrt(2) * sin(-m_ele * phi_ele);
        }
        return result;
    };
    // gradient function for N_m
    auto G_Nm_function = [](const T &phi_ele, const T &m_ele) {
        T result = 0;
        if (m_ele > 0) {
            result = -m_ele * sqrt(2) * sin(m_ele * phi_ele);
        } else if (m_ele == 0) {
            result = 0;
        } else {
            result = -m_ele * sqrt(2) * cos(m_ele * phi_ele);
        }
        return result;
    };

    // start the for loop
    for (int l = 0; l <= SH_table_config.max_SH_degree; ++l) {
        // with respect to theta
        Eigen::MatrixX<T> G_Ylm_theta(dirs.rows(), 2 * l + 1);
        Eigen::MatrixX<T> G_Ylm_phi(dirs.rows(), 2 * l + 1);
        G_Ylm_theta.setZero();
        G_Ylm_phi.setZero();
        if (l > 0) {
            Eigen::VectorXi m_idx = Eigen::VectorXi::LinSpaced(2 * l + 1, 0, 2 * l);
            Eigen::VectorX<T> pos_m = Eigen::VectorX<T>::LinSpaced(l, 1, l);
            // for m = 0
            G_Ylm_theta(Eigen::all, l) =
                -sqrt((2 * l + 1) / (4 * M_PI)) *
                cos(dirs.col(1).array()).unaryExpr([&](const T &elem) { return std::assoc_legendre(l, 1, elem); });

            // abs(m)*cot(theta)
            Eigen::MatrixX<T> pos_m_matrix = Eigen::MatrixX<T>::Ones(dirs.rows(), pos_m.size());
            pos_m_matrix = pos_m_matrix.array().rowwise() * pos_m.transpose().array();
            Eigen::VectorX<T> cot_theta = cos(dirs.col(1).array()) / sin(dirs.col(1).array());
            Eigen::MatrixX<T> m_cot_theta = pos_m_matrix.array().colwise() * cot_theta.array();
            // abs(m)*Y_l_m
            // the order need to be reversed -3 -2 -1 0 1 2 3
            G_Ylm_theta(Eigen::all, Eigen::seq(0, l - 1)) =
                m_cot_theta(Eigen::all, Eigen::seq(Eigen::last, 0, -1)).array() *
                SH_table_config.SH_table(Eigen::all, Eigen::seq(idx_Y, idx_Y + l - 1)).array();
            G_Ylm_theta(Eigen::all, Eigen::seq(l + 1, l + l)) =
                m_cot_theta.array() *
                SH_table_config.SH_table(Eigen::all, Eigen::seq(idx_Y + l + 1, idx_Y + l + l)).array();
            // inilize m matrix
            // inilize m matrix
            Eigen::MatrixX<T> neg_m_matrix = pos_m_matrix.array() - l - 1;
            Eigen::MatrixX<T> m_matrix(dirs.rows(), 2 * pos_m.size());
            m_matrix << neg_m_matrix, pos_m_matrix;
            // normalization matrix
            Eigen::MatrixX<T> normalization_matrix(dirs.rows(), 2 * pos_m.size());
            Eigen::Map<Eigen::MatrixX<T>> neg_half_normali(normalization_matrix.data(), dirs.rows(), pos_m.size());
            Eigen::Map<Eigen::MatrixX<T>> pos_half_normali(normalization_matrix.data() + dirs.rows() * pos_m.size(),
                                                           dirs.rows(), pos_m.size());
            Eigen::MatrixX<T> P_lm(dirs.rows(), 2 * pos_m.size());
            Eigen::Map<Eigen::MatrixX<T>> neg_half_P_lm(P_lm.data(), dirs.rows(), pos_m.size());
            Eigen::Map<Eigen::MatrixX<T>> pos_half_P_lm(P_lm.data() + dirs.rows() * pos_m.size(), dirs.rows(),
                                                        pos_m.size());
            pos_half_P_lm =
                Eigen::MatrixX<T>::Ones(dirs.rows(), pos_m.size()).array().colwise() * cos(dirs.col(1).array());
            for (int i = 0; i < pos_m.size(); ++i) {
                pos_half_normali.col(i) =
                    Eigen::VectorX<T>::Ones(dirs.rows()) *
                    sqrt((2 * l + 1) / (4 * M_PI) * factorial<T>(l - pos_m(i)) / factorial<T>(l + pos_m(i)));
                pos_half_P_lm.col(i) = pos_half_P_lm.col(i).unaryExpr(
                    [&](const T &elem) { return std::assoc_legendre(l, pos_m(i), elem); });
            }
            neg_half_normali = pos_half_normali(Eigen::all, Eigen::seq(Eigen::last, 0, -1));
            neg_half_P_lm = pos_half_P_lm(Eigen::all, Eigen::seq(Eigen::last, 0, -1));
            Eigen::MatrixX<T> phi_matrix(dirs.rows(), 2 * pos_m.size());
            phi_matrix = Eigen::MatrixX<T>::Ones(dirs.rows(), 2 * pos_m.size()).array().colwise() * dirs.col(0).array();
            if (l > 1) {
                Eigen::Map<Eigen::MatrixX<T>> m_sub_matrix(m_matrix.data() + dirs.rows(), dirs.rows(),
                                                           2 * (pos_m.size() - 1));
                Eigen::MatrixX<T> Nm_sub_matrix =
                    phi_matrix(Eigen::all, Eigen::seq(1, Eigen::last - 1)).binaryExpr(m_sub_matrix, Nm_func);
                // multiply all elements
                std::vector<int> P_lm_sub_idx(2 * pos_m.size());
                std::iota(P_lm_sub_idx.begin(), P_lm_sub_idx.end(), 0);
                std::set<int> tmp_set(P_lm_sub_idx.begin(), P_lm_sub_idx.end());
                tmp_set.erase(l);
                tmp_set.erase(l - 1);
                P_lm_sub_idx.assign(tmp_set.begin(), tmp_set.end());
                Eigen::MatrixX<T> right_term =
                    normalization_matrix(Eigen::all, Eigen::seq(1, Eigen::last - 1)).array() * Nm_sub_matrix.array() *
                    P_lm(Eigen::all, P_lm_sub_idx).array();
                // update the G_Ylm_theta
                G_Ylm_theta(Eigen::all, Eigen::seq(1, l - 1)) =
                    (G_Ylm_theta(Eigen::all, Eigen::seq(1, l - 1)) -
                     right_term(Eigen::all, Eigen::seq(0, pos_m.size() - 2)))
                        .eval();
                G_Ylm_theta(Eigen::all, Eigen::seq(l + 1, l + l - 1)) =
                    (G_Ylm_theta(Eigen::all, Eigen::seq(l + 1, l + l - 1)) -
                     right_term(Eigen::all, Eigen::seq(pos_m.size() - 1, 2 * (pos_m.size() - 1) - 1)))
                        .eval();
            }
            // gradient with respect to phi
            Eigen::MatrixX<T> Nm_gradient_matrix = phi_matrix.binaryExpr(m_matrix, G_Nm_function);
            G_Ylm_phi(Eigen::all, Eigen::seq(0, l - 1)) =
                normalization_matrix(Eigen::all, Eigen::seq(0, l - 1)).array() *
                P_lm(Eigen::all, Eigen::seq(0, l - 1)).array() *
                Nm_gradient_matrix(Eigen::all, Eigen::seq(0, l - 1)).array();

            G_Ylm_phi(Eigen::all, Eigen::seq(l + 1, l + l)) =
                normalization_matrix(Eigen::all, Eigen::seq(l, l + l - 1)).array() *
                P_lm(Eigen::all, Eigen::seq(l, l + l - 1)).array() *
                Nm_gradient_matrix(Eigen::all, Eigen::seq(l, l + l - 1)).array();
        }
        SH_table_config.SH_G_theta_table(Eigen::all, Eigen::seq(idx_Y, idx_Y + 2 * l)) = G_Ylm_theta;
        SH_table_config.SH_G_phi_table(Eigen::all, Eigen::seq(idx_Y, idx_Y + 2 * l)) = G_Ylm_phi;
        // update the index
        idx_Y = idx_Y + 2 * l + 1;
    }
    //    FileReaderBase::write_txt_file("/home/user/project/ros/slam/src/CURL_SLAM_ros/tmp_data/G_theta.txt",
    //                                   Eigen::MatrixXd(SH_table_config.SH_G_theta.template cast<double>()));
    //    FileReaderBase::write_txt_file("/home/user/project/ros/slam/src/CURL_SLAM_ros/tmp_data/G_phi.txt",
    //                                   Eigen::MatrixXd(SH_table_config.SH_G_phi.template cast<double>()));
    //    std::cout << "here" << std::endl;
}

template <typename T = double>
Eigen::VectorX<T> leastSquaresSHT(int N, const Eigen::VectorX<T> &F, const Eigen::MatrixX<T> &dirs,
                                  const std::string &inv_method = "svd") {
    Eigen::VectorX<T> F_N;
    Eigen::MatrixX<T> Y_N;
    Y_N = getSH<T>(N, dirs);
    if (inv_method == "svd") {
        F_N = Y_N.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(F);
    } else if (inv_method == "qr") {
        F_N = Y_N.colPivHouseholderQr().solve(F);
    } else if (inv_method == "normal") {
        F_N = (Y_N.transpose() * Y_N).ldlt().solve(Y_N.transpose() * F);
    }
    return F_N;
}

template <typename T = double>
Eigen::VectorX<T> IRF_least_square(int max_degree, const Eigen::VectorX<T> &F, const Eigen::MatrixX<T> &dirs, int num_p,
                                   int num_g, const std::string &inv_method = "svd") {
    Eigen::MatrixX<T> Y_N = getSH<T>(max_degree, dirs);
    Eigen::VectorX<T> residual = F;
    Eigen::VectorX<T> sph_coeff = Eigen::VectorX<T>::Zero(pow(max_degree + 1, 2));
    for (int i = 0; i < num_p; ++i) {
        int d = 0;
        while (d <= max_degree) {
            int d_dash = d + num_g - 1;
            if (d_dash > max_degree) {
                d_dash = max_degree;
            }
            Eigen::VectorX<T> sph_coeff_tmp;
            if (inv_method == "svd") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                                    .solve(residual);
            } else if (inv_method == "qr") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .colPivHouseholderQr()
                                    .solve(residual);
            } else if (inv_method == "normal") {
                sph_coeff_tmp =
                    (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() *
                     Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)))
                        .ldlt()
                        .solve(Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() * residual);
            }
            residual -= (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) * sph_coeff_tmp);
            sph_coeff(Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) += sph_coeff_tmp;
            d = d_dash + 1;
        }
    }
    return sph_coeff;
}

template <typename T = double>
void IRF_least_square(const Eigen::VectorX<T> &F, const Eigen::MatrixX<T> &dirs, int num_p, int num_g,
                      Eigen::VectorX<T> &sph_coeff, const std::string &inv_method = "svd") {
    int max_degree = static_cast<int>(sqrt(static_cast<T>(sph_coeff.size())) - 1);
    Eigen::MatrixX<T> Y_N = getSH<T>(max_degree, dirs);
    Eigen::VectorX<T> residual = F;
    //    Eigen::VectorX<T> sph_coeff = Eigen::VectorX<T>::Zero(pow(max_degree + 1, 2));
    for (int i = 0; i < num_p; ++i) {
        int d = 0;
        while (d <= max_degree) {
            int d_dash = d + num_g - 1;
            if (d_dash > max_degree) {
                d_dash = max_degree;
            }
            Eigen::VectorX<T> sph_coeff_tmp;
            if (inv_method == "svd") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                                    .solve(residual);
            } else if (inv_method == "qr") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .colPivHouseholderQr()
                                    .solve(residual);
            } else if (inv_method == "normal") {
                sph_coeff_tmp =
                    (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() *
                     Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)))
                        .ldlt()
                        .solve(Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() * residual);
            }

            residual -= (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) * sph_coeff_tmp);
            sph_coeff(Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) += sph_coeff_tmp;
            d = d_dash + 1;
        }
    }
}

// no succeed check
template <typename T = double>
Eigen::VectorX<T> IRF_least_square_table(int max_degree, const Eigen::VectorX<T> &F, const Eigen::MatrixX<T> &dirs,
                                         int num_p, int num_g, const SH_TABLE_CONFIG<T> &SH_table_config,
                                         const std::string &inv_method = "svd") {
    assert(pow(max_degree + 1, 2) <= SH_table_config.SH_table.cols());
    T azi_low = SH_table_config.get_azi_low();
    T azi_high = SH_table_config.get_azi_high();
    T elev_low = SH_table_config.get_elev_low();
    T elev_high = SH_table_config.get_elev_high();
    // get the index of the nearest azimuth and elevation from the table
    Eigen::VectorXi azi_idx = ((dirs.col(0).array() - azi_low) / (azi_high - azi_low) * (SH_table_config.azi_rso - 1))
                                  .array()
                                  .round()
                                  .template cast<int>();
    Eigen::VectorXi elev_idx =
        ((dirs.col(1).array() - elev_low) / (elev_high - elev_low) * (SH_table_config.elev_rso - 1))
            .array()
            .round()
            .template cast<int>();
    Eigen::MatrixX<T> Y_N = SH_table_config.SH_table(azi_idx * SH_table_config.elev_rso + elev_idx,
                                                     Eigen::seq(0, pow(max_degree + 1, 2) - 1));
    Eigen::VectorX<T> residual = F;
    Eigen::VectorX<T> sph_coeff = Eigen::VectorX<T>::Zero(static_cast<long>(pow(max_degree + 1, 2)));
    for (int i = 0; i < num_p; ++i) {
        int d = 0;
        while (d <= max_degree) {
            int d_dash = d + num_g - 1;
            if (d_dash > max_degree) {
                d_dash = max_degree;
            }
            Eigen::VectorX<T> sph_coeff_tmp;
            if (inv_method == "svd") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                                    .solve(residual);
            } else if (inv_method == "qr") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .colPivHouseholderQr()
                                    .solve(residual);
            } else if (inv_method == "normal") {
                sph_coeff_tmp =
                    (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() *
                     Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)))
                        .ldlt()
                        .solve(Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() * residual);
            }
            residual -= (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) * sph_coeff_tmp);
            sph_coeff(Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) += sph_coeff_tmp;
            d = d_dash + 1;
        }
    }
    return sph_coeff;
}

// sph coeff quality
enum { GOOD, MEDIUM, BAD };
// have the sph coeff checking
template <typename T = double>
Eigen::VectorX<T> IRF_least_square_table(int max_degree, const Eigen::VectorX<T> &F, const Eigen::MatrixX<T> &dirs,
                                         int num_p, int num_g, const SH_TABLE_CONFIG<T> &SH_table_config,
                                         T medium_thres, T bad_thres, int &status,
                                         const std::string &inv_method = "svd") {
    assert(pow(max_degree + 1, 2) <= SH_table_config.SH_table.cols());
    T azi_low = SH_table_config.get_azi_low();
    T azi_high = SH_table_config.get_azi_high();
    T elev_low = SH_table_config.get_elev_low();
    T elev_high = SH_table_config.get_elev_high();
    // get the index of the nearest azimuth and elevation from the table
    Eigen::VectorXi azi_idx = ((dirs.col(0).array() - azi_low) / (azi_high - azi_low) * (SH_table_config.azi_rso - 1))
                                  .array()
                                  .round()
                                  .template cast<int>();
    Eigen::VectorXi elev_idx =
        ((dirs.col(1).array() - elev_low) / (elev_high - elev_low) * (SH_table_config.elev_rso - 1))
            .array()
            .round()
            .template cast<int>();
    Eigen::MatrixX<T> Y_N = SH_table_config.SH_table(azi_idx * SH_table_config.elev_rso + elev_idx,
                                                     Eigen::seq(0, pow(max_degree + 1, 2) - 1));
    Eigen::VectorX<T> residual = F;
    Eigen::VectorX<T> sph_coeff = Eigen::VectorX<T>::Zero(static_cast<long>(pow(max_degree + 1, 2)));
    for (int i = 0; i < num_p; ++i) {
        int d = 0;
        while (d <= max_degree) {
            int d_dash = d + num_g - 1;
            if (d_dash > max_degree) {
                d_dash = max_degree;
            }
            Eigen::VectorX<T> sph_coeff_tmp;
            if (inv_method == "svd") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                                    .solve(residual);
            } else if (inv_method == "qr") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .colPivHouseholderQr()
                                    .solve(residual);
            } else if (inv_method == "normal") {
                sph_coeff_tmp =
                    (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() *
                     Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)))
                        .ldlt()
                        .solve(Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() * residual);
            }
            residual -= (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) * sph_coeff_tmp);
            sph_coeff(Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) += sph_coeff_tmp;
            d = d_dash + 1;
        }
    }
    int medium_num = (residual.cwiseAbs().array() > medium_thres).count();
    if (medium_num == 0) {
        status = GOOD;
        return sph_coeff;
    }
    int bad_num = (residual.cwiseAbs().array() > bad_thres).count();
    if (bad_num > 3) {
        status = BAD;
        return sph_coeff;
    } else {
        status = MEDIUM;
        return sph_coeff;
    }
    //    FileReaderBase::write_txt_file("/home/user/project/ros/slam/src/CURL_SLAM_ros/tmp_data/residual_" +
    //                                       std::to_string(file_idx) + ".txt",
    //                                   Eigen::MatrixXd(residual.template cast<double>()));
    //    return sph_coeff;
}

template <typename T = double>
void IRF_least_square_table(const Eigen::VectorX<T> &F, const Eigen::MatrixX<T> &dirs,
                            const SH_TABLE_CONFIG<T> &SH_table_config, int num_p, int num_g,
                            Eigen::VectorX<T> &sph_coeff, const std::string &inv_method = "svd") {
    assert(sph_coeff.size() <= SH_table_config.SH_table.cols());
    int max_degree = static_cast<int>(sqrt(static_cast<T>(sph_coeff.size())) - 1);
    T azi_low = SH_table_config.get_azi_low();
    T azi_high = SH_table_config.get_azi_high();
    T elev_low = SH_table_config.get_elev_low();
    T elev_high = SH_table_config.get_elev_high();
    // get the index of the nearest azimuth and elevation from the table
    Eigen::VectorXi azi_idx = ((dirs.col(0).array() - azi_low) / (azi_high - azi_low) * (SH_table_config.azi_rso - 1))
                                  .array()
                                  .round()
                                  .template cast<int>();
    Eigen::VectorXi elev_idx =
        ((dirs.col(1).array() - elev_low) / (elev_high - elev_low) * (SH_table_config.elev_rso - 1))
            .array()
            .round()
            .template cast<int>();
    Eigen::MatrixX<T> Y_N = SH_table_config.SH_table(azi_idx * SH_table_config.elev_rso + elev_idx,
                                                     Eigen::seq(0, pow(max_degree + 1, 2) - 1));
    Eigen::VectorX<T> residual = F;
    //    Eigen::VectorX<T> sph_coeff = Eigen::VectorX<T>::Zero(static_cast<long>(pow(max_degree +
    //    1, 2)));
    for (int i = 0; i < num_p; ++i) {
        int d = 0;
        while (d <= max_degree) {
            int d_dash = d + num_g - 1;
            if (d_dash > max_degree) {
                d_dash = max_degree;
            }
            Eigen::VectorX<T> sph_coeff_tmp;
            if (inv_method == "svd") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                                    .solve(residual);
            } else if (inv_method == "qr") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .colPivHouseholderQr()
                                    .solve(residual);
            } else if (inv_method == "normal") {
                sph_coeff_tmp =
                    (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() *
                     Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)))
                        .ldlt()
                        .solve(Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() * residual);
            }
            residual -= (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) * sph_coeff_tmp);
            sph_coeff(Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) += sph_coeff_tmp;
            d = d_dash + 1;
        }
    }
}

template <typename T = double>
void IRF_least_square_table(const Eigen::VectorX<T> &F, Eigen::VectorXi &SH_table_idx,
                            const SH_TABLE_CONFIG<T> &SH_table_config, int num_p, int num_g,
                            Eigen::VectorX<T> &sph_coeff, const std::string &inv_method = "svd") {
    assert(sph_coeff.size() <= SH_table_config.SH_table.cols());
    int max_degree = static_cast<int>(sqrt(static_cast<T>(sph_coeff.size())) - 1);
    Eigen::MatrixX<T> Y_N = SH_table_config.SH_table(SH_table_idx, Eigen::seq(0, pow(max_degree + 1, 2) - 1));
    Eigen::VectorX<T> residual = F;
    //    Eigen::VectorX<T> sph_coeff = Eigen::VectorX<T>::Zero(static_cast<long>(pow(max_degree +
    //    1, 2)));
    for (int i = 0; i < num_p; ++i) {
        int d = 0;
        while (d <= max_degree) {
            int d_dash = d + num_g - 1;
            if (d_dash > max_degree) {
                d_dash = max_degree;
            }
            Eigen::VectorX<T> sph_coeff_tmp;
            if (inv_method == "svd") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                                    .solve(residual);
            } else if (inv_method == "qr") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .colPivHouseholderQr()
                                    .solve(residual);
            } else if (inv_method == "normal") {
                sph_coeff_tmp =
                    (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() *
                     Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)))
                        .ldlt()
                        .solve(Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() * residual);
            }
            residual -= (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) * sph_coeff_tmp);
            sph_coeff(Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) += sph_coeff_tmp;
            d = d_dash + 1;
        }
    }
}

template <typename T = double>
Eigen::VectorX<T> IRF_least_square_table_update_sph(const Eigen::VectorX<T> &F, Eigen::VectorXi &SH_table_idx,
                                                    const SH_TABLE_CONFIG<T> &SH_table_config, int num_p, int num_g,
                                                    const Eigen::VectorX<T> &sph_coeff,
                                                    const std::string &inv_method = "svd") {
    assert(sph_coeff.size() <= SH_table_config.SH_table.cols());
    int max_degree = static_cast<int>(sqrt(static_cast<T>(sph_coeff.size())) - 1);
    Eigen::MatrixX<T> Y_N = SH_table_config.SH_table(SH_table_idx, Eigen::seq(0, pow(max_degree + 1, 2) - 1));
    Eigen::VectorX<T> residual = F;
    //    Eigen::VectorXd sph_coeff = Eigen::VectorXd::Zero(static_cast<long>(pow(max_degree + 1,
    //    2)));
    Eigen::VectorX<T> sph_coeff_update = sph_coeff;
    for (int i = 0; i < num_p; ++i) {
        int d = 0;
        while (d <= max_degree) {
            int d_dash = d + num_g - 1;
            if (d_dash > max_degree) {
                d_dash = max_degree;
            }
            Eigen::VectorX<T> sph_coeff_tmp;
            if (inv_method == "svd") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                                    .solve(residual);
            } else if (inv_method == "qr") {
                sph_coeff_tmp = Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1))
                                    .colPivHouseholderQr()
                                    .solve(residual);
            } else if (inv_method == "normal") {
                sph_coeff_tmp =
                    (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() *
                     Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)))
                        .ldlt()
                        .solve(Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)).transpose() * residual);
            }
            residual -= (Y_N(Eigen::all, Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) * sph_coeff_tmp);
            sph_coeff_update(Eigen::seq(pow(d, 2), pow((d_dash + 1), 2) - 1)) += sph_coeff_tmp;
            d = d_dash + 1;
        }
    }
    return sph_coeff_update;
}

template <typename T = double>
Eigen::VectorX<T> invLeastSquaresSHT(const Eigen::VectorX<T> &coefficients, const Eigen::MatrixX<T> &dirs,
                                     int max_degree = std::numeric_limits<int>::max()) {
    Eigen::VectorX<T> F;
    if (max_degree == std::numeric_limits<int>::max()) {
        max_degree = int(sqrt(int(coefficients.size()))) - 1;
    }
    F = getSH<T>(max_degree, dirs) * coefficients(Eigen::seq(0, pow(max_degree + 1, 2) - 1));
    return F;
}

template <typename T = double> void getSH_table(SH_TABLE_CONFIG<T> &SH_table_config) {
    T azi_low = SH_table_config.get_azi_low();
    T azi_high = SH_table_config.get_azi_high();
    T elev_low = SH_table_config.get_elev_low();
    T elev_high = SH_table_config.get_elev_high();
    Eigen::VectorX<T> azi_tmp = Eigen::VectorX<T>::LinSpaced(SH_table_config.azi_rso, azi_low, azi_high);
    Eigen::VectorX<T> elev_tmp = Eigen::VectorX<T>::LinSpaced(SH_table_config.elev_rso, elev_low, elev_high);
    Eigen::MatrixX<T> dirs(SH_table_config.azi_rso * SH_table_config.elev_rso, 2);
    for (int i = 0; i < azi_tmp.size(); ++i) {
        dirs(Eigen::seq(i * elev_tmp.size(), (i + 1) * elev_tmp.size() - 1), 0) =
            Eigen::VectorX<T>::Ones(elev_tmp.size()) * azi_tmp(i);
        dirs(Eigen::seq(i * elev_tmp.size(), (i + 1) * elev_tmp.size() - 1), 1) = elev_tmp;
    }
    SH_table_config.SH_table = getSH<T>(SH_table_config.max_SH_degree, dirs);
    get_sph_gradient_table<T>(dirs, SH_table_config);
    std::cout << "Finish initialization" << std::endl;
}

template <typename T = double> void get_update_idx(SH_TABLE_CONFIG<T> &SH_table_config) {
    // this decide the azi resolution used for updating sph_coeff
    double azi_step =
        (SH_table_config.azi_rso - 1) / ((SH_table_config.max_SH_degree + 1) * 2 - 1) / SH_table_config.sampling_factor;
    azi_step = round(azi_step);
    // this decide the elev resolution used for updating sph_coeff
    double elev_step = (SH_table_config.elev_rso - 1) / ((SH_table_config.max_SH_degree + 1) * 2 - 1) /
                       SH_table_config.sampling_factor;
    elev_step = round(elev_step);
    // initialize the update_idx
    Eigen::VectorXd std_azi_idx = curl::range<double>(0, SH_table_config.azi_rso - 1, azi_step);
    Eigen::VectorXd std_elev_idx = curl::range<double>(0, SH_table_config.elev_rso - 1, elev_step);
    SH_table_config.update_azi_rso = std_azi_idx.size();
    SH_table_config.update_elev_rso = std_elev_idx.size();
    SH_table_config.update_idx.resize(std_azi_idx.size() * std_elev_idx.size());
    for (int i = 0; i < std_azi_idx.size(); ++i) {
        SH_table_config.update_idx(Eigen::seq(i * std_elev_idx.size(), (i + 1) * std_elev_idx.size() - 1)) =
            (std_azi_idx(i) * SH_table_config.elev_rso + std_elev_idx.array()).template cast<int>();
    }
}

template <typename T = double>
Eigen::VectorX<T> invLeastSquaresSHT_table(const Eigen::VectorX<T> &coefficients, const Eigen::MatrixX<T> &dirs,
                                           const SH_TABLE_CONFIG<T> &SH_table_config,
                                           int max_degree = std::numeric_limits<int>::max()) {
    assert(coefficients.size() <= SH_table_config.SH_table.cols());
    if (max_degree == std::numeric_limits<int>::max()) {
        max_degree = static_cast<int>(sqrt(static_cast<T>(coefficients.size())) - 1);
    }
    assert(pow(max_degree + 1, 2) <= coefficients.size());
    T azi_low = SH_table_config.get_azi_low();
    T azi_high = SH_table_config.get_azi_high();
    T elev_low = SH_table_config.get_elev_low();
    T elev_high = SH_table_config.get_elev_high();
    // get the index of the nearest azimuth and elevation from the table
    Eigen::VectorXi azi_idx = ((dirs.col(0).array() - azi_low) / (azi_high - azi_low) * (SH_table_config.azi_rso - 1))
                                  .array()
                                  .round()
                                  .template cast<int>();
    Eigen::VectorXi elev_idx =
        ((dirs.col(1).array() - elev_low) / (elev_high - elev_low) * (SH_table_config.elev_rso - 1))
            .array()
            .round()
            .template cast<int>();
    Eigen::MatrixX<T> new_SH_table = SH_table_config.SH_table(azi_idx * SH_table_config.elev_rso + elev_idx,
                                                              Eigen::seq(0, pow(max_degree + 1, 2) - 1));
    Eigen::VectorX<T> F = new_SH_table * coefficients(Eigen::seq(0, pow(max_degree + 1, 2) - 1));
    return F;
}

template <typename Derived> Derived load_Matrix(const std::string &read_dir) {
    Derived matrix;
    std::ifstream infile;
    std::string STRING, ITEM;
    std::vector<std::vector<double>> vecMatrix;
    std::vector<double> vec;
    infile.open(read_dir);
    while (!infile.eof()) // To get you all the lines.
    {
        std::getline(infile, STRING); // Saves the line in STRING.
        if (STRING.empty()) {
            break;
        }
        std::stringstream ss(STRING);
        vec.clear();
        while (ss >> ITEM) {
            vec.push_back(std::stod(ITEM));
        }
        vecMatrix.push_back(vec);
    }
    infile.close();

    matrix.resize(int(vecMatrix.size()), int(vecMatrix[0].size()));
    for (int i = 0; i < int(vecMatrix.size()); i++) {
        for (int j = 0; j < int(vecMatrix[0].size()); j++) {
            matrix(i, j) = vecMatrix[i][j];
        }
    }
    return matrix;
}

void loadCoefficients(const std::string &read_dir, std::vector<Eigen::VectorXd> &coefficients, int &row, int &col) {
    std::vector<std::vector<double>> vecMatrix;
    //    std::vector<double> vec;
    std::ifstream infile;
    std::string STRING, ITEM;
    infile.open(read_dir);
    std::getline(infile, STRING); // Saves the line in STRING.
    std::stringstream ss(STRING);
    ss >> ITEM;
    row = std::stod(ITEM);
    ss >> ITEM;
    col = std::stod(ITEM);
    vecMatrix.resize(row * col);
    for (int i = 0; i < row * col; ++i) {
        std::getline(infile, STRING); // Saves the line in STRING.
        if (!STRING.empty()) {
            std::stringstream ss(STRING);
            while (ss >> ITEM) {
                vecMatrix[i].push_back(std::stod(ITEM));
            }
        }
    }
    infile.close();
    coefficients.resize(row * col);
    for (int i = 0; i < row * col; ++i) {
        if (!vecMatrix[i].empty()) {
            coefficients[i].resize(vecMatrix[i].size());
            for (int j = 0; j < vecMatrix[i].size(); ++j) {
                coefficients[i](j) = vecMatrix[i][j];
            }
        }
    }
}

template <typename Derived, typename T>
T P(int i, int l, int a, int b, const Eigen::MatrixBase<Derived> &R_1, const Eigen::MatrixBase<Derived> &R_lm1) {
    double ri1 = R_1(i + 1, 2);
    double rim1 = R_1(i + 1, 0);
    double ri0 = R_1(i + 1, 1);
    if (b == -l) {
        return (ri1 * R_lm1(a + l - 1, 0) + rim1 * R_lm1(a + l - 1, 2 * l - 2));
    } else {
        if (b == l)
            return (ri1 * R_lm1(a + l - 1, 2 * l - 2) - rim1 * R_lm1(a + l - 1, 0));
        else
            return (ri0 * R_lm1(a + l - 1, b + l - 1));
    }
}

template <typename Derived, typename T>
T U(int l, int m, int n, const Eigen::MatrixBase<Derived> &R_1, const Eigen::MatrixBase<Derived> &R_lm1) {
    return P<Derived, T>(0, l, m, n, R_1, R_lm1);
}

template <typename Derived, typename T>
T V(int l, int m, int n, const Eigen::MatrixBase<Derived> &R_1, const Eigen::MatrixBase<Derived> &R_lm1) {
    if (m == 0) {
        T p0 = P<Derived, T>(1, l, 1, n, R_1, R_lm1);
        T p1 = P<Derived, T>(-1, l, -1, n, R_1, R_lm1);
        return (p0 + p1);
    } else {
        if (m > 0) {
            bool d = (m == 1);
            T p0 = P<Derived, T>(1, l, m - 1, n, R_1, R_lm1);
            T p1 = P<Derived, T>(-1, l, -m + 1, n, R_1, R_lm1);
            return (p0 * sqrt(1 + double(d)) - p1 * (1 - double(d)));
        } else {
            bool d = (m == -1);
            T p0 = P<Derived, T>(1, l, m + 1, n, R_1, R_lm1);
            T p1 = P<Derived, T>(-1, l, -m - 1, n, R_1, R_lm1);
            return (p0 * (1 - double(d)) + p1 * sqrt(1 + double(d)));
        }
    }
}

template <typename Derived, typename T>
T Wf(int l, int m, int n, const Eigen::MatrixBase<Derived> &R_1, const Eigen::MatrixBase<Derived> &R_lm1) {
    if (m == 0) {
        std::cerr << "should not be called" << std::endl;
        return -1;
    } else {
        if (m > 0) {
            T p0 = P<Derived, T>(1, l, m + 1, n, R_1, R_lm1);
            T p1 = P<Derived, T>(-1, l, -m - 1, n, R_1, R_lm1);
            return (p0 + p1);
        } else {
            T p0 = P<Derived, T>(1, l, m - 1, n, R_1, R_lm1);
            T p1 = P<Derived, T>(-1, l, -m + 1, n, R_1, R_lm1);
            return (p0 - p1);
        }
    }
}

template <typename Derived, typename T> Derived getSHrotMtx(const Eigen::MatrixBase<Derived> &R_xyz, int degree) {
    Derived R_sh = Derived::Zero((degree + 1) * (degree + 1), (degree + 1) * (degree + 1));
    R_sh(0, 0) = 1;
    // the first band (l=1) is directly related to the rotation matrix
    Derived R_1 = Derived::Zero(3, 3);
    R_1(0, 0) = R_xyz(1, 1);
    R_1(0, 1) = R_xyz(1, 2);
    R_1(0, 2) = R_xyz(1, 0);
    R_1(1, 0) = R_xyz(2, 1);
    R_1(1, 1) = R_xyz(2, 2);
    R_1(1, 2) = R_xyz(2, 0);
    R_1(2, 0) = R_xyz(0, 1);
    R_1(2, 1) = R_xyz(0, 2);
    R_1(2, 2) = R_xyz(0, 0);

    R_sh(Eigen::seq(1, 3), Eigen::seq(1, 3)) = R_1;
    Derived R_lm1 = R_1;

    // compute rotation matrix of each subsequent band recursively
    int band_idx = 4;
    double denom = 0;
    bool d;
    double u;
    double v;
    double w;
    for (int l = 2; l <= degree; ++l) {
        Derived R_l = Derived::Zero(2 * l + 1, 2 * l + 1);
        for (int m = -l; m <= l; ++m) {
            for (int n = -l; n <= l; ++n) {
                // compute u,v,w terms of Eq.8.1 (Table I)
                d = (m == 0); // the delta function d_m0
                if (abs(n) == l) {
                    denom = (2 * l) * (2 * l - 1);
                } else {
                    denom = (l * l - n * n);
                }
                u = sqrt((l * l - m * m) / denom);
                v = sqrt((1 + double(d)) * (l + abs(m) - 1) * (l + abs(m)) / denom) * (1 - 2 * double(d)) * 0.5;
                w = sqrt((l - abs(m) - 1) * (l - abs(m)) / denom) * (1 - double(d)) * (-0.5);

                // computes Eq.8.1
                if (u != 0)
                    u = u * U<Derived, T>(l, m, n, R_1, R_lm1);
                if (v != 0) {
                    v = v * V<Derived, T>(l, m, n, R_1, R_lm1);
                }
                if (w != 0) {
                    w = w * Wf<Derived, T>(l, m, n, R_1, R_lm1);
                }
                R_l(m + l, n + l) = u + v + w;
            }
        }
        R_sh(Eigen::seq(band_idx, band_idx + 2 * l), Eigen::seq(band_idx, band_idx + 2 * l)) = R_l;
        R_lm1 = R_l;
        band_idx = band_idx + 2 * l + 1;
    }
    return R_sh;
}

template <typename Derived> Derived getInvSHrotMtx(const Eigen::MatrixBase<Derived> &R_sh) {
    Derived R_1 = R_sh(Eigen::seq(1, 3), Eigen::seq(1, 3));
    Derived R_xyz(3, 3);
    R_xyz(1, 1) = R_1(0, 0);
    R_xyz(1, 2) = R_1(0, 1);
    R_xyz(1, 0) = R_1(0, 2);
    R_xyz(2, 1) = R_1(1, 0);
    R_xyz(2, 2) = R_1(1, 1);
    R_xyz(2, 0) = R_1(1, 2);
    R_xyz(0, 1) = R_1(2, 0);
    R_xyz(0, 2) = R_1(2, 1);
    R_xyz(0, 0) = R_1(2, 2);
    return R_xyz;
}

// vector sorting
template <typename T> std::vector<size_t> sort_indexes(const std::vector<T> &v) {

    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    stable_sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

    return idx;
}

void convert_to_point_cloud(const Eigen::MatrixXd &source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    cloud->points.resize(source_cloud.rows());
    for (int i = 0; i < source_cloud.rows(); i++) {
        cloud->points[i].getVector3fMap() =
            Eigen::Vector3d(source_cloud(i, 0), source_cloud(i, 1), source_cloud(i, 2)).cast<float>();
    }
}

void convert_to_point_cloud_2d(const Eigen::MatrixXd &source_cloud, pcl::PointCloud<pcl::PointXY>::Ptr &cloud) {
    cloud->points.resize(source_cloud.rows());
    for (int i = 0; i < source_cloud.rows(); i++) {
        cloud->points[i].getVector2fMap() = Eigen::Vector2d(source_cloud(i, 0), source_cloud(i, 1)).cast<float>();
    }
}

// Eigen::VectorXd range(double low, double high, double step, bool with_last = false) {
//     int N = static_cast<int>(std::floor((high - low) / step) + 1);
//     Eigen::VectorXd vec = Eigen::VectorXd::Zero(N);
//     std::iota(vec.data(), vec.data() + N, 0);
//     vec.array() *= step;
//     vec.array() += low;
//     if (with_last && (vec(N - 1) != high)) {
//         vec.conservativeResize(N + 1);
//         vec(N) = high;
//     }
//     return vec;
// }

template <typename T> Eigen::VectorX<T> range(T low, T high, T step, bool with_last) {
    int N = static_cast<int>(std::floor((high - low) / step) + 1);
    Eigen::VectorX<T> vec = Eigen::VectorX<T>::Zero(N);
    std::iota(vec.data(), vec.data() + N, 0);
    vec.array() *= step;
    vec.array() += low;
    if (with_last && (vec(N - 1) != high)) {
        vec.conservativeResize(N + 1);
        vec(N) = high;
    }
    return vec;
}

void linear_loss_function(Eigen::VectorXd &vec, double delta, double huber_para) {
    for (int i = 0; i < vec.size(); ++i) {
        if (abs(vec(i)) >= delta) {
            vec(i) = vec(i) / huber_para;
        }
    }
}

void trapeziform(Eigen::VectorXd &vec, double lower_band, double higher_band) {
    for (int i = 0; i < vec.size(); ++i) {
        if ((abs(vec(i)) >= lower_band) && (abs(vec(i)) <= higher_band)) {
            vec(i) = vec(i) * (higher_band - abs(vec(i))) / (higher_band - lower_band);
        } else if (abs(vec(i)) > higher_band) {
            vec(i) = 0;
        }
    }
}

// ToDo: Haven't tested
template <typename T>
void mesh_grid(T x_low, T x_high, int x_size, T y_low, T y_high, int y_size, Eigen::MatrixX<T> &grid) {
    Eigen::VectorX<T> x = Eigen::VectorX<T>::LinSpaced(x_size, x_low, x_high);
    Eigen::VectorX<T> y = Eigen::VectorX<T>::LinSpaced(y_size, y_low, y_high);
    grid.resize(x_size * y_size, 2);
    for (int i = 0; i < x_size; ++i) {
        grid(Eigen::seq(i * y_size, (i + 1) * y_size - 1), 0) = Eigen::VectorX<T>::Ones(y_size).array() * x(i);
        grid(Eigen::seq(i * y_size, (i + 1) * y_size - 1), 1) = y;
    }
}

// ToDo: Haven't tested
template <typename T>
void mesh_grid(T x_low, T x_high, T x_step, T y_low, T y_high, T y_step, bool with_last, Eigen::MatrixX<T> &grid) {
    Eigen::VectorX<T> x = range<T>(x_low, x_high, x_step, with_last);
    Eigen::VectorX<T> y = range<T>(y_low, y_high, y_step, with_last);
    grid.resize(x.size() * y.size(), 2);
    for (int i = 0; i < x.size(); ++i) {
        grid(Eigen::seq(i * y.size(), (i + 1) * y.size()), 0) = Eigen::VectorX<T>::Ones(y.size()).array() * x(i);
        grid(Eigen::seq(i * y.size(), (i + 1) * y.size()), 1) = y;
    }
}

template <typename T> T IQR_thres_with_bdy(std::vector<T> &vec, T scale_factor, int minimum_size) {
    std::sort(vec.begin(), vec.end());
    int n = 0;
    if ((vec.size() % 2) == 0) {
        n = static_cast<int>(vec.size() / 2);
    } else {
        n = static_cast<int>((vec.size() - 1) / 2);
    }
    T Q1, Q3;
    if ((n % 2) != 0) {
        Q1 = vec[n / 2];
        Q3 = vec[(vec.size() * 2 - n) / 2];
    } else {
        Q1 = (vec[n / 2 - 1] + vec[n / 2]) / 2;
        Q3 = (vec[(vec.size() - n + vec.size() - 1) / 2] + vec[(vec.size() - n + vec.size() - 1) / 2 + 1]) / 2;
    }
    T IQR = Q3 - Q1;
    T thres = Q3 + scale_factor * IQR;
    T high_value = 0;
    if (vec.size() > minimum_size) {
        high_value = *(vec.end() - 1 - minimum_size);
    } else {
        high_value = *(vec.end() - 1);
    }
    while (thres > high_value) {
        scale_factor -= 0.1;
        thres = Q3 + scale_factor * IQR;
    }
    return thres;
}

template <typename T> T IQR_thres(std::vector<T> &vec, T scale_factor) {
    std::sort(vec.begin(), vec.end());
    int n = 0;
    if ((vec.size() % 2) == 0) {
        n = static_cast<int>(vec.size() / 2);
    } else {
        n = static_cast<int>((vec.size() - 1) / 2);
    }
    T Q1, Q3;
    if ((n % 2) != 0) {
        Q1 = vec[n / 2];
        Q3 = vec[(vec.size() * 2 - n) / 2];
    } else {
        Q1 = (vec[n / 2 - 1] + vec[n / 2]) / 2;
        Q3 = (vec[(vec.size() - n + vec.size() - 1) / 2] + vec[(vec.size() - n + vec.size() - 1) / 2 + 1]) / 2;
    }
    T IQR = Q3 - Q1;
    return (Q3 + scale_factor * IQR);
}

template <typename T> std::pair<T, T> IQR_thres_pair(std::vector<T> &vec, T scale_factor) {
    std::sort(vec.begin(), vec.end());
    int n = 0;
    if ((vec.size() % 2) == 0) {
        n = static_cast<int>(vec.size() / 2);
    } else {
        n = static_cast<int>((vec.size() - 1) / 2);
    }
    T Q1, Q3;
    if ((n % 2) != 0) {
        Q1 = vec[n / 2];
        Q3 = vec[(vec.size() * 2 - n) / 2];
    } else {
        Q1 = (vec[n / 2 - 1] + vec[n / 2]) / 2;
        Q3 = (vec[(vec.size() - n + vec.size() - 1) / 2] + vec[(vec.size() - n + vec.size() - 1) / 2 + 1]) / 2;
    }
    T IQR = Q3 - Q1;
    return std::make_pair(Q1 - scale_factor * IQR, Q3 + scale_factor * IQR);
}

// template <typename T> std::pair<T, T> IQR_thres(std::vector<T> &vec, T scale_factor = 1.5) {
//     std::sort(vec.begin(), vec.end());
//     int n = 0;
//     if ((vec.size() % 2) == 0) {
//         n = static_cast<int>(vec.size() / 2);
//     } else {
//         n = static_cast<int>((vec.size() - 1) / 2);
//     }
//     T Q1, Q3;
//     if ((n % 2) != 0) {
//         Q1 = vec[n / 2];
//         Q3 = vec[(vec.size() * 2 - n) / 2];
//     } else {
//         Q1 = (vec[n / 2 - 1] + vec[n / 2]) / 2;
//         Q3 = (vec[(vec.size() - n + vec.size() - 1) / 2] + vec[(vec.size() - n + vec.size() - 1) / 2 + 1]) / 2;
//     }
//     T IQR = Q3 - Q1;
//
//     return std::make_pair<T, T>(Q1 - scale_factor * Q3, Q1 + scale_factor * Q3);
// }

template <typename T>
Eigen::MatrixX<T> eigen_central_gradient(const Eigen::MatrixX<T> &img, const std::string &direction) {
    const T T_INVALID = std::numeric_limits<T>::max();
    Eigen::MatrixX<T> delta_img = Eigen::MatrixX<T>::Ones(img.rows(), img.cols()) * T_INVALID;
    if (direction == "x") {
        for (int y = 0; y < img.rows(); ++y) {
            for (int x = 0; x < img.cols(); ++x) {
                if (img(y, x) != T_INVALID) {
                    int prev = x - 1;
                    int next = x + 1;
                    if ((prev < 0) || (next > (img.cols() - 1))) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    if (img(y, prev) == T_INVALID) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    if (img(y, next) == T_INVALID) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    delta_img(y, x) = (img(y, next) - img(y, prev)) * 0.5;
                } else {
                    delta_img(y, x) = T_INVALID;
                }
            }
        }
    } else if (direction == "y") {
        for (int y = 0; y < img.rows(); ++y) {
            for (int x = 0; x < img.cols(); ++x) {
                if (img(y, x) != T_INVALID) {
                    int prev = y + 1; // origion of the coordinate is at the left
                                      // bottom corner rather than left top
                    int next = y - 1;
                    if ((prev > (img.rows() - 1)) || (next < 0)) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    if (img(prev, x) == T_INVALID) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    if (img(next, x) == T_INVALID) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    delta_img(y, x) = (img(next, x) - img(prev, x)) * 0.5;
                } else {
                    delta_img(y, x) = T_INVALID;
                }
            }
        }
    } else {
        std::cerr << "Please input correction direction. Either x or y" << std::endl;
    }
    return delta_img;
}

template <typename T>
Eigen::MatrixX<T> eigen_central_gradient(const Eigen::MatrixX<T> &img, const std::string &direction, const T &scale) {
    const T T_INVALID = std::numeric_limits<T>::max();
    Eigen::MatrixX<T> delta_img = Eigen::MatrixX<T>::Ones(img.rows(), img.cols()) * T_INVALID;
    if (direction == "x") {
        for (int y = 0; y < img.rows(); ++y) {
            for (int x = 0; x < img.cols(); ++x) {
                if (img(y, x) != T_INVALID) {
                    int prev = x - 1;
                    int next = x + 1;
                    if ((prev < 0) || (next > (img.cols() - 1))) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    if (img(y, prev) == T_INVALID) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    if (img(y, next) == T_INVALID) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    delta_img(y, x) = (img(y, next) - img(y, prev)) * 0.5 * (1 / scale);
                } else {
                    delta_img(y, x) = T_INVALID;
                }
            }
        }
    } else if (direction == "y") {
        for (int y = 0; y < img.rows(); ++y) {
            for (int x = 0; x < img.cols(); ++x) {
                if (img(y, x) != T_INVALID) {
                    int prev = y + 1; // origion of the coordinate is at the left
                                      // bottom corner rather than left top
                    int next = y - 1;
                    if ((prev > (img.rows() - 1)) || (next < 0)) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    if (img(prev, x) == T_INVALID) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    if (img(next, x) == T_INVALID) {
                        delta_img(y, x) = T_INVALID;
                        continue;
                    }
                    delta_img(y, x) = (img(next, x) - img(prev, x)) * 0.5 * (1 / scale);
                } else {
                    delta_img(y, x) = T_INVALID;
                }
            }
        }
    } else {
        std::cerr << "Please input correction direction. Either x or y" << std::endl;
    }
    return delta_img;
}

template <typename T> Eigen::Matrix<T, 3, 3, Eigen::RowMajor> skew(const Eigen::Vector3<T> &v) {
    Eigen::Matrix<T, 3, 3, Eigen::RowMajor> m;
    m.setZero();
    m(0, 1) = -v(2);
    m(0, 2) = v(1);
    m(1, 2) = -v(0);
    m(1, 0) = v(2);
    m(2, 0) = -v(1);
    m(2, 1) = v(0);
    return m;
}

double volume_box(const std::vector<double> &lower_bound, const std::vector<double> &upper_bound) {
    // Check if box dimensions are valid
    if ((upper_bound[0] > lower_bound[0]) && (upper_bound[1] > lower_bound[1]) && (upper_bound[2] > lower_bound[2]))
        return ((upper_bound[0] - lower_bound[0]) * (upper_bound[1] - lower_bound[1]) *
                (upper_bound[2] - lower_bound[2]));
    else
        return 0.0; // Return zero if dimensions are not valid (i.e., no intersection)
}

double IoU_3d(const std::vector<double> &lower_bound_a, const std::vector<double> &upper_bound_a,
              const std::vector<double> &lower_bound_b, const std::vector<double> &upper_bound_b) {
    std::vector<double> intersect_lower_bound(3);
    intersect_lower_bound[0] = std::max(lower_bound_a[0], lower_bound_b[0]);
    intersect_lower_bound[1] = std::max(lower_bound_a[1], lower_bound_b[1]);
    intersect_lower_bound[2] = std::max(lower_bound_a[2], lower_bound_b[2]);

    std::vector<double> intersect_upper_bound(3);
    intersect_upper_bound[0] = std::min(upper_bound_a[0], upper_bound_b[0]);
    intersect_upper_bound[1] = std::min(upper_bound_a[1], upper_bound_b[1]);
    intersect_upper_bound[2] = std::min(upper_bound_a[2], upper_bound_b[2]);
    double intersect_volume = volume_box(intersect_lower_bound, intersect_upper_bound);

    double union_volume =
        volume_box(lower_bound_a, upper_bound_a) + volume_box(lower_bound_b, upper_bound_b) - intersect_volume;

    if (union_volume > 0)
        return intersect_volume / union_volume;
    else
        return 0.0;
}

void create_directory_if_not_exists(const std::string &path) {
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directories(path);
    }
}

int is_directory_non_empty(const std::string &path) {
    if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
        return std::distance(std::filesystem::directory_iterator(path), std::filesystem::directory_iterator());
    }
    return 0;
}

template <typename T>
Eigen::MatrixX<T> generate_box_points(const std::vector<double> &lower_bound, const std::vector<double> &upper_bound) {
    Eigen::MatrixX<T> points(3, 24); // 3 for x, y, z coordinate and 24 for total points.

    // 8 corners
    std::vector<Eigen::Matrix<T, 3, 1>> corners(8);
    corners[0] << lower_bound[0], lower_bound[1], lower_bound[2]; // lower, lower, lower
    corners[1] << upper_bound[0], lower_bound[1], lower_bound[2]; // upper, lower, lower
    corners[2] << upper_bound[0], upper_bound[1], lower_bound[2]; // upper, upper, lower
    corners[3] << lower_bound[0], upper_bound[1], lower_bound[2]; // lower, upper, lower
    corners[4] << lower_bound[0], lower_bound[1], upper_bound[2]; // lower, lower, upper
    corners[5] << upper_bound[0], lower_bound[1], upper_bound[2]; // upper, lower, upper
    corners[6] << upper_bound[0], upper_bound[1], upper_bound[2]; // upper, upper, upper
    corners[7] << lower_bound[0], upper_bound[1], upper_bound[2]; // lower, upper, upper

    // 12 edges
    std::vector<std::pair<int, int>> edges = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                                              {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

    // Fill points
    for (int i = 0; i < 12; ++i) {
        points.col(i * 2) = corners[edges[i].first];
        points.col(i * 2 + 1) = corners[edges[i].second];
    }

    return points;
}

template <typename T = double> T calculate_score(T weight_of_IoU, T largest_distance, T IoU, T centroid_distance) {
    return (weight_of_IoU * IoU +
            (1 - weight_of_IoU) * (1 - std::min(centroid_distance, largest_distance) / largest_distance));
}

} // namespace curl
#endif // CURL_SLAM_CURL_TOOLS_LIGHT_H
