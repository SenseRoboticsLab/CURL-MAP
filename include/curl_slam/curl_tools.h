//
// Created by zkc on 22/04/22.
//

#ifndef EXAMPLE_TOOLS_H
#define EXAMPLE_TOOLS_H

#include "Eigen/Core"
#include "FileReaderBase.h"
#include "Timer.h"
#include "memory"
#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
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

void atan2(Eigen::Ref<Eigen::VectorXd> y, Eigen::Ref<Eigen::VectorXd> x,
           Eigen::Ref<Eigen::VectorXd> result) {
    // #pragma omp parallel for num_threads(16) default(none) shared(result, x, y)
    for (int i = 0; i < y.rows(); i++) {
        if (y(i) != 0 || x(i) > 0) {
            result(i) = 2 * atan(y(i) / (sqrt(x(i) * x(i) + y(i) * y(i)) + x(i)));
        } else if (x(i) < 0 && y(i) == 0) {
            result(i) = M_PI;
        }
    }
}

// output: azi elev r
Matrix3dCol cart2sph(Eigen::Ref<Matrix3dCol> xyz) {
    Matrix3dCol sph(xyz.rows(), 3);
    Eigen::VectorXd xyz_col_x = Eigen::Map<Eigen::VectorXd>(xyz.data(), xyz.rows());
    Eigen::VectorXd xyz_col_y = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows(), xyz.rows());
    Eigen::VectorXd xyz_col_z =
        Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2, xyz.rows());
    Eigen::VectorXd hypotxy =
        (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
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
    Eigen::VectorXd xyz_col_z =
        Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2, xyz.rows());
    Eigen::VectorXd hypotxy =
        (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
    sph.col(2) = (hypotxy.array().pow(2) + xyz.col(2).array().pow(2)).array().sqrt();
    atan2(xyz_col_z, hypotxy, sph.col(1));
    atan2(xyz_col_y, xyz_col_x, sph.col(0));
}

template <typename T>
void cart2sph(Eigen::Matrix<T, Eigen::Dynamic, 3> &xyz, Eigen::Matrix<T, Eigen::Dynamic, 3> sph) {
    Eigen::VectorXd xyz_col_x = Eigen::Map<Eigen::VectorXd>(xyz.data(), xyz.rows());
    Eigen::VectorXd xyz_col_y = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows(), xyz.rows());
    Eigen::VectorXd xyz_col_z =
        Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2, xyz.rows());
    Eigen::VectorXd hypotxy =
        (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
    sph.col(2) = (hypotxy.array().pow(2) + xyz.col(2).array().pow(2)).array().sqrt();
    atan2(xyz_col_z, hypotxy, sph.col(1));
    atan2(xyz_col_y, xyz_col_x, sph.col(0));
}

// Use Eigen::Ref can use parts as the input without const
// output: azi inc r
void cart2sphTable(Eigen::Ref<Matrix3dCol> xyz, Eigen::Ref<Matrix3dCol> sph) {
    Eigen::VectorXd xyz_col_x = Eigen::Map<Eigen::VectorXd>(xyz.data(), xyz.rows());
    Eigen::VectorXd xyz_col_y = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows(), xyz.rows());
    Eigen::VectorXd xyz_col_z =
        Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2, xyz.rows());
    Eigen::VectorXd hypotxy =
        (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
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

template <typename T>
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

void assocLegendre(int l, Eigen::VectorXd X, Eigen::MatrixXd &P) {
    P.resize(l + 1, X.size());
    // #pragma omp parallel for num_threads(16) default(none) shared(l, X, P)
    for (int i = 0; i < X.size(); i++) {
        for (int m = 0; m <= l; m++) {
            P(m, i) = pow(-1, m) * std::assoc_legendre(l, m, X(i));
        }
    }
}

Eigen::VectorXd factorial(Eigen::VectorXi n) {
    Eigen::VectorXd f(n.size());
    double f_mid;
    for (int i = 0; i < n.size(); i++) {
        f_mid = 1;
        for (int j = 1; j <= n(i); j++) {
            f_mid = f_mid * j;
        }
        f(i) = f_mid;
    }
    return f;
}

Eigen::MatrixXd getSH(int N, Eigen::MatrixXd dirs) {
    Eigen::MatrixXd Y_N;
    // initialization
    Eigen::VectorXi m;
    Eigen::MatrixXd Lnm_real;
    Eigen::MatrixXd condon;
    Y_N.resize(int(pow((N + 1), 2)), dirs.rows());
    Y_N.setZero();
    int idx_Y = 0;
    Eigen::VectorXd norm_real;
    Eigen::MatrixXd Nnm_real;
    Eigen::MatrixXd Nnm_real_mid;
    Eigen::MatrixXd CosSin;
    Eigen::MatrixXd Ynm;
    // start calculation
    for (int n = 0; n <= N; n++) {
        m = Eigen::VectorXi::LinSpaced(n + 1, 0, n);
        assocLegendre(
            n, cos((Eigen::Map<Eigen::VectorXd>(dirs.data() + dirs.rows(), dirs.rows())).array()),
            Lnm_real);
        if (n != 0) {
            Eigen::MatrixXi condon_mid(m.size() + m.size() - 1, 1);
            condon_mid << m(Eigen::seq(m.size() - 1, 2 - 1, -1)), m;
            condon = Eigen::MatrixXd::Ones(m.size() + m.size() - 1, 1).array() * (-1);
            condon = condon.array().pow(condon_mid.array().cast<double>()).matrix() *
                     Eigen::MatrixXd::Ones(1, dirs.rows());
            Eigen::MatrixXd Lnm_real_mid(Lnm_real.rows() + Lnm_real.rows() - 1, Lnm_real.cols());
            Lnm_real_mid << Lnm_real(Eigen::seq(Lnm_real.rows() - 1, 2 - 1, -1), Eigen::all),
                Lnm_real;
            Lnm_real = condon.array() * Lnm_real_mid.array();
        }
        // normalisations
        norm_real = ((2 * n + 1) * factorial(n - m.array()).array() /
                     (4 * M_PIl * factorial(n + m.array())).array())
                        .array()
                        .sqrt();
        // convert to matrix, for direct matrix multiplication with the rest
        Nnm_real = norm_real * Eigen::MatrixXd::Ones(1, dirs.rows());
        if (n != 0) {
            Nnm_real_mid.resize(Nnm_real.rows() + Nnm_real.rows() - 1, Nnm_real.cols());
            Nnm_real_mid << Nnm_real(Eigen::seq(Nnm_real.rows() - 1, 2 - 1, -1), Eigen::all),
                Nnm_real;
            Nnm_real = Nnm_real_mid;
        }
        CosSin = Eigen::MatrixXd::Zero(2 * n + 1, dirs.rows());
        // zero degree
        CosSin(n, Eigen::all) = Eigen::MatrixXd::Ones(1, dirs.rows());
        // positive and negative degrees
        if (n != 0) {
            CosSin(m(Eigen::seq(2 - 1, m.size() - 1)).array() + n, Eigen::all) =
                sqrt(2) * cos((m(Eigen::seq(2 - 1, m.size() - 1)).cast<double>() *
                               (dirs(Eigen::all, 0).transpose()))
                                  .array());
            CosSin(-m(Eigen::seq(m.size() - 1, 2 - 1, -1)).array() + n, Eigen::all) =
                sqrt(2) * sin((m(Eigen::seq(m.size() - 1, 2 - 1, -1)).cast<double>() *
                               (dirs(Eigen::all, 0).transpose()))
                                  .array());
        }
        Ynm = Nnm_real.array() * Lnm_real.array() * CosSin.array();
        Y_N(Eigen::seq(idx_Y, idx_Y + (2 * n)), Eigen::all) = Ynm;
        idx_Y = idx_Y + 2 * n + 1;
    }
    Y_N.transposeInPlace();
    return Y_N;
}

Eigen::VectorXd leastSquaresSHT(int N, const Eigen::VectorXd &F, Eigen::MatrixXd dirs) {
    Eigen::VectorXd F_N;
    Eigen::MatrixXd Y_N;
    Y_N = getSH(N, std::move(dirs));
    F_N = Y_N.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(F);
    return F_N;
}

Eigen::VectorXd invLeastSquaresSHT(const Eigen::VectorXd &coefficients, Eigen::MatrixXd dirs) {
    //    std::cout << "Inverse sph transform" << std::endl;
    Eigen::VectorXd F;
    int degree;
    degree = int(sqrt(int(coefficients.size()))) - 1;
    F = getSH(degree, std::move(dirs)) * coefficients;
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

void loadCoefficients(const std::string &read_dir, std::vector<Eigen::VectorXd> &coefficients,
                      int &row, int &col) {
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
T P(int i, int l, int a, int b, const Eigen::MatrixBase<Derived> &R_1,
    const Eigen::MatrixBase<Derived> &R_lm1) {
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
T U(int l, int m, int n, const Eigen::MatrixBase<Derived> &R_1,
    const Eigen::MatrixBase<Derived> &R_lm1) {
    return P<Derived, T>(0, l, m, n, R_1, R_lm1);
}

template <typename Derived, typename T>
T V(int l, int m, int n, const Eigen::MatrixBase<Derived> &R_1,
    const Eigen::MatrixBase<Derived> &R_lm1) {
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
T Wf(int l, int m, int n, const Eigen::MatrixBase<Derived> &R_1,
     const Eigen::MatrixBase<Derived> &R_lm1) {
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

template <typename Derived, typename T>
Derived getSHrotMtx(const Eigen::MatrixBase<Derived> &R_xyz, int degree) {
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
                v = sqrt((1 + double(d)) * (l + abs(m) - 1) * (l + abs(m)) / denom) *
                    (1 - 2 * double(d)) * 0.5;
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

// mesh viewer
template <typename BasicType>
void mesh_viewer(Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> V,
                 Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F,
                 Eigen::Matrix<double, Eigen::Dynamic, 3> color) {
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(color);
    viewer.launch();
}

// mesh viewer
template <typename BasicType>
void mesh_viewer_no_color(Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> V,
                          Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F) {
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    const Eigen::RowVector3d blue(0.0, 0.2, 0.8);
    viewer.data().set_colors(blue);
    viewer.launch();
}

template <typename BasicType>
void mesh_color_viewer(Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> V,
                       Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F,
                       Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> data) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> colors(data.rows(), 3);
    Eigen::MatrixXd colormap = FileReaderBase::read_txt_file<double>(
        "/home/user/project/CURL_SLAM/tmp_data/color_map/parula.txt");
    Eigen::VectorXi seg_idx = Eigen::VectorXi::Zero(V.rows());
    for (int i = 0; i < data.rows(); ++i) {
        colors.row(i) = colormap.row(
            static_cast<int>((data(i) - data.minCoeff()) / (data.maxCoeff() - data.minCoeff()) *
                             (colormap.rows() - 1)));
    }
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(colors);
    viewer.launch();
}
// mesh normal viewer
template <typename BasicType>
void mesh_vertex_normal_viewer(Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> V,
                               Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F,
                               Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> normals) {
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    const Eigen::RowVector3d red(0.8, 0.2, 0.2);
    viewer.data().add_edges(V, V + normals * 0.2, red);
    viewer.launch();
}

template <typename BasicType>
void mesh_data_viewer(Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> V,
                      Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F,
                      Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> data) {
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_data(data);
    viewer.launch();
}

template <typename BasicType>
void point_viewer(Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> V, int point_size,
                  Eigen::Matrix<double, Eigen::Dynamic, 3> color) {
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_points(V, color);
    viewer.data().point_size = point_size;
    viewer.launch();
}

template <typename BasicType>
void point_normal_viewer(Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> V,
                         Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> normals,
                         int point_size, Eigen::Matrix<double, Eigen::Dynamic, 3> color) {
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_points(V, color);
    viewer.data().point_size = point_size;
    const Eigen::RowVector3d red(0.8, 0.2, 0.2);
    viewer.data().add_edges(V, V + normals * 0.2, red);
    viewer.launch();
}

template <typename BasicType>
void point_viewer_pedigree_chart(
    Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> V, int point_size,
    Eigen::Matrix<double, Eigen::Dynamic, 3> color,
    Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> pedigree_chart) {
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_points(V, color);
    viewer.data().point_size = point_size;
    const Eigen::RowVector3d blue(0.2, 0.2, 0.8);
    viewer.data().add_edges(pedigree_chart(Eigen::all, Eigen::seq(0, 2)),
                            pedigree_chart(Eigen::all, Eigen::seq(3, 5)), blue);
    viewer.launch();
}

template <typename BasicType, typename DerviedV, typename DerviedF>
void points_curvature_viewer(const Eigen::MatrixBase<DerviedV> &V,
                             const Eigen::MatrixBase<DerviedF> &F,
                             Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> curvature,
                             Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> normals =
                                 Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic>()) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> colors(curvature.rows(), 3);
    Eigen::MatrixXd colormap = FileReaderBase::read_txt_file<double>(
        "/home/user/project/CURL_SLAM/tmp_data/color_map/parula.txt");
    Eigen::VectorXi seg_idx = Eigen::VectorXi::Zero(V.rows());
    int cap_num = 300;
    for (int i = 0; i < curvature.rows(); ++i) {
        if (curvature(i) > cap_num) {
            curvature(i) = cap_num;
        } else if (curvature(i) < -cap_num) {
            curvature(i) = -cap_num;
        }
        colors.row(i) = colormap.row(static_cast<int>(
            (curvature(i) - (-cap_num)) / (cap_num - (-cap_num)) * (colormap.rows() - 1)));
    }
    if (normals.size() > 0) {
        point_normal_viewer<BasicType>(V, normals, 3, colors);
    } else {
        point_viewer<BasicType>(V, 3, colors);
    }
}

template <typename BasicType>
void mesh_normal_viewer(Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> V,
                        Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F,
                        Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> normals,
                        Eigen::Matrix<double, Eigen::Dynamic, 3> color) {
    Eigen::MatrixXd centroid_points(F.rows(), 3);
    for (int i = 0; i < F.rows(); ++i) {
        centroid_points(i, Eigen::all) = (V.row(F(i, 0)) + V.row(F(i, 1)) + V.row(F(i, 2))) / 3;
    }
    igl::opengl::glfw::Viewer viewer;
    //    viewer.data().set_mesh(V, F);
    //    viewer.data().set_colors(color);
    viewer.data().set_points(V, color);
    viewer.data().point_size = 3;
    const Eigen::RowVector3d red(0.8, 0.2, 0.2);
    viewer.data().add_edges(centroid_points, centroid_points + normals * 0.2, red);
    viewer.launch();
}

template <typename BasicType, typename DerviedV, typename DerviedF>
void mesh_curvature_viewer(const Eigen::MatrixBase<DerviedV> &V,
                           const Eigen::MatrixBase<DerviedF> &F,
                           Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> curvature,
                           Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> normals =
                               Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic>()) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> colors(curvature.rows(), 3);
    Eigen::MatrixXd colormap = FileReaderBase::read_txt_file<double>(
        "/home/user/project/CURL_SLAM/tmp_data/color_map/parula.txt");
    Eigen::VectorXi seg_idx = Eigen::VectorXi::Zero(V.rows());
    int cap_num = 300;
    for (int i = 0; i < curvature.rows(); ++i) {
        if (curvature(i) > cap_num) {
            curvature(i) = cap_num;
        } else if (curvature(i) < -cap_num) {
            curvature(i) = -cap_num;
        }
        colors.row(i) = colormap.row(static_cast<int>(
            (curvature(i) - (-cap_num)) / (cap_num - (-cap_num)) * (colormap.rows() - 1)));
    }
    if (normals.size() > 0) {
        mesh_normal_viewer<BasicType>(V, F, normals, colors);
    } else {
        mesh_viewer<BasicType>(V, F, colors);
    }
}

template <typename BasicType, typename DerviedV, typename DerviedF>
void points_data_viewer(const Eigen::MatrixBase<DerviedV> &V, const Eigen::MatrixBase<DerviedF> &F,
                        Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> data,
                        Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> normals =
                            Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic>()) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> colors(data.rows(), 3);
    Eigen::MatrixXd colormap = FileReaderBase::read_txt_file<double>(
        "/home/user/project/CURL_SLAM/tmp_data/color_map/parula.txt");
    Eigen::VectorXi seg_idx = Eigen::VectorXi::Zero(V.rows());
    for (int i = 0; i < data.rows(); ++i) {
        colors.row(i) = colormap.row(
            static_cast<int>((data(i) - data.minCoeff()) / (data.maxCoeff() - data.minCoeff()) *
                             (colormap.rows() - 1)));
    }
    if (normals.size() > 0) {
        point_normal_viewer<BasicType>(V, normals, 3, colors);
    } else {
        point_viewer<BasicType>(V, 3, colors);
    }
}
// points vertex normal

std::vector<int> random_idx_generator(int number, int lower_bound, int higher_bound) {
    std::vector<int> random_idx;
    std::random_device rd;  // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(lower_bound, higher_bound); // define the range

    for (int n = 0; n < number; ++n)
        random_idx.push_back(distr(gen)); // generate numbers
    return random_idx;
}

template <typename BasicType>
void random_color_points_plot_with_pedigree_chart(
    const Eigen::MatrixXd V, const Eigen::MatrixXi F, const Eigen::VectorXi scan_segmentation,
    const Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> pedigree_chart) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> colors(V.size(), 3);
    Eigen::MatrixXd colormap = FileReaderBase::read_txt_file<double>(
        "/home/user/project/CURL_SLAM/tmp_data/color_map/parula.txt");
    std::vector<int> random_idx;
    random_idx = curl::random_idx_generator(static_cast<int>(scan_segmentation.maxCoeff() + 1), 0,
                                            static_cast<int>(colormap.rows() - 1));
    Eigen::VectorXi seg_idx = Eigen::VectorXi::Zero(V.rows());
    for (int i = 0; i < scan_segmentation.size(); ++i) {
        int v_i = F(i, 0);
        int v_j = F(i, 1);
        int v_k = F(i, 2);
        if (seg_idx(v_i) == 0) {
            colors.row(v_i) = colormap.row(random_idx[scan_segmentation(i)]);
            seg_idx(v_i) = scan_segmentation(i);
        } else if (seg_idx(v_i) != scan_segmentation(i)) {
            colors.row(v_i) = Eigen::RowVector3d(1, 0, 0);
        }
        if (seg_idx(v_j) == 0) {
            colors.row(v_j) = colormap.row(random_idx[scan_segmentation(i)]);
            seg_idx(v_j) = scan_segmentation(i);
        } else if (seg_idx(v_j) != scan_segmentation(i)) {
            colors.row(v_j) = Eigen::RowVector3d(1, 0, 0);
        }
        if (seg_idx(v_k) == 0) {
            colors.row(v_k) = colormap.row(random_idx[scan_segmentation(i)]);
            seg_idx(v_k) = scan_segmentation(i);
        } else if (seg_idx(v_k) != scan_segmentation(i)) {
            colors.row(v_k) = Eigen::RowVector3d(1, 0, 0);
        }
    }
    point_viewer_pedigree_chart<BasicType>(V, 3, colors, pedigree_chart);
}

template <typename BasicType, typename DerviedV, typename DerviedF>
void vsa_color_points_plot(const Eigen::MatrixBase<DerviedV> &V,
                           const Eigen::MatrixBase<DerviedF> &F,
                           const Eigen::VectorXi &scan_segmentation) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> colors(V.size(), 3);
    Eigen::MatrixXd colormap = FileReaderBase::read_txt_file<double>(
        "/home/user/project/CURL_SLAM/tmp_data/color_map/parula.txt");
    std::vector<int> random_idx;
    random_idx = curl::random_idx_generator(static_cast<int>(scan_segmentation.maxCoeff() + 1), 0,
                                            static_cast<int>(colormap.rows() - 1));
    Eigen::VectorXi seg_idx = Eigen::VectorXi::Zero(V.rows());
    for (int i = 0; i < scan_segmentation.size(); ++i) {
        int v_i = F(i, 0);
        int v_j = F(i, 1);
        int v_k = F(i, 2);
        if (seg_idx(v_i) == 0) {
            colors.row(v_i) = colormap.row(random_idx[scan_segmentation(i)]);
            seg_idx(v_i) = scan_segmentation(i);
        } else if (seg_idx(v_i) != scan_segmentation(i)) {
            colors.row(v_i) = Eigen::RowVector3d(1, 0, 0);
        }
        if (seg_idx(v_j) == 0) {
            colors.row(v_j) = colormap.row(random_idx[scan_segmentation(i)]);
            seg_idx(v_j) = scan_segmentation(i);
        } else if (seg_idx(v_j) != scan_segmentation(i)) {
            colors.row(v_j) = Eigen::RowVector3d(1, 0, 0);
        }
        if (seg_idx(v_k) == 0) {
            colors.row(v_k) = colormap.row(random_idx[scan_segmentation(i)]);
            seg_idx(v_k) = scan_segmentation(i);
        } else if (seg_idx(v_k) != scan_segmentation(i)) {
            colors.row(v_k) = Eigen::RowVector3d(1, 0, 0);
        }
    }
    point_viewer<BasicType>(V, 3, colors);
}

template <typename BasicType, typename DerviedV>
void region_growing_points_plot(const Eigen::MatrixBase<DerviedV> &V,
                                const Eigen::Matrix<double, Eigen::Dynamic, 3> &colors) {
    point_viewer<BasicType>(V, 3, colors);
}

void convert_to_point_cloud(const Eigen::MatrixXd &source_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    cloud->points.resize(source_cloud.rows());
    for (int i = 0; i < source_cloud.rows(); i++) {
        cloud->points[i].getVector3fMap() =
            Eigen::Vector3d(source_cloud(i, 0), source_cloud(i, 1), source_cloud(i, 2))
                .cast<float>();
    }
}

} // namespace curl

#endif // EXAMPLE_TOOLS_H
