//
// Created by zkc on 15/03/23.
//

#ifndef CURL_SLAM_OPTIMIZATION_TYPES_H
#define CURL_SLAM_OPTIMIZATION_TYPES_H

#include "FileReader.h"
#include "RosHandler.h"
#include "curl_tools_light.h"
#include "linear_interpolation_2.h"
#include "linear_interpolation_2_func.h"
#include "load_config.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/autodiff_cost_function.h>
#include <ceres/autodiff_manifold.h>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <chrono>
#include <iostream>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <sstream>
#include <thread>
#include <visualization_msgs/Marker.h>

/******************* SO3 Manifold *******************/
class SO3Manifold {
  public:
    template <typename T> bool Plus(const T *x_SO3, const T *delta_so3, T *x_plus_delta) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> x(x_SO3);
        Sophus::SO3<T> x_on_manifold(x);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> delta(delta_so3);
        Sophus::SO3<T> delta_on_manifold = Sophus::SO3<T>::exp(delta);
        Sophus::SO3<T> x_hat = x_on_manifold * delta_on_manifold;
        Eigen::Matrix<T, 3, 3> x_hat_manifold = x_hat.matrix();
        Eigen::Map<Eigen::Matrix<T, 3, 3>> residuals_map(x_plus_delta);
        residuals_map.template block<3, 3>(0, 0) = x_hat_manifold;
        return true;
    }

    template <typename T> bool Minus(const T *y_SO3, const T *x_SO3, T *y_minus_x) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> y(y_SO3);
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> x(x_SO3);
        Sophus::SO3<T> y_on_manifold(y);
        Sophus::SO3<T> x_on_manifold(x);

        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(y_minus_x);
        residuals_map = (x_on_manifold.inverse() * y_on_manifold).log();
        return true;
    }

    static ceres::Manifold *Create() { return new ceres::AutoDiffManifold<SO3Manifold, 9, 3>; }
};

class SE3Manifold : public ceres::Manifold {
  public:
    SE3Manifold() {}

    ~SE3Manifold() override {}

    int AmbientSize() const override { return 16; }

    int TangentSize() const override { return 6; }

    bool Plus(const double *x_SE3, const double *delta_se3, double *x_plus_delta) const override {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> x(x_SE3);
        Sophus::SE3<double> x_on_manifold(x);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta(delta_se3);
        Sophus::SE3<double> delta_on_manifold = Sophus::SE3<double>::exp(delta);
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> residuals_map(x_plus_delta);
        residuals_map = (x_on_manifold * delta_on_manifold).matrix();
        return true;
    }

    bool PlusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 16, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    bool Minus(const double *y_SE3, const double *x_SE3, double *y_minus_x) const override {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> y(y_SE3);
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> x(x_SE3);
        Sophus::SE3<double> y_on_manifold(y);
        Sophus::SE3<double> x_on_manifold(x);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residuals_map(y_minus_x);
        residuals_map = (x_on_manifold.inverse() * y_on_manifold).log();
        return true;
    }

    bool MinusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 16, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    static ceres::Manifold *Create() { return new SE3Manifold(); }
};

class SE3ManifoldXYYall : public ceres::Manifold {
  public:
    SE3ManifoldXYYall() {}

    ~SE3ManifoldXYYall() override {}

    int AmbientSize() const override { return 16; }

    int TangentSize() const override { return 6; }

    bool Plus(const double *x_SE3, const double *delta_se3, double *x_plus_delta) const override {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> x_on_manifold(x_SE3);
        //        Eigen::Vector3d euler_original =
        //            Eigen::Matrix3d(x_on_manifold(Eigen::seq(0, 2), Eigen::seq(0, 2))).eulerAngles(0, 1, 2);
        //        Eigen::Matrix<double, 6, 1> x_se3 = Sophus::SE3<double>(x_on_manifold).log();
        //        Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta(delta_se3);
        //        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> update_x_SE3(x_plus_delta);
        //        update_x_SE3 = x_on_manifold * Sophus::SE3<double>::exp(delta).matrix();
        //        Eigen::Vector3d euler_update =
        //            Eigen::Matrix3d(update_x_SE3(Eigen::seq(0, 2), Eigen::seq(0, 2))).eulerAngles(0, 1, 2);
        //        euler_update(0) = euler_original(0);
        //        euler_update(1) = euler_original(1);
        //        update_x_SE3(Eigen::seq(0, 2), Eigen::seq(0, 2)) =
        //            (Eigen::AngleAxisd(euler_update(0), Eigen::Vector3d::UnitX()) *
        //             Eigen::AngleAxisd(euler_update(1), Eigen::Vector3d::UnitY()) *
        //             Eigen::AngleAxisd(euler_update(2), Eigen::Vector3d::UnitZ()))
        //                .matrix();
        //        update_x_SE3(2, 3) = x_on_manifold(2, 3);
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> x(x_SE3);
        Sophus::SE3<double> x_on_manifold_XYYall(x);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta(delta_se3);
        Eigen::Matrix<double, 6, 1> delta_XYYall = delta;
        delta_XYYall(3) = 0;
        delta_XYYall(4) = 0;
        Sophus::SE3<double> delta_on_manifold = Sophus::SE3<double>::exp(delta_XYYall);
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> residuals_map(x_plus_delta);
        residuals_map = (x_on_manifold_XYYall * delta_on_manifold).matrix();
        residuals_map(2, 3) = x_on_manifold_XYYall.matrix()(2, 3);
        return true;
    }

    bool PlusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 16, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    bool Minus(const double *y_SE3, const double *x_SE3, double *y_minus_x) const override {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> y(y_SE3);
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> x(x_SE3);
        Sophus::SE3<double> y_on_manifold(y);
        Sophus::SE3<double> x_on_manifold(x);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residuals_map(y_minus_x);
        residuals_map = (x_on_manifold.inverse() * y_on_manifold).log();
        return true;
    }

    bool MinusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 16, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    static ceres::Manifold *Create() { return new SE3ManifoldXYYall(); }
};

class SE3ManifoldZRollPitch : public ceres::Manifold {
  public:
    SE3ManifoldZRollPitch() {}

    ~SE3ManifoldZRollPitch() override {}

    int AmbientSize() const override { return 16; }

    int TangentSize() const override { return 6; }

    bool Plus(const double *x_SE3, const double *delta_se3, double *x_plus_delta) const override {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> x_on_manifold(x_SE3);
        //        Eigen::Vector3d euler_original =
        //            Eigen::Matrix3d(x_on_manifold(Eigen::seq(0, 2), Eigen::seq(0, 2))).eulerAngles(0, 1, 2);
        //        Eigen::Matrix<double, 6, 1> x_se3 = Sophus::SE3<double>(x_on_manifold).log();
        //        Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta(delta_se3);
        //        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> update_x_SE3(x_plus_delta);
        //        update_x_SE3 = x_on_manifold * Sophus::SE3<double>::exp(delta).matrix();
        //        Eigen::Vector3d euler_update =
        //            Eigen::Matrix3d(update_x_SE3(Eigen::seq(0, 2), Eigen::seq(0, 2))).eulerAngles(0, 1, 2);
        //        euler_update(2) = euler_original(2);
        //        update_x_SE3(Eigen::seq(0, 2), Eigen::seq(0, 2)) =
        //            (Eigen::AngleAxisd(euler_update(0), Eigen::Vector3d::UnitX()) *
        //             Eigen::AngleAxisd(euler_update(1), Eigen::Vector3d::UnitY()) *
        //             Eigen::AngleAxisd(euler_update(2), Eigen::Vector3d::UnitZ()))
        //                .matrix();
        //        update_x_SE3(0, 3) = x_on_manifold(0, 3);
        //        update_x_SE3(1, 3) = x_on_manifold(1, 3);
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> x(x_SE3);
        Sophus::SE3<double> x_on_manifold_ZRollPitch(x);
        Eigen::Vector3d euler_original =
            Eigen::Matrix3d(x_on_manifold_ZRollPitch.matrix()(Eigen::seq(0, 2), Eigen::seq(0, 2))).eulerAngles(0, 1, 2);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta(delta_se3);
        Eigen::Matrix<double, 6, 1> delta_ZRollPitch = delta;
        delta_ZRollPitch(5) = 0;
        Sophus::SE3<double> delta_on_manifold = Sophus::SE3<double>::exp(delta_ZRollPitch);
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> residuals_map(x_plus_delta);
        residuals_map = (x_on_manifold_ZRollPitch * delta_on_manifold).matrix();
        residuals_map(0, 3) = x_on_manifold_ZRollPitch.matrix()(0, 3);
        residuals_map(1, 3) = x_on_manifold_ZRollPitch.matrix()(1, 3);
        return true;
    }

    bool PlusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 16, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    bool Minus(const double *y_SE3, const double *x_SE3, double *y_minus_x) const override {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> y(y_SE3);
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> x(x_SE3);
        Sophus::SE3<double> y_on_manifold(y);
        Sophus::SE3<double> x_on_manifold(x);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residuals_map(y_minus_x);
        residuals_map = (x_on_manifold.inverse() * y_on_manifold).log();
        return true;
    }

    bool MinusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 16, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    static ceres::Manifold *Create() { return new SE3ManifoldZRollPitch(); }
};

class se3Manifold : public ceres::Manifold {
  public:
    se3Manifold() {}

    ~se3Manifold() override {}

    int AmbientSize() const override { return 6; }

    int TangentSize() const override { return 6; }

    bool Plus(const double *x_se3, const double *delta_se3, double *x_plus_delta) const override {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> x(x_se3);
        Sophus::SE3<double> x_on_manifold = Sophus::SE3<double>::exp(x);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta(delta_se3);
        Sophus::SE3<double> delta_on_manifold = Sophus::SE3<double>::exp(delta);
        Eigen::Map<Eigen::Matrix<double, 6, 1>> residuals_map(x_plus_delta);
        residuals_map = (x_on_manifold * delta_on_manifold).log().matrix();
        return true;
    }

    bool PlusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    bool Minus(const double *y_se3, const double *x_se3, double *y_minus_x) const override {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> y(y_se3);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> x(x_se3);
        Sophus::SE3<double> y_on_manifold = Sophus::SE3<double>::exp(y);
        Sophus::SE3<double> x_on_manifold = Sophus::SE3<double>::exp(x);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residuals_map(y_minus_x);
        residuals_map = (x_on_manifold.inverse() * y_on_manifold).log();
        return true;
    }

    bool MinusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    static ceres::Manifold *Create() { return new se3Manifold(); }
};

class se3ManifoldXYYall : public ceres::Manifold {
  public:
    se3ManifoldXYYall() {}

    ~se3ManifoldXYYall() override {}

    int AmbientSize() const override { return 6; }

    int TangentSize() const override { return 6; }

    bool Plus(const double *x_se3, const double *delta_se3, double *x_plus_delta) const override {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> x(x_se3);
        Sophus::SE3<double> x_on_manifold = Sophus::SE3<double>::exp(x);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta(delta_se3);
        Sophus::SE3<double> delta_on_manifold = Sophus::SE3<double>::exp(delta);
        Eigen::Map<Eigen::Matrix<double, 6, 1>> residuals_map(x_plus_delta);
        residuals_map = (x_on_manifold * delta_on_manifold).log().matrix();
        residuals_map(2) = x(2);
        residuals_map(3) = x(3);
        residuals_map(4) = x(4);
        return true;
    }

    bool PlusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    bool Minus(const double *y_se3, const double *x_se3, double *y_minus_x) const override {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> y(y_se3);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> x(x_se3);
        Sophus::SE3<double> y_on_manifold = Sophus::SE3<double>::exp(y);
        Sophus::SE3<double> x_on_manifold = Sophus::SE3<double>::exp(x);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residuals_map(y_minus_x);
        residuals_map = (x_on_manifold.inverse() * y_on_manifold).log();
        return true;
    }

    bool MinusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    static ceres::Manifold *Create() { return new se3ManifoldXYYall(); }
};

class se3ManifoldZRollPitch : public ceres::Manifold {
  public:
    se3ManifoldZRollPitch() {}

    ~se3ManifoldZRollPitch() override {}

    int AmbientSize() const override { return 6; }

    int TangentSize() const override { return 6; }

    bool Plus(const double *x_se3, const double *delta_se3, double *x_plus_delta) const override {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> x(x_se3);
        Sophus::SE3<double> x_on_manifold = Sophus::SE3<double>::exp(x);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta(delta_se3);
        Sophus::SE3<double> delta_on_manifold = Sophus::SE3<double>::exp(delta);
        Eigen::Map<Eigen::Matrix<double, 6, 1>> residuals_map(x_plus_delta);
        residuals_map = (x_on_manifold * delta_on_manifold).log().matrix();
        residuals_map(0) = x(0);
        residuals_map(1) = x(1);
        residuals_map(5) = x(5);
        return true;
    }

    bool PlusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    bool Minus(const double *y_se3, const double *x_se3, double *y_minus_x) const override {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> y(y_se3);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> x(x_se3);
        Sophus::SE3<double> y_on_manifold = Sophus::SE3<double>::exp(y);
        Sophus::SE3<double> x_on_manifold = Sophus::SE3<double>::exp(x);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residuals_map(y_minus_x);
        residuals_map = (x_on_manifold.inverse() * y_on_manifold).log();
        return true;
    }

    bool MinusJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    static ceres::Manifold *Create() { return new se3ManifoldZRollPitch(); }
};

/******************* Odometry Factor *******************/

class OdometryFactor {
  public:
    OdometryFactor(const Eigen::Matrix3d &_R_z, const Eigen::Vector3d &_t_z) : R_z(_R_z), t_z(_t_z) {
        R_z = Eigen::Quaterniond(R_z).normalized().toRotationMatrix();
    }

    OdometryFactor(const Eigen::Isometry3d &_T_z) : T_z(_T_z) {
        R_z = Eigen::Quaterniond(T_z.rotation()).normalized().toRotationMatrix();
        t_z = T_z.translation();
    }

    ~OdometryFactor() = default;

    template <typename T>
    bool operator()(const T *const R_w_j_ptr, const T *const t_w_j_ptr, const T *const R_w_j_1_ptr,
                    const T *const t_w_j_1_ptr, T *residual_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j(R_w_j_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j(t_w_j_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j_1(R_w_j_1_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j_1(t_w_j_1_ptr);
        // residual equation
        Sophus::SO3<T> R_err_on_manifold(R_z * R_w_j_1.transpose() * R_w_j);
        Eigen::Matrix<T, 3, 1> R_err = R_err_on_manifold.log();
        Eigen::Matrix<T, 3, 1> t_err = R_z * R_w_j_1.transpose() * t_w_j - R_z * R_w_j_1.transpose() * t_w_j_1 + t_z;
        // bind residual to pointer
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residual_ptr);
        residuals_map.template head<3>() = R_err;
        residuals_map.template segment<3>(3) = t_err;
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix3d &_R_z, const Eigen::Vector3d &_t_z) {
        return (new ceres::AutoDiffCostFunction<OdometryFactor, 6, 9, 3, 9, 3>(new OdometryFactor(_R_z, _t_z)));
    }

    static ceres::CostFunction *Create(const Eigen::Isometry3d &_T_z) {
        return (new ceres::AutoDiffCostFunction<OdometryFactor, 6, 9, 3, 9, 3>(new OdometryFactor(_T_z)));
    }

  public:
    Eigen::Isometry3d T_z;
    Eigen::Matrix3d R_z;
    Eigen::Vector3d t_z;
};

class OdometryInitFixFactor {
  public:
    OdometryInitFixFactor(const Eigen::Matrix3d &_R_z, const Eigen::Vector3d &_t_z, const Eigen::Matrix3d &_R_j_1_w,
                          const Eigen::Vector3d &_t_j_1_w)
        : R_z(_R_z), t_z(_t_z), R_w_j_1(_R_j_1_w), t_w_j_1(_t_j_1_w) {
        R_z = Eigen::Quaterniond(R_z).normalized().toRotationMatrix();
        R_w_j_1 = Eigen::Quaterniond(R_w_j_1).normalized().toRotationMatrix();
    }

    OdometryInitFixFactor(const Eigen::Isometry3d &_T_z, const Eigen::Isometry3d &_T_j_1_w)
        : T_z(_T_z), T_j_1_w(_T_j_1_w) {
        R_z = Eigen::Quaterniond(T_z.rotation()).normalized().toRotationMatrix();
        t_z = T_z.translation();
        R_w_j_1 = Eigen::Quaterniond(T_j_1_w.rotation()).normalized().toRotationMatrix();
        t_w_j_1 = T_j_1_w.translation();
    }

    ~OdometryInitFixFactor() = default;

    template <typename T> bool operator()(const T *const R_w_j_ptr, const T *const t_w_j_ptr, T *residual_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j(R_w_j_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j(t_w_j_ptr);
        // residual equation
        Sophus::SO3<T> R_err_on_manifold(R_z * R_w_j_1.transpose() * R_w_j);
        Eigen::Matrix<T, 3, 1> R_err = R_err_on_manifold.log();
        Eigen::Matrix<T, 3, 1> t_err = R_z * R_w_j_1.transpose() * t_w_j - R_z * R_w_j_1.transpose() * t_w_j_1 + t_z;
        // bind residual to pointer
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residual_ptr);
        residuals_map.template head<3>() = R_err;
        residuals_map.template segment<3>(3) = t_err;
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix3d &_R_z, const Eigen::Vector3d &_t_z,
                                       const Eigen::Matrix3d &_R_j_1_w, const Eigen::Vector3d &_t_j_1_w) {
        return (new ceres::AutoDiffCostFunction<OdometryInitFixFactor, 6, 9, 3>(
            new OdometryInitFixFactor(_R_z, _t_z, _R_j_1_w, _t_j_1_w)));
    }

    static ceres::CostFunction *Create(const Eigen::Isometry3d &_T_z, const Eigen::Isometry3d &_T_j_1_w) {
        return (
            new ceres::AutoDiffCostFunction<OdometryInitFixFactor, 6, 9, 3>(new OdometryInitFixFactor(_T_z, _T_j_1_w)));
    }

  public:
    Eigen::Isometry3d T_z;
    Eigen::Matrix3d R_z;
    Eigen::Vector3d t_z;
    Eigen::Isometry3d T_j_1_w;
    Eigen::Matrix3d R_w_j_1;
    Eigen::Vector3d t_w_j_1;
};

class OdometryEndFixFactor {
  public:
    OdometryEndFixFactor(const Eigen::Matrix3d &_R_z, const Eigen::Vector3d &_t_z, const Eigen::Matrix3d &_R_j_w,
                         const Eigen::Vector3d &_t_j_w)
        : R_z(_R_z), t_z(_t_z), R_w_j(_R_j_w), t_w_j(_t_j_w) {
        R_z = Eigen::Quaterniond(R_z).normalized().toRotationMatrix();
        R_w_j = Eigen::Quaterniond(R_w_j).normalized().toRotationMatrix();
    }

    OdometryEndFixFactor(const Eigen::Isometry3d &_T_z, const Eigen::Isometry3d &_T_j_w) : T_z(_T_z), T_j_w(_T_j_w) {
        R_z = Eigen::Quaterniond(T_z.rotation()).normalized().toRotationMatrix();
        t_z = T_z.translation();
        R_w_j = Eigen::Quaterniond(T_j_w.rotation()).normalized().toRotationMatrix();
        t_w_j = T_j_w.translation();
    }

    ~OdometryEndFixFactor() = default;

    template <typename T>
    bool operator()(const T *const R_w_j_1_ptr, const T *const t_w_j_1_ptr, T *residual_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j_1(R_w_j_1_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j_1(t_w_j_1_ptr);
        // residual equation
        Sophus::SO3<T> R_err_on_manifold(R_z * R_w_j_1.transpose() * R_w_j);
        Eigen::Matrix<T, 3, 1> R_err = R_err_on_manifold.log();
        Eigen::Matrix<T, 3, 1> t_err = R_z * R_w_j_1.transpose() * t_w_j - R_z * R_w_j_1.transpose() * t_w_j_1 + t_z;
        // bind residual to pointer
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residual_ptr);
        residuals_map.template head<3>() = R_err;
        residuals_map.template segment<3>(3) = t_err;
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix3d &_R_z, const Eigen::Vector3d &_t_z,
                                       const Eigen::Matrix3d &_R_j_w, const Eigen::Vector3d &_t_j_w) {
        return (new ceres::AutoDiffCostFunction<OdometryEndFixFactor, 6, 9, 3>(
            new OdometryEndFixFactor(_R_z, _t_z, _R_j_w, _t_j_w)));
    }

    static ceres::CostFunction *Create(const Eigen::Isometry3d &_T_z, const Eigen::Isometry3d &_T_j_w) {
        return (new ceres::AutoDiffCostFunction<OdometryEndFixFactor, 6, 9, 3>(new OdometryEndFixFactor(_T_z, _T_j_w)));
    }

  public:
    Eigen::Isometry3d T_z;
    Eigen::Matrix3d R_z;
    Eigen::Vector3d t_z;
    Eigen::Isometry3d T_j_w;
    Eigen::Matrix3d R_w_j;
    Eigen::Vector3d t_w_j;
};

/******************* Object Factor *******************/

class ObjectFactor {
  public:
    ObjectFactor(const Eigen::Matrix3d &_R_o_j, const Eigen::VectorXd &_t_o_j) : R_o_j(_R_o_j), t_o_j(_t_o_j) {
        R_o_j = Eigen::Quaterniond(R_o_j).normalized().toRotationMatrix();
    }

    ObjectFactor(const Eigen::Isometry3d &_T_o_j) : T_o_j(_T_o_j) {
        R_o_j = Eigen::Quaterniond(T_o_j.rotation()).normalized().toRotationMatrix();
        //        R_o_j = T_o_j.rotation();
        t_o_j = T_o_j.translation();
    }

    ~ObjectFactor() = default;

    template <typename T>
    bool operator()(const T *const &R_w_j_ptr, const T *const &t_w_j_ptr, const T *const &R_o_w_ptr,
                    const T *const &t_o_w_ptr, T *residual_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j(R_w_j_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j(t_w_j_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_o_w(R_o_w_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_o_w(t_o_w_ptr);
        // residual equation
        Sophus::SO3<T> R_err_on_manifold(R_w_j.transpose() * R_o_w.transpose() * R_o_j);
        Eigen::Matrix<T, 3, 1> R_err = R_err_on_manifold.log();
        Eigen::Matrix<T, 3, 1> t_err = -R_w_j.transpose() * t_w_j - R_w_j.transpose() * R_o_w.transpose() * t_o_w +
                                       R_w_j.transpose() * R_o_w.transpose() * t_o_j;
        // bind residual to pointer
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residual_ptr);
        residuals_map.template head<3>() = R_err;
        residuals_map.template segment<3>(3) = t_err;
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix3d &_R_o_j, const Eigen::Vector3d &_t_o_j) {
        return (new ceres::AutoDiffCostFunction<ObjectFactor, 6, 9, 3, 9, 3>(new ObjectFactor(_R_o_j, _t_o_j)));
    }

    static ceres::CostFunction *Create(const Eigen::Isometry3d &_T_o_j) {
        return (new ceres::AutoDiffCostFunction<ObjectFactor, 6, 9, 3, 9, 3>(new ObjectFactor(_T_o_j)));
    }

  public:
    Eigen::Isometry3d T_o_j;
    Eigen::Matrix3d R_o_j;
    Eigen::VectorXd t_o_j;
};

class ObjectFixFactor {
  public:
    ObjectFixFactor(const Eigen::Matrix3d &_R_o_j, const Eigen::VectorXd &_t_o_j, const Eigen::Matrix3d &_R_o_w,
                    const Eigen::VectorXd &_t_o_w)
        : R_o_j(_R_o_j), t_o_j(_t_o_j), R_o_w(_R_o_w), t_o_w(_t_o_w) {
        R_o_j = Eigen::Quaterniond(R_o_j).normalized().toRotationMatrix();
        R_o_w = Eigen::Quaterniond(R_o_w).normalized().toRotationMatrix();
    }

    ObjectFixFactor(const Eigen::Isometry3d &_T_o_j, const Eigen::Isometry3d &_T_o_w) : T_o_j(_T_o_j), T_o_w(_T_o_w) {
        R_o_j = Eigen::Quaterniond(T_o_j.rotation()).normalized().toRotationMatrix();
        t_o_j = T_o_j.translation();
        R_o_w = Eigen::Quaterniond(T_o_w.rotation()).normalized().toRotationMatrix();
        t_o_w = T_o_w.translation();
        t_o_w = T_o_w.translation();
    }

    ~ObjectFixFactor() = default;

    template <typename T> bool operator()(const T *const &R_w_j_ptr, const T *const &t_w_j_ptr, T *residual_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j(R_w_j_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j(t_w_j_ptr);
        // residual equation
        Sophus::SO3<T> R_err_on_manifold(R_w_j.transpose() * R_o_w.transpose() * R_o_j);
        Eigen::Matrix<T, 3, 1> R_err = R_err_on_manifold.log();
        Eigen::Matrix<T, 3, 1> t_err = -R_w_j.transpose() * t_w_j - R_w_j.transpose() * R_o_w.transpose() * t_o_w +
                                       R_w_j.transpose() * R_o_w.transpose() * t_o_j;
        // bind residual to pointer
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residual_ptr);
        residuals_map.template head<3>() = R_err;
        residuals_map.template segment<3>(3) = t_err;
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix3d &_R_o_j, const Eigen::Vector3d &_t_o_j,
                                       const Eigen::Matrix3d &_R_o_w, const Eigen::Vector3d &_t_o_w) {
        return (new ceres::AutoDiffCostFunction<ObjectFixFactor, 6, 9, 3>(
            new ObjectFixFactor(_R_o_j, _t_o_j, _R_o_w, _t_o_w)));
    }

    static ceres::CostFunction *Create(const Eigen::Isometry3d &_T_o_j, const Eigen::Isometry3d &_T_o_w) {
        return (new ceres::AutoDiffCostFunction<ObjectFixFactor, 6, 9, 3>(new ObjectFixFactor(_T_o_j, _T_o_w)));
    }

  public:
    Eigen::Isometry3d T_o_j;
    Eigen::Matrix3d R_o_j;
    Eigen::VectorXd t_o_j;
    Eigen::Isometry3d T_o_w;
    Eigen::Matrix3d R_o_w;
    Eigen::Vector3d t_o_w;
};

/******************* Object to Object Factor *******************/
class ObjToObjFactor {
  public:
    ObjToObjFactor(const Eigen::Matrix3d &_R_o_j, const Eigen::Vector3d &_t_o_j, const Eigen::Matrix3d &_R_o_j_1,
                   const Eigen::Vector3d &_t_o_j_1)
        : R_o_j(_R_o_j), t_o_j(_t_o_j), R_o_j_1(_R_o_j_1), t_o_j_1(_t_o_j_1) {
        R_o_j = Eigen::Quaterniond(R_o_j).normalized().toRotationMatrix();
        R_o_j_1 = Eigen::Quaterniond(R_o_j_1).normalized().toRotationMatrix();
    }

    ObjToObjFactor(const Eigen::Isometry3d &_T_o_j, const Eigen::Isometry3d &_T_o_j_1)
        : T_o_j(_T_o_j), T_o_j_1(_T_o_j_1) {
        R_o_j = Eigen::Quaterniond(T_o_j.rotation()).normalized().toRotationMatrix();
        t_o_j = T_o_j.translation();
        R_o_j_1 = Eigen::Quaterniond(T_o_j_1.rotation()).normalized().toRotationMatrix();
        t_o_j_1 = T_o_j_1.translation();
    }

    ~ObjToObjFactor() = default;

    template <typename T>
    bool operator()(const T *const R_w_j_ptr, const T *const t_w_j_ptr, const T *const R_w_j_1_ptr,
                    const T *const t_w_j_1_ptr, T *residual_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j(R_w_j_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j(t_w_j_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j_1(R_w_j_1_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j_1(t_w_j_1_ptr);
        // residual equation
        Sophus::SO3<T> R_err_on_manifold(R_w_j.transpose() * R_w_j_1 * R_o_j_1.transpose() * R_o_j);
        Eigen::Matrix<T, 3, 1> R_err = R_err_on_manifold.log();
        Eigen::Matrix<T, 3, 1> t_err = -R_w_j.transpose() * t_w_j + R_w_j.transpose() * t_w_j_1 -
                                       R_w_j.transpose() * R_w_j_1 * R_o_j_1.transpose() * t_o_j_1 +
                                       R_w_j.transpose() * R_w_j_1 * R_o_j_1.transpose() * t_o_j;
        // bind residual to pointer
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residual_ptr);
        residuals_map.template head<3>() = R_err;
        residuals_map.template segment<3>(3) = t_err;
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix3d &_R_o_j, const Eigen::Vector3d &_t_o_j,
                                       const Eigen::Matrix3d &_R_o_j_1, const Eigen::Vector3d &_t_o_j_1) {
        return (new ceres::AutoDiffCostFunction<ObjToObjFactor, 6, 9, 3, 9, 3>(
            new ObjToObjFactor(_R_o_j, _t_o_j, _R_o_j_1, _t_o_j_1)));
    }

    static ceres::CostFunction *Create(const Eigen::Isometry3d &_T_o_j, const Eigen::Isometry3d &_T_o_j_1) {
        return (new ceres::AutoDiffCostFunction<ObjToObjFactor, 6, 9, 3, 9, 3>(new ObjToObjFactor(_T_o_j, _T_o_j_1)));
    }

  public:
    Eigen::Isometry3d T_o_j;
    Eigen::Matrix3d R_o_j;
    Eigen::VectorXd t_o_j;
    Eigen::Isometry3d T_o_j_1;
    Eigen::Matrix3d R_o_j_1;
    Eigen::VectorXd t_o_j_1;
};

/******************* Object Points to Object Points Factor *******************/

class ObjPointToObjPointFactor {
  public:
    ObjPointToObjPointFactor(const Eigen::Vector3d &_p_j, const Eigen::Vector3d &_p_j_1) : p_j(_p_j), p_j_1(_p_j_1) {}

    template <typename T>
    bool operator()(const T *const R_w_j_ptr, const T *const t_w_j_ptr, const T *const R_w_j_1_ptr,
                    const T *const t_w_j_1_ptr, T *residual_ptr) const {
        // map values
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j(R_w_j_ptr, 3, 3);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j(t_w_j_ptr, 3);
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j_1(R_w_j_1_ptr, 3, 3);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j_1(t_w_j_1_ptr, 3);
        // residual equation
        Eigen::Matrix<T, 3, 1> error =
            p_j_1 - (R_w_j_1.transpose() * R_w_j * p_j - R_w_j_1.transpose() * t_w_j_1 + R_w_j_1.transpose() * t_w_j);
        // map residual
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residual_map(residual_ptr, 3);
        residual_map = error;
        return true;
    }

    //    static ceres::CostFunction *Create(const Eigen::Vector3d _p_j, const Eigen::Vector3d
    //    _p_j_1) {
    //        return (
    //            new ceres::NumericDiffCostFunction<ObjPointToObjPointFactor, ceres::CENTRAL, 3, 9,
    //            3, 9,
    //                                               3>(new ObjPointToObjPointFactor(_p_j,
    //                                               _p_j_1)));
    //    }
    static ceres::CostFunction *Create(const Eigen::Vector3d _p_j, const Eigen::Vector3d _p_j_1) {
        return (new ceres::AutoDiffCostFunction<ObjPointToObjPointFactor, 3, 9, 3, 9, 3>(
            new ObjPointToObjPointFactor(_p_j, _p_j_1)));
    }

  public:
    Eigen::Matrix<double, 3, Eigen::Dynamic> p_j;
    Eigen::Matrix<double, 3, Eigen::Dynamic> p_j_1;
};

/******************* Spherical Harmonics Factor *******************/

class SphFixConformalPtoPFactor {
  public:
    SphFixConformalPtoPFactor(const Eigen::Isometry3d &_T_o_j_1, const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j,
                              const Eigen::VectorXd &_sph_coeff,
                              const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
                              const Eigen::Vector2d &_conformal_map_boundary, const double &_weight,
                              const int &_debug_obj_idx, const LOCAL_BA_CONFIG &_local_BA_config,
                              const int &_debug_frame_num)
        : T_o_j_1(_T_o_j_1), p_j(_p_j), sph_coeff(_sph_coeff), interp_func(_interp_func),
          conformal_map_boundary(_conformal_map_boundary), weight(_weight), debug_obj_idx(_debug_obj_idx),
          local_BA_config(_local_BA_config), debug_frame_num(_debug_frame_num) {
        R_o_j_1 = Eigen::Quaterniond(T_o_j_1.rotation()).normalized().toRotationMatrix();
        t_o_j_1 = T_o_j_1.translation();
    }

    SphFixConformalPtoPFactor(const Eigen::Matrix3d &_R_o_j_1, const Eigen::Vector3d &_t_o_j_1,
                              const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j, const Eigen::VectorXd &_sph_coeff,
                              const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
                              const Eigen::Vector2d &_conformal_map_boundary, const double &_weight)
        : R_o_j_1(_R_o_j_1), t_o_j_1(_t_o_j_1), p_j(_p_j), sph_coeff(_sph_coeff), interp_func(_interp_func),
          conformal_map_boundary(_conformal_map_boundary), weight(_weight) {
        R_o_j_1 = Eigen::Quaterniond(R_o_j_1).normalized().toRotationMatrix();
    }

    SphFixConformalPtoPFactor(const Eigen::Isometry3d &_T_o_j_1, const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j,
                              const Eigen::VectorXd &_sph_coeff,
                              const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
                              const Eigen::Vector2d &_conformal_map_boundary, const double &_weight)
        : T_o_j_1(_T_o_j_1), p_j(_p_j), sph_coeff(_sph_coeff), interp_func(_interp_func),
          conformal_map_boundary(_conformal_map_boundary), weight(_weight) {
        R_o_j_1 = Eigen::Quaterniond(T_o_j_1.rotation()).normalized().toRotationMatrix();
        t_o_j_1 = T_o_j_1.translation();
    }

    ~SphFixConformalPtoPFactor() = default;

    bool operator()(double const *const *para, double *residual_ptr) const {
        Eigen::Map<const Eigen::Matrix3d> R_w_j(para[0], 3, 3);
        Eigen::Map<const Eigen::Vector3d> t_w_j(para[1], 3);
        Eigen::Map<const Eigen::Matrix3d> R_w_j_1(para[2], 3, 3);
        Eigen::Map<const Eigen::Vector3d> t_w_j_1(para[3], 3);
        // residual equation
        // transfer points to last frame's coordinate object
        Eigen::Matrix3d R_o_j = R_o_j_1 * R_w_j_1.transpose() * R_w_j;
        Eigen::Vector3d t_o_j =
            t_o_j_1 - R_o_j_1 * R_w_j_1.transpose() * t_w_j_1 + R_o_j_1 * R_w_j_1.transpose() * t_w_j;
        Eigen::MatrixXd p_o = (R_o_j * p_j).colwise() + t_o_j;
        // calculate its 2d coordinate of conformal mapping
        p_o.transposeInPlace();
        double tmp_bdy[2] = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()};
        std::pair<std::vector<double>, std::vector<int>> x0 = linear_interpolation_2(p_o, interp_func.first, tmp_bdy);
        std::pair<std::vector<double>, std::vector<int>> x1 = linear_interpolation_2(p_o, interp_func.second, tmp_bdy);
        std::vector<double> phis;
        std::vector<double> thetas;
        std::vector<int> idx_vec;
        int counter = 0;
        for (auto iter0 = x0.first.begin(), iter1 = x1.first.begin(); iter0 != x0.first.end(); ++iter0, ++iter1) {
            if (((*iter0 >= 0) && *iter0 <= conformal_map_boundary(0)) &&
                ((*iter1 >= 0) && (*iter1 <= conformal_map_boundary(1)))) {
                phis.push_back(*iter0 / conformal_map_boundary(0) * 2 * M_PI * SH_scale + M_PI * (1 - SH_scale));
                thetas.push_back(*iter1 / conformal_map_boundary(1) * M_PI * SH_scale + M_PI_2 * (1 - SH_scale));
                idx_vec.push_back(counter);
            }
            ++counter;
        }
        Eigen::MatrixXd error(p_j.cols(), 3);
        error.setZero();
        Eigen::VectorXd p_recons = Eigen::VectorXd::Zero(p_j.cols(), 1) * weight;
        if (!idx_vec.empty()) {
            Eigen::MatrixXd sph_dirs(thetas.size(), 2);
            phis.insert(phis.end(), thetas.begin(), thetas.end());
            sph_dirs = Eigen::Map<Eigen::MatrixXd>(phis.data(), static_cast<long>(thetas.size()), 2);
            Eigen::VectorXd z_recons = curl::invLeastSquaresSHT(sph_coeff, sph_dirs);
            p_recons(idx_vec) = z_recons;
        }
        error.col(2) = (p_recons - p_o.col(2)) * (p_j.cols() / idx_vec.size());
        // bind residual to pointer
        Eigen::Map<Eigen::MatrixXd> residuals_map(residual_ptr, p_j.cols(), 3);
        residuals_map = error;
        if (local_BA_config.is_debug && (debug_iteration % 50) == 0) {
            Eigen::MatrixXd conformal_projection(x0.first.size(), 2);
            conformal_projection << Eigen::Map<Eigen::MatrixXd>(x0.first.data(), x0.first.size(), 1),
                Eigen::Map<Eigen::MatrixXd>(x1.first.data(), x1.first.size(), 1);
            std::string conformal_projection_file =
                local_BA_config.debug_2D_projection_dir + "/frame_" + std::to_string(debug_frame_num) + "_obj_" +
                std::to_string(debug_obj_idx) + "_iter_" + std::to_string(debug_iteration) + ".txt";
            FileReaderBase::write_txt_file(conformal_projection_file, conformal_projection);
            ++debug_iteration;
        }
        return true;
    }

    static ceres::DynamicNumericDiffCostFunction<SphFixConformalPtoPFactor> *
    Create(const Eigen::Matrix3d &_R_o_j_1, const Eigen::Vector3d &_t_o_j_1,
           const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j, const Eigen::VectorXd &_sph_coeff,
           const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
           const Eigen::Vector2d &_conformal_map_boundary, const double &_weight) {
        return (new ceres::DynamicNumericDiffCostFunction<SphFixConformalPtoPFactor, ceres::CENTRAL>(
            new SphFixConformalPtoPFactor(_R_o_j_1, _t_o_j_1, _p_j, _sph_coeff, _interp_func, _conformal_map_boundary,
                                          _weight)));
    }

    static ceres::DynamicNumericDiffCostFunction<SphFixConformalPtoPFactor> *
    Create(const Eigen::Isometry3d &_T_o_j_1, const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j,
           const Eigen::VectorXd &_sph_coeff, const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
           const Eigen::Vector2d &_conformal_map_boundary, const double &_weight) {
        return (new ceres::DynamicNumericDiffCostFunction<SphFixConformalPtoPFactor, ceres::CENTRAL>(
            new SphFixConformalPtoPFactor(_T_o_j_1, _p_j, _sph_coeff, _interp_func, _conformal_map_boundary, _weight)));
    }

    static ceres::DynamicNumericDiffCostFunction<SphFixConformalPtoPFactor> *
    Create(const Eigen::Isometry3d &_T_o_j_1, const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j,
           const Eigen::VectorXd &_sph_coeff, const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
           const Eigen::Vector2d &_conformal_map_boundary, const double &_weight, const int &_debug_obj_idx,
           const LOCAL_BA_CONFIG &_local_BA_config, const int &_debug_frame_num) {
        return (new ceres::DynamicNumericDiffCostFunction<SphFixConformalPtoPFactor, ceres::CENTRAL>(
            new SphFixConformalPtoPFactor(_T_o_j_1, _p_j, _sph_coeff, _interp_func, _conformal_map_boundary, _weight,
                                          _debug_obj_idx, _local_BA_config, _debug_frame_num)));
    }

  public:
    Eigen::Isometry3d T_o_j_1;
    Eigen::Matrix3d R_o_j_1;
    Eigen::Vector3d t_o_j_1;
    Eigen::Matrix<double, 3, Eigen::Dynamic> p_j;
    Eigen::VectorXd sph_coeff;
    std::pair<interp_func_pair, interp_func_pair> interp_func;
    Eigen::Vector2d conformal_map_boundary;
    double SH_scale = 0.8;
    double weight;
    int debug_obj_idx;
    mutable int debug_iteration = 0;
    int debug_frame_num;
    LOCAL_BA_CONFIG local_BA_config;
};

// sph point to map
class SphFixConformalPtoMFactor {
  public:
    SphFixConformalPtoMFactor(const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j, const Eigen::VectorXd &_sph_coeff,
                              const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
                              const Eigen::Vector2d &_conformal_map_boundary, const double &_weight,
                              const int &_debug_obj_idx, const LOCAL_BA_CONFIG &_local_BA_config,
                              const int &_debug_frame_num)
        : p_j(_p_j), sph_coeff(_sph_coeff), interp_func(_interp_func), conformal_map_boundary(_conformal_map_boundary),
          weight(_weight), debug_obj_idx(_debug_obj_idx), local_BA_config(_local_BA_config),
          debug_frame_num(_debug_frame_num) {}

    SphFixConformalPtoMFactor(const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j, const Eigen::VectorXd &_sph_coeff,
                              const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
                              const Eigen::Vector2d &_conformal_map_boundary, const double &_weight)
        : p_j(_p_j), sph_coeff(_sph_coeff), interp_func(_interp_func), conformal_map_boundary(_conformal_map_boundary),
          weight(_weight) {}

    ~SphFixConformalPtoMFactor() = default;

    bool operator()(double const *const *para, double *residual_ptr) const {
        Eigen::Map<const Eigen::Matrix3d> R_w_j(para[0], 3, 3);
        Eigen::Map<const Eigen::Vector3d> t_w_j(para[1], 3);
        Eigen::Map<const Eigen::Matrix3d> R_o_w(para[2], 3, 3);
        Eigen::Map<const Eigen::Vector3d> t_o_w(para[3], 3);
        // residual equation
        // transfer points to last frame's coordinate object
        Eigen::Matrix3d R_o_j = R_o_w * R_w_j;
        Eigen::Vector3d t_o_j = t_o_w + R_o_w * t_w_j;
        Eigen::MatrixXd p_o = (R_o_j * p_j).colwise() + t_o_j;
        // calculate its 2d coordinate of conformal mapping
        p_o.transposeInPlace();
        double tmp_bdy[2] = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()};
        std::pair<std::vector<double>, std::vector<int>> x0 = linear_interpolation_2(p_o, interp_func.first, tmp_bdy);
        std::pair<std::vector<double>, std::vector<int>> x1 = linear_interpolation_2(p_o, interp_func.second, tmp_bdy);
        std::vector<double> phis;
        std::vector<double> thetas;
        std::vector<int> idx_vec;
        int counter = 0;
        for (auto iter0 = x0.first.begin(), iter1 = x1.first.begin(); iter0 != x0.first.end(); ++iter0, ++iter1) {
            if (((*iter0 >= 0) && *iter0 <= conformal_map_boundary(0)) &&
                ((*iter1 >= 0) && (*iter1 <= conformal_map_boundary(1)))) {
                phis.push_back(*iter0 / conformal_map_boundary(0) * 2 * M_PI * SH_scale + M_PI * (1 - SH_scale));
                thetas.push_back(*iter1 / conformal_map_boundary(1) * M_PI * SH_scale + M_PI_2 * (1 - SH_scale));
                idx_vec.push_back(counter);
            }
            ++counter;
        }
        Eigen::VectorXd error(p_j.cols(), 1);
        error.setZero();
        Eigen::VectorXd p_recons = Eigen::VectorXd::Zero(p_j.cols(), 1) * weight;
        if (!idx_vec.empty()) {
            Eigen::MatrixXd sph_dirs(thetas.size(), 2);
            phis.insert(phis.end(), thetas.begin(), thetas.end());
            sph_dirs = Eigen::Map<Eigen::MatrixXd>(phis.data(), static_cast<long>(thetas.size()), 2);
            Eigen::VectorXd z_recons = curl::invLeastSquaresSHT(sph_coeff, sph_dirs);
            p_recons(idx_vec) = z_recons;
        }
        error = (p_recons - p_o.col(2)) * (p_j.cols() / idx_vec.size());
        // bind residual to pointer
        Eigen::Map<Eigen::VectorXd> residuals_map(residual_ptr, p_j.cols(), 3);
        residuals_map = error;
        if (local_BA_config.is_debug) {
            Eigen::MatrixXd conformal_projection(x0.first.size(), 2);
            conformal_projection << Eigen::Map<Eigen::MatrixXd>(x0.first.data(), x0.first.size(), 1),
                Eigen::Map<Eigen::MatrixXd>(x1.first.data(), x1.first.size(), 1);
            std::string conformal_projection_file =
                local_BA_config.debug_2D_projection_dir + "/frame_" + std::to_string(debug_frame_num) + "_obj_" +
                std::to_string(debug_obj_idx) + "_iter_" + std::to_string(debug_iteration) + ".txt";
            FileReaderBase::write_txt_file(conformal_projection_file, conformal_projection);
            ++debug_iteration;
        }
        return true;
    }

    static ceres::DynamicNumericDiffCostFunction<SphFixConformalPtoMFactor> *
    Create(const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j, const Eigen::VectorXd &_sph_coeff,
           const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
           const Eigen::Vector2d &_conformal_map_boundary, const double &_weight) {
        return (new ceres::DynamicNumericDiffCostFunction<SphFixConformalPtoMFactor, ceres::CENTRAL>(
            new SphFixConformalPtoMFactor(_p_j, _sph_coeff, _interp_func, _conformal_map_boundary, _weight)));
    }

    static ceres::DynamicNumericDiffCostFunction<SphFixConformalPtoMFactor> *
    Create(const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j, const Eigen::VectorXd &_sph_coeff,
           const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
           const Eigen::Vector2d &_conformal_map_boundary, const double &_weight, const int &_debug_obj_idx,
           const LOCAL_BA_CONFIG &_local_BA_config, const int &_debug_frame_num) {
        return (new ceres::DynamicNumericDiffCostFunction<SphFixConformalPtoMFactor, ceres::CENTRAL>(
            new SphFixConformalPtoMFactor(_p_j, _sph_coeff, _interp_func, _conformal_map_boundary, _weight,
                                          _debug_obj_idx, _local_BA_config, _debug_frame_num)));
    }

  public:
    Eigen::Matrix<double, 3, Eigen::Dynamic> p_j;
    Eigen::VectorXd sph_coeff;
    std::pair<interp_func_pair, interp_func_pair> interp_func;
    Eigen::Vector2d conformal_map_boundary;
    double SH_scale = 0.8;
    double weight;
    int debug_obj_idx;
    mutable int debug_iteration = 0;
    int debug_frame_num;
    LOCAL_BA_CONFIG local_BA_config;
};

// sph point to map
class SphConformalPtoMFactor {
  public:
    SphConformalPtoMFactor(const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j,
                           const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
                           const Eigen::Vector2d &_conformal_map_boundary, const double &_weight,
                           const int &_debug_obj_idx, const LOCAL_BA_CONFIG &_local_BA_config,
                           const int &_debug_frame_num)
        : p_j(_p_j), interp_func(_interp_func), conformal_map_boundary(_conformal_map_boundary), weight(_weight),
          debug_obj_idx(_debug_obj_idx), local_BA_config(_local_BA_config), debug_frame_num(_debug_frame_num) {}

    SphConformalPtoMFactor(const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j,
                           const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
                           const Eigen::Vector2d &_conformal_map_boundary, const double &_weight)
        : p_j(_p_j), interp_func(_interp_func), conformal_map_boundary(_conformal_map_boundary), weight(_weight) {}

    ~SphConformalPtoMFactor() = default;

    bool operator()(double const *const *para, double *residual_ptr) const {
        Eigen::Map<const Eigen::Matrix3d> R_w_j(para[0], 3, 3);
        Eigen::Map<const Eigen::Vector3d> t_w_j(para[1], 3);
        Eigen::Map<const Eigen::Matrix3d> R_o_w(para[2], 3, 3);
        Eigen::Map<const Eigen::Vector3d> t_o_w(para[3], 3);
        Eigen::Map<const Eigen::VectorXd> sph_coeff(para[4], 121);
        // residual equation
        // transfer points to last frame's coordinate object
        Eigen::Matrix3d R_o_j = R_o_w * R_w_j;
        Eigen::Vector3d t_o_j = t_o_w + R_o_w * t_w_j;
        Eigen::MatrixXd p_o = (R_o_j * p_j).colwise() + t_o_j;
        // calculate its 2d coordinate of conformal mapping
        p_o.transposeInPlace();
        double tmp_bdy[2] = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()};
        std::pair<std::vector<double>, std::vector<int>> x0 = linear_interpolation_2(p_o, interp_func.first, tmp_bdy);
        std::pair<std::vector<double>, std::vector<int>> x1 = linear_interpolation_2(p_o, interp_func.second, tmp_bdy);
        std::vector<double> phis;
        std::vector<double> thetas;
        std::vector<int> idx_vec;
        int counter = 0;
        for (auto iter0 = x0.first.begin(), iter1 = x1.first.begin(); iter0 != x0.first.end(); ++iter0, ++iter1) {
            if (((*iter0 >= 0) && *iter0 <= conformal_map_boundary(0)) &&
                ((*iter1 >= 0) && (*iter1 <= conformal_map_boundary(1)))) {
                phis.push_back(*iter0 / conformal_map_boundary(0) * 2 * M_PI * SH_scale + M_PI * (1 - SH_scale));
                thetas.push_back(*iter1 / conformal_map_boundary(1) * M_PI * SH_scale + M_PI_2 * (1 - SH_scale));
                idx_vec.push_back(counter);
            }
            ++counter;
        }
        Eigen::VectorXd error(p_j.cols(), 1);
        error.setZero();
        Eigen::VectorXd p_recons = Eigen::VectorXd::Zero(p_j.cols(), 1) * weight;
        if (!idx_vec.empty()) {
            Eigen::MatrixXd sph_dirs(thetas.size(), 2);
            phis.insert(phis.end(), thetas.begin(), thetas.end());
            sph_dirs = Eigen::Map<Eigen::MatrixXd>(phis.data(), static_cast<long>(thetas.size()), 2);
            Eigen::VectorXd z_recons = curl::invLeastSquaresSHT<double>(sph_coeff, sph_dirs);
            p_recons(idx_vec) = z_recons;
        }
        error = (p_recons - p_o.col(2)) * (p_j.cols() / idx_vec.size());
        // bind residual to pointer
        Eigen::Map<Eigen::VectorXd> residuals_map(residual_ptr, p_j.cols(), 3);
        residuals_map = error;
        if (local_BA_config.is_debug) {
            Eigen::MatrixXd conformal_projection(x0.first.size(), 2);
            conformal_projection << Eigen::Map<Eigen::MatrixXd>(x0.first.data(), x0.first.size(), 1),
                Eigen::Map<Eigen::MatrixXd>(x1.first.data(), x1.first.size(), 1);
            std::string conformal_projection_file =
                local_BA_config.debug_2D_projection_dir + "/frame_" + std::to_string(debug_frame_num) + "_obj_" +
                std::to_string(debug_obj_idx) + "_iter_" + std::to_string(debug_iteration) + ".txt";
            FileReaderBase::write_txt_file(conformal_projection_file, conformal_projection);
            ++debug_iteration;
        }
        return true;
    }

    static ceres::DynamicNumericDiffCostFunction<SphConformalPtoMFactor> *
    Create(const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j,
           const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
           const Eigen::Vector2d &_conformal_map_boundary, const double &_weight) {
        return (new ceres::DynamicNumericDiffCostFunction<SphConformalPtoMFactor, ceres::CENTRAL>(
            new SphConformalPtoMFactor(_p_j, _interp_func, _conformal_map_boundary, _weight)));
    }

    static ceres::DynamicNumericDiffCostFunction<SphConformalPtoMFactor> *
    Create(const Eigen::Matrix<double, 3, Eigen::Dynamic> &_p_j,
           const std::pair<interp_func_pair, interp_func_pair> &_interp_func,
           const Eigen::Vector2d &_conformal_map_boundary, const double &_weight, const int &_debug_obj_idx,
           const LOCAL_BA_CONFIG &_local_BA_config, const int &_debug_frame_num) {
        return (new ceres::DynamicNumericDiffCostFunction<SphConformalPtoMFactor, ceres::CENTRAL>(
            new SphConformalPtoMFactor(_p_j, _interp_func, _conformal_map_boundary, _weight, _debug_obj_idx,
                                       _local_BA_config, _debug_frame_num)));
    }

  public:
    Eigen::Matrix<double, 3, Eigen::Dynamic> p_j;
    std::pair<interp_func_pair, interp_func_pair> interp_func;
    Eigen::Vector2d conformal_map_boundary;
    double SH_scale = 0.8;
    double weight;
    int debug_obj_idx;
    mutable int debug_iteration = 0;
    int debug_frame_num;
    LOCAL_BA_CONFIG local_BA_config;
};

/***************** Spherical Harmonics Point to Map Factor ******************/

class SphFixPointToMapFactor {
  public:
    SphFixPointToMapFactor(const Eigen::Vector3d &_p_j, const Eigen::Vector3d &_p_o) : p_j(_p_j), p_o(_p_o) {}

    template <typename T>
    bool operator()(const T *const R_w_j_ptr, const T *const t_w_j_ptr, const T *const R_o_w_ptr,
                    const T *const t_o_w_ptr, T *residual_ptr) const {
        // map values
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j(R_w_j_ptr, 3, 3);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j(t_w_j_ptr, 3);
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_o_w(R_o_w_ptr, 3, 3);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_o_w(t_o_w_ptr, 3);
        // residual equation
        // transfer points to last frame's coordinate object
        Eigen::Matrix<T, 3, 3> R_o_j = R_o_w * R_w_j;
        Eigen::Matrix<T, 3, 1> t_o_j = t_o_w + R_o_w * t_w_j;
        Eigen::Matrix<T, 3, 1> p_o_transformed = (R_o_j * p_j) + t_o_j;

        // residual equation
        Eigen::Matrix<T, 3, 1> error = p_o - p_o_transformed;
        // map residual
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residual_map(residual_ptr, 3);
        residual_map = error;
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &_p_j, const Eigen::Vector3d &_p_o) {
        return (new ceres::AutoDiffCostFunction<SphFixPointToMapFactor, 3, 9, 3, 9, 3>(
            new SphFixPointToMapFactor(_p_j, _p_o)));
    }

  public:
    Eigen::Vector3d p_j;
    Eigen::Vector3d p_o;
};

class VisCallback : public ceres::IterationCallback {
  public:
    VisCallback(int _start_frame, int _end_frame, int _obj_num, RosHandler *_ros_handler_ptr,
                std::vector<Eigen::Matrix3d> *_R_w_j_vec_ptr, std::vector<Eigen::Vector3d> *_t_w_j_vec_ptr,
                std::vector<Eigen::Matrix3d> *_R_o_w_vec_ptr, std::vector<Eigen::Vector3d> *_t_o_w_vec_ptr,
                std::vector<std::vector<Eigen::MatrixXd>> *_points_j_valid_vec_ptr,
                std::vector<std::vector<std::vector<int>>> *_overlap_idx_ptr,
                std::vector<std::vector<Eigen::MatrixXd>> *_points_j_transformed_recons_vec_ptr)
        : start_frame(_start_frame), end_frame(_end_frame), obj_num(_obj_num), ros_handler_ptr(_ros_handler_ptr),
          R_w_j_vec_ptr(_R_w_j_vec_ptr), t_w_j_vec_ptr(_t_w_j_vec_ptr), R_o_w_vec_ptr(_R_o_w_vec_ptr),
          t_o_w_vec_ptr(_t_o_w_vec_ptr), points_j_valid_vec_ptr(_points_j_valid_vec_ptr),
          overlap_idx_ptr(_overlap_idx_ptr), points_j_transformed_recons_vec_ptr(_points_j_transformed_recons_vec_ptr) {
    }

    ~VisCallback() override = default;
    ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override {
        RosHandler::plot_rviz("real_time", start_frame, end_frame, obj_num, ros_handler_ptr, R_w_j_vec_ptr,
                              t_w_j_vec_ptr, R_o_w_vec_ptr, t_o_w_vec_ptr, points_j_valid_vec_ptr, overlap_idx_ptr,
                              points_j_transformed_recons_vec_ptr);
        if (summary.cost_change < 0) {
            return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
        }
        return ceres::SOLVER_CONTINUE;
    }

  private:
    int start_frame;
    int end_frame;
    int obj_num;
    RosHandler *ros_handler_ptr;
    std::vector<Eigen::Matrix3d> *R_w_j_vec_ptr;
    std::vector<Eigen::Vector3d> *t_w_j_vec_ptr;
    std::vector<Eigen::Matrix3d> *R_o_w_vec_ptr;
    std::vector<Eigen::Vector3d> *t_o_w_vec_ptr;
    std::vector<std::vector<Eigen::MatrixXd>> *points_j_valid_vec_ptr;
    std::vector<std::vector<std::vector<int>>> *overlap_idx_ptr;
    std::vector<std::vector<Eigen::MatrixXd>> *points_j_transformed_recons_vec_ptr;
};

class StopCallback : public ceres::IterationCallback {
  public:
    StopCallback() = default;
    ~StopCallback() override = default;
    ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override {
        if (summary.cost_change < 0) {
            return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
        }
        return ceres::SOLVER_CONTINUE;
    }
};

class SphFixPointToMapFixFactor {
  public:
    SphFixPointToMapFixFactor(const Eigen::Isometry3d &_T_o_w, const Eigen::Vector3d &_p_j, const Eigen::Vector3d &_p_o)
        : T_o_w(_T_o_w), p_j(_p_j), p_o(_p_o) {
        R_o_w = Eigen::Quaterniond(T_o_w.rotation()).normalized().toRotationMatrix();
        t_o_w = T_o_w.translation();
    }

    SphFixPointToMapFixFactor(const Eigen::Matrix3d &_R_o_w, const Eigen::Vector3d &_t_o_w, const Eigen::Vector3d &_p_j,
                              const Eigen::Vector3d &_p_o)
        : R_o_w(_R_o_w), t_o_w(_t_o_w), p_j(_p_j), p_o(_p_o) {
        R_o_w = Eigen::Quaterniond(R_o_w).normalized().toRotationMatrix();
    }

    template <typename T> bool operator()(const T *const R_w_j_ptr, const T *const t_w_j_ptr, T *residual_ptr) const {
        // map values
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> R_w_j(R_w_j_ptr, 3, 3);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_w_j(t_w_j_ptr, 3);
        // residual equation
        // transfer points to last frame's coordinate object
        Eigen::Matrix<T, 3, 3> R_o_j = R_o_w * R_w_j;
        Eigen::Matrix<T, 3, 1> t_o_j = R_o_w * t_w_j + t_o_w;
        Eigen::Matrix<T, 3, 1> p_o_transformed = (R_o_j * p_j) + t_o_j;

        // residual equation
        Eigen::Matrix<T, 3, 1> error = p_o - p_o_transformed;
        // map residual
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residual_map(residual_ptr, 3);
        residual_map = error;
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Isometry3d &_T_o_w, const Eigen::Vector3d &_p_j,
                                       const Eigen::Vector3d &_p_o) {
        return (new ceres::AutoDiffCostFunction<SphFixPointToMapFixFactor, 3, 9, 3>(
            new SphFixPointToMapFixFactor(_T_o_w, _p_j, _p_o)));
    }

    static ceres::CostFunction *Create(const Eigen::Matrix3d &_R_o_w, const Eigen::Vector3d &_t_o_w,
                                       const Eigen::Vector3d &_p_j, const Eigen::Vector3d &_p_o) {
        return (new ceres::AutoDiffCostFunction<SphFixPointToMapFixFactor, 3, 9, 3>(
            new SphFixPointToMapFixFactor(_R_o_w, _t_o_w, _p_j, _p_o)));
    }

  private:
    Eigen::Isometry3d T_o_w;
    Eigen::Matrix3d R_o_w;
    Eigen::Vector3d t_o_w;
    Eigen::Vector3d p_j;
    Eigen::Vector3d p_o;
};

class CoupleFixSphFactorTangent : public ceres::SizedCostFunction<1, 6> {
  public:
    CoupleFixSphFactorTangent(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w, const Eigen::Vector3d &_p_j,
                              const PatchProcession<double> *_patch_processtion_ptr)
        : T_o_w(_T_o_w), p_j(_p_j), patch_procession_ptr(_patch_processtion_ptr) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2));
        T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2)) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
        invalid_res = patch_procession_ptr->T_INVALID;
        weight = 1;
    }

    CoupleFixSphFactorTangent(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w, const Eigen::Vector3d &_p_j,
                              const PatchProcession<double> *_patch_processtion_ptr, double _invalid_res)
        : T_o_w(_T_o_w), p_j(_p_j), patch_procession_ptr(_patch_processtion_ptr), invalid_res(_invalid_res) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2));
        T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2)) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
        weight = 1;
    }

    CoupleFixSphFactorTangent(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w, const Eigen::Vector3d &_p_j,
                              const PatchProcession<double> *_patch_processtion_ptr, double _invalid_res,
                              double _weight)
        : T_o_w(_T_o_w), p_j(_p_j), patch_procession_ptr(_patch_processtion_ptr), invalid_res(_invalid_res),
          weight(_weight) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2));
        T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2)) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
    }

    ~CoupleFixSphFactorTangent() override {}
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> T_w_j_tangent(parameters[0]);
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_w_j = Sophus::SE3<double>::exp(T_w_j_tangent).matrix();
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_o_j = T_o_w * T_w_j;
        Eigen::Vector3d p_o = T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2)) * p_j + T_o_j(Eigen::seq(0, 2), 3);
        double x = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x, p_o(0), p_o(1));
        //        double x = patch_procession_ptr->fetch_Gamma_x_interp(p_o(0), p_o(1));
        bool is_fault = false;
        if (x == patch_procession_ptr->T_INVALID) {
            is_fault = true;
        }
        double y = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y, p_o(0), p_o(1));
        //        double y = patch_procession_ptr->fetch_Gamma_y_interp(p_o(0), p_o(1));
        if (y == patch_procession_ptr->T_INVALID) {
            is_fault = true;
        }
        patch_procession_ptr->I_lock.lock();
        double I = patch_procession_ptr->fetch_I(patch_procession_ptr->I, x, y);
        patch_procession_ptr->I_lock.unlock();
        //        if (I != patch_procession_ptr->T_INVALID) {
        //            std::cerr << "redidual: " << I << std::endl;
        //        }
        //        double I = patch_procession_ptr->fetch_I_interp(x, y);
        if (I == patch_procession_ptr->T_INVALID || I > invalid_res) {
            is_fault = true;
        }
        if (!is_fault) {
            residuals[0] = weight * (p_o(2) - I);
        } else {
            residuals[0] = 0;
        }

        // Compute the Jacobian if asked for.
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            // Map jacobians
            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> Jac(jacobians[0], 1, 6);
            Jac.setZero();
            if (is_fault) {
                return true;
            }
            // Z_jac
            Eigen::Matrix<double, 1, 3, Eigen::RowMajor> Z_Jac;
            Z_Jac << 0, 0, 1;
            // SE3_Jac
            Eigen::Matrix<double, 3, 6> SE3_Jac;
            SE3_Jac(Eigen::seq(0, 2), Eigen::seq(0, 2)) = T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2));
            SE3_Jac(Eigen::seq(0, 2), Eigen::seq(3, 5)) =
                -T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2)) * curl::skew<double>(p_j);
            // I_Jac
            Eigen::Matrix<double, 1, 2, Eigen::RowMajor> I_Jac;
            patch_procession_ptr->I_lock.lock();
            I_Jac(0, 0) = patch_procession_ptr->fetch_I(patch_procession_ptr->I_Gx, x, y);
            patch_procession_ptr->I_lock.unlock();
            //            I_Jac(0, 0) = patch_procession_ptr->fetch_I_Gx_interp(x, y);
            if (I_Jac(0, 0) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            patch_procession_ptr->I_lock.lock();
            I_Jac(0, 1) = patch_procession_ptr->fetch_I(patch_procession_ptr->I_Gy, x, y);
            patch_procession_ptr->I_lock.unlock();
            //            I_Jac(0, 1) = patch_procession_ptr->fetch_I_Gy_interp(x, y);
            if (I_Jac(0, 1) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            // Gamma_Jac
            Eigen::Matrix<double, 2, 2, Eigen::RowMajor> Gamma_Jac;
            Gamma_Jac(0, 0) = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x_Gx, p_o(0), p_o(1));
            //            Gamma_Jac(0, 0) = patch_procession_ptr->fetch_Gamma_x_Gx_interp(p_o(0),
            //            p_o(1));
            if (Gamma_Jac(0, 0) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            Gamma_Jac(0, 1) = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x_Gy, p_o(0), p_o(1));
            //            Gamma_Jac(0, 1) = patch_procession_ptr->fetch_Gamma_x_Gy_interp(p_o(0),
            //            p_o(1));
            if (Gamma_Jac(0, 1) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            Gamma_Jac(1, 0) = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y_Gx, p_o(0), p_o(1));
            //            Gamma_Jac(1, 0) = patch_procession_ptr->fetch_Gamma_y_Gx_interp(p_o(0),
            //            p_o(1));
            if (Gamma_Jac(1, 0) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            Gamma_Jac(1, 1) = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y_Gy, p_o(0), p_o(1));
            //            Gamma_Jac(1, 1) = patch_procession_ptr->fetch_Gamma_y_Gy_interp(p_o(0),
            //            p_o(1));
            if (Gamma_Jac(1, 1) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            // XY_Jac
            Eigen::Matrix<double, 2, 3, Eigen::RowMajor> XY_Jac;
            XY_Jac << 1, 0, 0, 0, 1, 0;
            // Jac equation
            Jac = weight * (Z_Jac * SE3_Jac - I_Jac * Gamma_Jac * XY_Jac * SE3_Jac);
        }
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w,
                                       const Eigen::Vector3d &_p_j,
                                       const PatchProcession<double> *_patch_processtion_ptr) {
        return (new CoupleFixSphFactorTangent(_T_o_w, _p_j, _patch_processtion_ptr));
    }

    static ceres::CostFunction *Create(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w,
                                       const Eigen::Vector3d &_p_j,
                                       const PatchProcession<double> *_patch_processtion_ptr, double _invalid_res) {
        return (new CoupleFixSphFactorTangent(_T_o_w, _p_j, _patch_processtion_ptr, _invalid_res));
    }

    static ceres::CostFunction *Create(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w,
                                       const Eigen::Vector3d &_p_j,
                                       const PatchProcession<double> *_patch_processtion_ptr, double _invalid_res,
                                       double _weight) {
        return (new CoupleFixSphFactorTangent(_T_o_w, _p_j, _patch_processtion_ptr, _invalid_res, _weight));
    }

  private:
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_o_w;
    Eigen::Vector3d p_j;
    const PatchProcession<double> *patch_procession_ptr;
    double invalid_res;
    double weight;
};

template <typename T> class CoupleFixSphFactor : public ceres::SizedCostFunction<1, 16> {
  public:
    CoupleFixSphFactor(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w, const Eigen::Vector3d &_p_j,
                       const PatchProcession<T> *_patch_processtion_ptr)
        : T_o_w(_T_o_w), p_j(_p_j), patch_procession_ptr(_patch_processtion_ptr) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2));
        T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2)) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
        invalid_res = patch_procession_ptr->T_INVALID;
        weight = 1;
    }

    CoupleFixSphFactor(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w, const Eigen::Vector3d &_p_j,
                       const PatchProcession<T> *_patch_processtion_ptr, double _invalid_res)
        : T_o_w(_T_o_w), p_j(_p_j), patch_procession_ptr(_patch_processtion_ptr), invalid_res(_invalid_res) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2));
        T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2)) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
        weight = 1;
    }

    CoupleFixSphFactor(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w, const Eigen::Vector3d &_p_j,
                       const PatchProcession<T> *_patch_processtion_ptr, double _invalid_res, double _weight)
        : T_o_w(_T_o_w), p_j(_p_j), patch_procession_ptr(_patch_processtion_ptr), invalid_res(_invalid_res),
          weight(_weight) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2));
        T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2)) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
    }

    ~CoupleFixSphFactor() override {}
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_w_j(parameters[0]);
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_o_j = T_o_w * T_w_j;
        Eigen::Vector3d p_o = T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2)) * p_j + T_o_j(Eigen::seq(0, 2), 3);
        double x = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x, p_o(0), p_o(1));
        //        double x = patch_procession_ptr->fetch_Gamma_x_interp(p_o(0), p_o(1));
        bool is_fault = false;
        if (x == patch_procession_ptr->T_INVALID) {
            is_fault = true;
        }
        double y = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y, p_o(0), p_o(1));
        //        double y = patch_procession_ptr->fetch_Gamma_y_interp(p_o(0), p_o(1));
        if (y == patch_procession_ptr->T_INVALID) {
            is_fault = true;
        }
        patch_procession_ptr->I_lock.lock();
        double I = patch_procession_ptr->fetch_I(patch_procession_ptr->I, x, y);
        patch_procession_ptr->I_lock.unlock();
        //        double I = patch_procession_ptr->fetch_I_interp(x, y);
        //        if (I != patch_procession_ptr->T_INVALID) {
        //            std::cerr << "redidual: " << I << std::endl;
        //        }
        if (I == patch_procession_ptr->T_INVALID || I > invalid_res) {
            is_fault = true;
        }
        if (!is_fault) {
            residuals[0] = weight * (p_o(2) - I);
        } else {
            residuals[0] = 0;
        }

        // Compute the Jacobian if asked for.
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            // Map jacobians
            Eigen::Map<Eigen::Matrix<double, 1, 16, Eigen::RowMajor>> Jac(jacobians[0], 1, 16);
            Jac.setZero();
            if (is_fault) {
                return true;
            }
            // Z_jac
            Eigen::Matrix<double, 1, 3, Eigen::RowMajor> Z_Jac;
            Z_Jac << 0, 0, 1;
            // SE3_Jac
            Eigen::Matrix<double, 3, 6> SE3_Jac;
            SE3_Jac(Eigen::seq(0, 2), Eigen::seq(0, 2)) = T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2));
            SE3_Jac(Eigen::seq(0, 2), Eigen::seq(3, 5)) =
                -T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2)) * curl::skew<double>(p_j);
            // I_Jac
            Eigen::Matrix<double, 1, 2, Eigen::RowMajor> I_Jac;
            patch_procession_ptr->I_lock.lock();
            I_Jac(0, 0) = patch_procession_ptr->fetch_I(patch_procession_ptr->I_Gx, x, y);
            patch_procession_ptr->I_lock.unlock();
            //            I_Jac(0, 0) = patch_procession_ptr->fetch_I_Gx_interp(x, y);
            if (I_Jac(0, 0) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            patch_procession_ptr->I_lock.lock();
            I_Jac(0, 1) = patch_procession_ptr->fetch_I(patch_procession_ptr->I_Gy, x, y);
            patch_procession_ptr->I_lock.unlock();
            //            I_Jac(0, 1) = patch_procession_ptr->fetch_I_Gy_interp(x, y);
            if (I_Jac(0, 1) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            // Gamma_Jac
            Eigen::Matrix<double, 2, 2, Eigen::RowMajor> Gamma_Jac;
            Gamma_Jac(0, 0) = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x_Gx, p_o(0), p_o(1));
            //            Gamma_Jac(0, 0) = patch_procession_ptr->fetch_Gamma_x_Gx_interp(p_o(0),
            //            p_o(1));
            if (Gamma_Jac(0, 0) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            Gamma_Jac(0, 1) = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x_Gy, p_o(0), p_o(1));
            //            Gamma_Jac(0, 1) = patch_procession_ptr->fetch_Gamma_x_Gy_interp(p_o(0),
            //            p_o(1));
            if (Gamma_Jac(0, 1) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            Gamma_Jac(1, 0) = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y_Gx, p_o(0), p_o(1));
            //            Gamma_Jac(1, 0) = patch_procession_ptr->fetch_Gamma_y_Gx_interp(p_o(0),
            //            p_o(1));
            if (Gamma_Jac(1, 0) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            Gamma_Jac(1, 1) = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y_Gy, p_o(0), p_o(1));
            //            Gamma_Jac(1, 1) = patch_procession_ptr->fetch_Gamma_y_Gy_interp(p_o(0),
            //            p_o(1));
            if (Gamma_Jac(1, 1) == patch_procession_ptr->T_INVALID) {
                return true;
            }
            // XY_Jac
            Eigen::Matrix<double, 2, 3, Eigen::RowMajor> XY_Jac;
            XY_Jac << 1, 0, 0, 0, 1, 0;
            // Jac equation
            Jac(0, Eigen::seq(0, 5)) = weight * (Z_Jac * SE3_Jac - I_Jac * Gamma_Jac * XY_Jac * SE3_Jac);
        }
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w,
                                       const Eigen::Vector3d &_p_j, const PatchProcession<T> *_patch_processtion_ptr) {
        return (new CoupleFixSphFactor(_T_o_w, _p_j, _patch_processtion_ptr));
    }

    static ceres::CostFunction *Create(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w,
                                       const Eigen::Vector3d &_p_j, const PatchProcession<T> *_patch_processtion_ptr,
                                       double _invalid_res) {
        return (new CoupleFixSphFactor(_T_o_w, _p_j, _patch_processtion_ptr, _invalid_res));
    }

    static ceres::CostFunction *Create(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w,
                                       const Eigen::Vector3d &_p_j, const PatchProcession<T> *_patch_processtion_ptr,
                                       double _invalid_res, double _weight) {
        return (new CoupleFixSphFactor(_T_o_w, _p_j, _patch_processtion_ptr, _invalid_res, _weight));
    }

  private:
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_o_w;
    Eigen::Vector3d p_j;
    const PatchProcession<T> *patch_procession_ptr;
    double invalid_res;
    double weight;
};

class CoupleSphFactor : public ceres::SizedCostFunction<1, 16> {
  public:
    CoupleSphFactor(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w, const Eigen::Vector3d &_p_j,
                    const PatchProcession<double> *_patch_processtion_ptr,
                    const SH_TABLE_CONFIG<double> *_SH_table_config_ptr, const Eigen::VectorXd &_sph_coeff)
        : T_o_w(_T_o_w), p_j(_p_j), patch_procession_ptr(_patch_processtion_ptr),
          SH_table_config_ptr(_SH_table_config_ptr), sph_coeff(_sph_coeff) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2));
        T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2)) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
    }

    ~CoupleSphFactor() override {}
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_w_j(parameters[0]);
        //        Eigen::Map<const Eigen::VectorX<double>> sph_coeff(parameters[1], 441);
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_o_j = T_o_w * T_w_j;
        Eigen::Vector3d p_o = T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2)) * p_j + T_o_j(Eigen::seq(0, 2), 3);
        double x = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x, p_o(0), p_o(1));
        //        double x = patch_procession_ptr->fetch_Gamma_x_interp(p_o(0), p_o(1));
        bool is_fault = false;
        if (x == patch_procession_ptr->T_INVALID) {
            is_fault = true;
        }
        double y = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y, p_o(0), p_o(1));
        //        double y = patch_procession_ptr->fetch_Gamma_y_interp(p_o(0), p_o(1));
        if (y == patch_procession_ptr->T_INVALID) {
            is_fault = true;
        }
        double azi_low = SH_table_config_ptr->get_azi_low();
        double azi_high = SH_table_config_ptr->get_azi_high();
        double elev_low = SH_table_config_ptr->get_elev_low();
        double elev_high = SH_table_config_ptr->get_elev_high();
        double theta = y / patch_procession_ptr->max_y * M_PI * SH_table_config_ptr->SH_scale + elev_low;
        double phi = x / patch_procession_ptr->max_x * 2 * M_PI * SH_table_config_ptr->SH_scale + azi_low;
        int theta_idx = std::round((theta - elev_low) / (elev_high - elev_low) * (SH_table_config_ptr->elev_rso - 1));
        int phi_idx = std::round((phi - azi_low) / (azi_high - azi_low) * (SH_table_config_ptr->azi_rso - 1));
        if (!is_fault) {
            // use spherical harmonics coefficients to reconstruct the point
            Eigen::MatrixXd dirs(1, 2);
            dirs << phi, theta;
            Eigen::VectorXd I = curl::invLeastSquaresSHT_table<double>(sph_coeff, dirs, *SH_table_config_ptr,
                                                                       SH_table_config_ptr->warping_SH_degree);
            residuals[0] = p_o(2) - I(0);
            if (I(0) == patch_procession_ptr->T_INVALID) {
                residuals[0] = 0;
            }

        } else {
            residuals[0] = 0;
        }

        // Compute the Jacobian if asked for.
        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                // Map jacobians
                Eigen::Map<Eigen::Matrix<double, 1, 16, Eigen::RowMajor>> Jac_left(jacobians[0], 1, 16);
                Jac_left.setZero();
                if (!is_fault) {
                    // Z_jac
                    Eigen::Matrix<double, 1, 3, Eigen::RowMajor> Z_Jac;
                    Z_Jac << 0, 0, 1;
                    // SE3_Jac
                    Eigen::Matrix<double, 3, 6> SE3_Jac;
                    SE3_Jac(Eigen::seq(0, 2), Eigen::seq(0, 2)) = T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2));
                    SE3_Jac(Eigen::seq(0, 2), Eigen::seq(3, 5)) =
                        -T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2)) * curl::skew<double>(p_j);
                    // Gamma_Jac
                    Eigen::Matrix<double, 2, 2, Eigen::RowMajor> Gamma_Jac;
                    Gamma_Jac(0, 0) =
                        patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x_Gx, p_o(0), p_o(1));
                    //            Gamma_Jac(0, 0) = patch_procession_ptr->fetch_Gamma_x_Gx_interp(p_o(0),
                    //            p_o(1));
                    if (Gamma_Jac(0, 0) == patch_procession_ptr->T_INVALID) {
                        is_fault = true;
                    }
                    Gamma_Jac(0, 1) =
                        patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x_Gy, p_o(0), p_o(1));
                    //            Gamma_Jac(0, 1) = patch_procession_ptr->fetch_Gamma_x_Gy_interp(p_o(0),
                    //            p_o(1));
                    if (Gamma_Jac(0, 1) == patch_procession_ptr->T_INVALID) {
                        is_fault = true;
                    }
                    Gamma_Jac(1, 0) =
                        patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y_Gx, p_o(0), p_o(1));
                    //            Gamma_Jac(1, 0) = patch_procession_ptr->fetch_Gamma_y_Gx_interp(p_o(0),
                    //            p_o(1));
                    if (Gamma_Jac(1, 0) == patch_procession_ptr->T_INVALID) {
                        is_fault = true;
                    }
                    Gamma_Jac(1, 1) =
                        patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y_Gy, p_o(0), p_o(1));
                    //            Gamma_Jac(1, 1) = patch_procession_ptr->fetch_Gamma_y_Gy_interp(p_o(0),
                    //            p_o(1));
                    if (Gamma_Jac(1, 1) == patch_procession_ptr->T_INVALID) {
                        is_fault = true;
                    }
                    if (!is_fault) {
                        // XY_Jac
                        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> XY_Jac;
                        XY_Jac << 1, 0, 0, 0, 1, 0;
                        // theta_Jac
                        double theta_Jac = SH_table_config_ptr->SH_scale * M_PI / patch_procession_ptr->max_y;
                        // phi_Jac
                        double phi_Jac = SH_table_config_ptr->SH_scale * 2 * M_PI / patch_procession_ptr->max_x;
                        // Y_Jac
                        Eigen::Matrix<double, 1, 2, Eigen::RowMajor> Y_Jac;
                        Y_Jac << 0, 1;
                        // X_Jac
                        Eigen::Matrix<double, 1, 2, Eigen::RowMajor> X_Jac;
                        X_Jac << 1, 0;
                        // Block_Jac
                        Eigen::Matrix<double, 2, 6, Eigen::RowMajor> Block_Jac;
                        Block_Jac.row(0) = theta_Jac * Y_Jac * Gamma_Jac * XY_Jac * SE3_Jac;
                        Block_Jac.row(1) = phi_Jac * X_Jac * Gamma_Jac * XY_Jac * SE3_Jac;
                        // Right_Jac & SPH_Jac
                        Eigen::Matrix<double, 1, 6, Eigen::RowMajor> Right_Jac;
                        Right_Jac.setZero();
                        Eigen::Matrix<double, 1, 2, Eigen::RowMajor> SPH_Jac;
                        for (int l = 0; l <= SH_table_config_ptr->warping_SH_degree; ++l) {
                            for (int m = -l; m <= l; ++m) {
                                SPH_Jac(0, 0) = SH_table_config_ptr->SH_G_theta_table(
                                    phi_idx * SH_table_config_ptr->elev_rso + theta_idx, l * l + l + m);
                                SPH_Jac(0, 1) = SH_table_config_ptr->SH_G_phi_table(
                                    phi_idx * SH_table_config_ptr->elev_rso + theta_idx, l * l + l + m);
                                Right_Jac = (Right_Jac + sph_coeff(l * l + l + m) * SPH_Jac * Block_Jac).eval();
                            }
                        }
                        // Jac_left equation
                        Jac_left(0, Eigen::seq(0, 5)) = Z_Jac * SE3_Jac - Right_Jac;
                    }
                }
            }
            //            if (jacobians[1] != nullptr) {
            //                Eigen::Map<Eigen::Matrix<double, 1, 441>> Jac_right(jacobians[1]);
            //                Jac_right.setZero();
            //                if (!is_fault) {
            //                    Jac_right = -SH_table_config_ptr->SH_table.row(phi_idx * SH_table_config_ptr->elev_rso
            //                    + theta_idx);
            //                }
            //            }
        }
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w,
                                       const Eigen::Vector3d &_p_j,
                                       const PatchProcession<double> *_patch_processtion_ptr,
                                       const SH_TABLE_CONFIG<double> *_SH_table_config_ptr,
                                       const Eigen::VectorXd &_sph_coeff) {
        return (new CoupleSphFactor(_T_o_w, _p_j, _patch_processtion_ptr, _SH_table_config_ptr, _sph_coeff));
    }

  private:
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_o_w;
    Eigen::Vector3d p_j;
    const PatchProcession<double> *patch_procession_ptr;
    const SH_TABLE_CONFIG<double> *SH_table_config_ptr;
    Eigen::VectorXd sph_coeff;
};

template <int DEGREE_SIZE> class CoupleSphModFactor : public ceres::SizedCostFunction<1, 16, DEGREE_SIZE> {
  public:
    CoupleSphModFactor(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w, const Eigen::Vector3d &_p_j,
                       const PatchProcession<double> *_patch_processtion_ptr,
                       const SH_TABLE_CONFIG<double> *_SH_table_config_ptr)
        : T_o_w(_T_o_w), p_j(_p_j), patch_procession_ptr(_patch_processtion_ptr),
          SH_table_config_ptr(_SH_table_config_ptr) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2));
        T_o_w(Eigen::seq(0, 2), Eigen::seq(0, 2)) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
    }

    ~CoupleSphModFactor() override {}
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_w_j(parameters[0]);
        Eigen::Map<const Eigen::VectorX<double>> sph_coeff(parameters[1], DEGREE_SIZE);
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_o_j = T_o_w * T_w_j;
        Eigen::Vector3d p_o = T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2)) * p_j + T_o_j(Eigen::seq(0, 2), 3);
        double x = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x, p_o(0), p_o(1));
        //        double x = patch_procession_ptr->fetch_Gamma_x_interp(p_o(0), p_o(1));
        bool is_fault = false;
        if (x == patch_procession_ptr->T_INVALID) {
            is_fault = true;
        }
        double y = patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y, p_o(0), p_o(1));
        //        double y = patch_procession_ptr->fetch_Gamma_y_interp(p_o(0), p_o(1));
        if (y == patch_procession_ptr->T_INVALID) {
            is_fault = true;
        }
        double azi_low = SH_table_config_ptr->get_azi_low();
        double azi_high = SH_table_config_ptr->get_azi_high();
        double elev_low = SH_table_config_ptr->get_elev_low();
        double elev_high = SH_table_config_ptr->get_elev_high();
        double theta = y / patch_procession_ptr->max_y * M_PI * SH_table_config_ptr->SH_scale + elev_low;
        double phi = x / patch_procession_ptr->max_x * 2 * M_PI * SH_table_config_ptr->SH_scale + azi_low;
        int theta_idx = std::round((theta - elev_low) / (elev_high - elev_low) * (SH_table_config_ptr->elev_rso - 1));
        int phi_idx = std::round((phi - azi_low) / (azi_high - azi_low) * (SH_table_config_ptr->azi_rso - 1));
        if (!is_fault) {
            // use spherical harmonics coefficients to reconstruct the point
            Eigen::MatrixXd dirs(1, 2);
            dirs << phi, theta;
            Eigen::VectorXd I = curl::invLeastSquaresSHT_table<double>(sph_coeff, dirs, *SH_table_config_ptr,
                                                                       SH_table_config_ptr->warping_SH_degree);
            //            Eigen::VectorXd I =
            //                curl::invLeastSquaresSHT<double>(sph_coeff, dirs, SH_table_config_ptr->warping_SH_degree);
            residuals[0] = p_o(2) - I(0);
            if (I(0) == patch_procession_ptr->T_INVALID) {
                residuals[0] = 0;
            }

        } else {
            residuals[0] = 0;
        }

        // Compute the Jacobian if asked for.
        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                // Map jacobians
                Eigen::Map<Eigen::Matrix<double, 1, 16, Eigen::RowMajor>> Jac_left(jacobians[0], 1, 16);
                Jac_left.setZero();
                if (!is_fault) {
                    // Z_jac
                    Eigen::Matrix<double, 1, 3, Eigen::RowMajor> Z_Jac;
                    Z_Jac << 0, 0, 1;
                    // SE3_Jac
                    Eigen::Matrix<double, 3, 6> SE3_Jac;
                    SE3_Jac(Eigen::seq(0, 2), Eigen::seq(0, 2)) = T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2));
                    SE3_Jac(Eigen::seq(0, 2), Eigen::seq(3, 5)) =
                        -T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2)) * curl::skew<double>(p_j);
                    // Gamma_Jac
                    Eigen::Matrix<double, 2, 2, Eigen::RowMajor> Gamma_Jac;
                    Gamma_Jac(0, 0) =
                        patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x_Gx, p_o(0), p_o(1));
                    //                    Gamma_Jac(0, 0) = patch_procession_ptr->fetch_Gamma_x_Gx_interp(p_o(0),
                    //                    p_o(1));
                    if (Gamma_Jac(0, 0) == patch_procession_ptr->T_INVALID) {
                        is_fault = true;
                    }
                    Gamma_Jac(0, 1) =
                        patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_x_Gy, p_o(0), p_o(1));
                    //                    Gamma_Jac(0, 1) = patch_procession_ptr->fetch_Gamma_x_Gy_interp(p_o(0),
                    //                    p_o(1));
                    if (Gamma_Jac(0, 1) == patch_procession_ptr->T_INVALID) {
                        is_fault = true;
                    }
                    Gamma_Jac(1, 0) =
                        patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y_Gx, p_o(0), p_o(1));
                    //                    Gamma_Jac(1, 0) = patch_procession_ptr->fetch_Gamma_y_Gx_interp(p_o(0),
                    //                    p_o(1));
                    if (Gamma_Jac(1, 0) == patch_procession_ptr->T_INVALID) {
                        is_fault = true;
                    }
                    Gamma_Jac(1, 1) =
                        patch_procession_ptr->fetch_Gamma(patch_procession_ptr->Gamma_y_Gy, p_o(0), p_o(1));
                    //                    Gamma_Jac(1, 1) = patch_procession_ptr->fetch_Gamma_y_Gy_interp(p_o(0),
                    //                    p_o(1));
                    if (Gamma_Jac(1, 1) == patch_procession_ptr->T_INVALID) {
                        is_fault = true;
                    }
                    if (!is_fault) {
                        // XY_Jac
                        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> XY_Jac;
                        XY_Jac << 1, 0, 0, 0, 1, 0;
                        // theta_Jac
                        double theta_Jac = SH_table_config_ptr->SH_scale * M_PI / patch_procession_ptr->max_y;
                        // phi_Jac
                        double phi_Jac = SH_table_config_ptr->SH_scale * 2 * M_PI / patch_procession_ptr->max_x;
                        // Y_Jac
                        Eigen::Matrix<double, 1, 2, Eigen::RowMajor> Y_Jac;
                        Y_Jac << 0, 1;
                        // X_Jac
                        Eigen::Matrix<double, 1, 2, Eigen::RowMajor> X_Jac;
                        X_Jac << 1, 0;
                        // Block_Jac
                        Eigen::Matrix<double, 2, 6, Eigen::RowMajor> Block_Jac;
                        Block_Jac.row(0) = theta_Jac * Y_Jac * Gamma_Jac * XY_Jac * SE3_Jac;
                        Block_Jac.row(1) = phi_Jac * X_Jac * Gamma_Jac * XY_Jac * SE3_Jac;
                        // Right_Jac & SPH_Jac
                        Eigen::Matrix<double, 1, 6, Eigen::RowMajor> Right_Jac;
                        Right_Jac.setZero();
                        Eigen::Matrix<double, 1, 2, Eigen::RowMajor> SPH_Jac;
                        for (int l = 0; l <= SH_table_config_ptr->warping_SH_degree; ++l) {
                            for (int m = -l; m <= l; ++m) {
                                SPH_Jac(0, 0) = SH_table_config_ptr->SH_G_theta_table(
                                    phi_idx * SH_table_config_ptr->elev_rso + theta_idx, l * l + l + m);
                                SPH_Jac(0, 1) = SH_table_config_ptr->SH_G_phi_table(
                                    phi_idx * SH_table_config_ptr->elev_rso + theta_idx, l * l + l + m);
                                Right_Jac = (Right_Jac + sph_coeff(l * l + l + m) * SPH_Jac * Block_Jac).eval();
                            }
                        }
                        // Jac_left equation
                        Jac_left(0, Eigen::seq(0, 5)) = Z_Jac * SE3_Jac - Right_Jac;
                    }
                }
            }
            if (jacobians[1] != nullptr) {
                Eigen::Map<Eigen::Matrix<double, 1, DEGREE_SIZE>> Jac_right(jacobians[1]);
                Jac_right.setZero();
                if (!is_fault) {
                    Jac_right = -SH_table_config_ptr->SH_table(phi_idx * SH_table_config_ptr->elev_rso + theta_idx,
                                                               Eigen::seq(0, DEGREE_SIZE - 1));
                }
            }
        }
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &_T_o_w,
                                       const Eigen::Vector3d &_p_j,
                                       const PatchProcession<double> *_patch_processtion_ptr,
                                       const SH_TABLE_CONFIG<double> *_SH_table_config_ptr) {
        return (new CoupleSphModFactor(_T_o_w, _p_j, _patch_processtion_ptr, _SH_table_config_ptr));
    }

  private:
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_o_w;
    Eigen::Vector3d p_j;
    const PatchProcession<double> *patch_procession_ptr;
    const SH_TABLE_CONFIG<double> *SH_table_config_ptr;
};

#endif // CURL_SLAM_OPTIMIZATION_TYPES_H