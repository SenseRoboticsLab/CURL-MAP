

//
// Created by zkc on 26/07/22.
//

#ifndef EXAMPLE_BA_ERROR_TERM_H
#define EXAMPLE_BA_ERROR_TERM_H
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "curl_tools.h"
#include <utility>

namespace curl {
class SPHCoeffRotError {
  private:
    const Eigen::VectorXd coeff_1;
    const Eigen::VectorXd coeff_2;

  public:
    SPHCoeffRotError(Eigen::VectorXd _coeff_1, Eigen::VectorXd _coeff_2)
        : coeff_1(std::move(_coeff_1)), coeff_2(std::move(_coeff_2)) {}

    bool operator()(const double *const quat_ptr, double *residuals_ptr) const {
        Eigen::Map<const Eigen::Quaternion<double>> quat(quat_ptr);
        Eigen::MatrixXd R_xyz = quat.normalized().toRotationMatrix();
        auto R_sh = getSHrotMtx<Eigen::MatrixXd, double>(R_xyz, 5);
        Eigen::VectorXd coeff_1_2 = R_sh * coeff_2;
        residuals_ptr[0] = (coeff_1 - coeff_1_2).array().pow(2).sqrt().sum();
        return true;
    }
    static ceres::CostFunction *Create(const Eigen::VectorXd &_coeff_1,
                                       const Eigen::VectorXd &_coeff_2) {
        return (new ceres::NumericDiffCostFunction<SPHCoeffRotError, ceres::CENTRAL, 1, 4>(
            new SPHCoeffRotError(_coeff_1, _coeff_2)));
    }
};
} // namespace curl

#endif // EXAMPLE_BA_ERROR_TERM_H
