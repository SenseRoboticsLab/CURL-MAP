//
// Created by zkc on 19/06/23.
//

#ifndef SRC_LIGHT_STRUCTURE_H
#define SRC_LIGHT_STRUCTURE_H
#include "linear_interpolation_2.h"
#include "linear_interpolation_2_func.h"
#include "map_attributes_aabb_tree.h"
#include <memory>
#include <tuple>
template <typename T = double> struct Gamma {
    std::pair<interp_func_pair, interp_func_pair> interp_kernel;
    T min_x;
    T max_x;
    T min_y;
    T max_y;

    Gamma(const std::pair<interp_func_pair, interp_func_pair> &_interp_kernel, const Eigen::MatrixX<T> &points)
        : interp_kernel(_interp_kernel) {
        min_x = points.col(0).minCoeff();
        max_x = points.col(0).maxCoeff();
        min_y = points.col(1).minCoeff();
        max_y = points.col(1).maxCoeff();
    }
};

struct DataInfo {
    DataInfo(double _score, double _IoU) : score(_score), IoU(_IoU) {}
    double score;
    double IoU;
};

struct PointCloudInfo {
    std::vector<Eigen::Isometry3d> T_obj_lidar_vec;
    double time;
    std::vector<bool> is_ground_cloud_vec;
    std::vector<std::pair<std::vector<double>, std::vector<double>>> bounding_box_pair_vec;
    std::vector<std::vector<std::pair<unsigned int, DataInfo>>> intersected_score_pairs_vec;
};

template <typename BasicType> struct TupleCompare {
    bool operator()(const std::tuple<int, double, std::shared_ptr<PatchInfo<BasicType>>> &a,
                    const std::tuple<int, double, std::shared_ptr<PatchInfo<BasicType>>> &b) const {
        if (std::get<1>(a) == std::get<1>(b))
            return std::get<0>(a) > std::get<0>(b);
        else
            return std::get<1>(a) > std::get<1>(b);
    }
};

struct PairCompare {
    bool operator()(const std::pair<int, double> &a, const std::pair<int, double> &b) const {
        if (a.first == b.first)
            return false;
        else
            return a.second <= b.second; // the equality of second value is allowed
    }
};

#endif // SRC_LIGHT_STRUCTURE_H
