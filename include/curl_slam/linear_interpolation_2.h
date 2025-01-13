//
// Created by zkc on 14/03/23.
//

#ifndef CURL_SLAM_LINEAR_INTERPOLATION_2_H
#define CURL_SLAM_LINEAR_INTERPOLATION_2_H
#include "linear_interpolation_2_func.h"
#include "types.h"
#define DOUBLE_INVALID (std::numeric_limits<double>::max())
#define BT_INVALID (std::numeric_limits<BT>::max())

BT default_boundary[2] = {-BT_INVALID, BT_INVALID};
/**
 *
 * @tparam DerivedP
 * @param query_points
 * @param F
 * @param idx
 * @return the interpolation result and valid indices
 */
// template <typename DerivedP>
// std::pair<std::vector<double>, std::vector<int>>
// linear_interpolation_2(const Eigen::MatrixBase<DerivedP> &query_points, const interp_func_pair &F,
//                        const double boundary[2] = default_boundary,
//                        const std::vector<double> &value = std::vector<double>()) {
//     //    assert(query_points.cols() == 2);
//     //    std::cout << "linear interpolation" << std::endl;
//     Delaunay_triangulation_linear::Face_handle fh;
//     Delaunay_triangulation_linear::Face_handle fh_last_valid;
//     std::pair<std::vector<double>, std::vector<int>> interpolate_result;
//     for (int i = 0; i < query_points.rows(); ++i) {
//         if (!value.empty()) {
//             if (value[i] != DOUBLE_INVALID) {
//                 K_linear ::Point_2 p(query_points(i, 0), query_points(i, 1));
//                 std::vector<std::pair<Point_linear, Coord_type_linear>> coords;
//                 fh = F.first.locate(p, fh_last_valid);
//                 Coord_type_linear norm =
//                     CGAL::natural_neighbor_coordinates_2(F.first, p, std::back_inserter(coords), fh)
//                         .second;
//                 if (!coords.empty()) {
//                     Coord_type_linear res = CGAL::linear_interpolation(
//                         coords.begin(), coords.end(), norm, Value_access_linear(F.second));
//                     interpolate_result.first.push_back(res);
//                     if ((res >= boundary[0]) && (res <= boundary[1])) {
//                         interpolate_result.second.push_back(i);
//                     }
//                     fh_last_valid = fh;
//                 } else {
//                     interpolate_result.first.push_back(DOUBLE_INVALID);
//                 }
//             } else {
//                 interpolate_result.first.push_back(DOUBLE_INVALID);
//             }
//         } else {
//             K_linear::Point_2 p(query_points(i, 0), query_points(i, 1));
//             std::vector<std::pair<Point_linear, Coord_type_linear>> coords;
//             fh = F.first.locate(p, fh_last_valid);
//             Coord_type_linear norm =
//                 CGAL::natural_neighbor_coordinates_2(F.first, p, std::back_inserter(coords), fh)
//                     .second;
//             if (!coords.empty()) {
//                 Coord_type_linear res = CGAL::linear_interpolation(
//                     coords.begin(), coords.end(), norm, Value_access_linear(F.second));
//                 interpolate_result.first.push_back(res);
//                 if ((res >= boundary[0]) && (res <= boundary[1])) {
//                     interpolate_result.second.push_back(i);
//                 }
//                 fh_last_valid = fh;
//             } else {
//                 interpolate_result.first.push_back(DOUBLE_INVALID);
//             }
//         }
//     }
//
//     return interpolate_result;
// }

template <typename T>
bool is_nearby(const Delaunay_triangulation_linear &triangulation, K_linear::Point_2 &query_point,
               Delaunay_triangulation_linear::Face_handle &fh, T threshold) {
    fh = triangulation.locate(query_point, fh);
    if (fh == nullptr) {
        return false;
    }

    auto nearest_vertex = triangulation.nearest_vertex(query_point, fh);
    if (CGAL::squared_distance(nearest_vertex->point(), query_point) < threshold * threshold) {
        query_point = nearest_vertex->point();
    }

    return true;
}

// ToDo: need to test
template <typename T>
std::pair<std::vector<T>, std::vector<int>> linear_interpolation_2(const Eigen::MatrixX<T> &query_points,
                                                                   const interp_func_pair &F, const T boundary[2],
                                                                   const std::vector<T> &value = std::vector<T>()) {
    //    assert(query_points.cols() == 2);
    //    std::cout << "linear interpolation" << std::endl;
    const T T_INVALID = std::numeric_limits<T>::max();
    Delaunay_triangulation_linear::Face_handle fh;
    //    Delaunay_triangulation_linear::Face_handle fh_last_valid;
    std::pair<std::vector<T>, std::vector<int>> interpolate_result;
    for (int i = 0; i < query_points.rows(); ++i) {
        if (!value.empty()) {
            if (value[i] != T_INVALID) {
                K_linear ::Point_2 p(query_points(i, 0), query_points(i, 1));
                std::vector<std::pair<Point_linear, Coord_type_linear>> coords;
                //                fh = F.first.locate(p, fh);
                is_nearby(F.first, p, fh, 0.001);
                Coord_type_linear norm =
                    CGAL::natural_neighbor_coordinates_2(F.first, p, std::back_inserter(coords), fh).second;
                if (!coords.empty()) {
                    Coord_type_linear res =
                        CGAL::linear_interpolation(coords.begin(), coords.end(), norm, Value_access_linear(F.second));
                    interpolate_result.first.push_back(res);
                    if ((res >= boundary[0]) && (res <= boundary[1])) {
                        interpolate_result.second.push_back(i);
                    }
                    //                    fh_last_valid = fh;
                } else {
                    interpolate_result.first.push_back(T_INVALID);
                }
            } else {
                interpolate_result.first.push_back(T_INVALID);
            }
        } else {
            K_linear::Point_2 p(query_points(i, 0), query_points(i, 1));
            std::vector<std::pair<Point_linear, Coord_type_linear>> coords;
            //            fh = F.first.locate(p, fh);
            is_nearby(F.first, p, fh, 0.001);
            Coord_type_linear norm =
                CGAL::natural_neighbor_coordinates_2(F.first, p, std::back_inserter(coords), fh).second;
            if (!coords.empty()) {
                Coord_type_linear res =
                    CGAL::linear_interpolation(coords.begin(), coords.end(), norm, Value_access_linear(F.second));
                interpolate_result.first.push_back(res);
                if ((res >= boundary[0]) && (res <= boundary[1])) {
                    interpolate_result.second.push_back(i);
                }
                //                fh_last_valid = fh;
            } else {
                interpolate_result.first.push_back(T_INVALID);
            }
        }
    }

    return interpolate_result;
}
#endif // CURL_SLAM_LINEAR_INTERPOLATION_2_H
