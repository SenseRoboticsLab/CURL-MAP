//
// Created by zkc on 15/05/23.
//

#ifndef SRC_CONVEX_HULL_2D_IMPL_H
#define SRC_CONVEX_HULL_2D_IMPL_H

#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/property_map.h>
#include <numeric>
#include <vector>

namespace cgal {
template <typename BasicType>
void convex_hull_2d_methods(const Eigen::MatrixX<BasicType> &points, std::vector<int> &out) {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_2 Point_2;
    typedef CGAL::Convex_hull_traits_adapter_2<K, CGAL::Pointer_property_map<Point_2>::type>
        Convex_hull_traits_2;
    std::vector<Point_2> points_cgal;
    if (points.cols() == 2) {
        points_cgal.reserve(points.rows());
        for (int i = 0; i < points.rows(); i++) {
            points_cgal.emplace_back(points(i, 0), points(i, 1));
        }
    } else if (points.rows() == 2) {
        points_cgal.reserve(points.cols());
        for (int i = 0; i < points.cols(); i++) {
            points_cgal.emplace_back(points(0, i), points(1, i));
        }
    }

    std::vector<std::size_t> indices(points_cgal.size());
    std::iota(indices.begin(), indices.end(), 0);
    CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                        Convex_hull_traits_2(CGAL::make_property_map(points_cgal)));
}

template <typename BasicType>
void convex_hull_2d_ch_bykat(const Eigen::MatrixX<BasicType> &points,
                             Eigen::MatrixX<BasicType> &out) {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_2 Point_2;
    typedef CGAL::Convex_hull_traits_adapter_2<K, CGAL::Pointer_property_map<Point_2>::type>
        Convex_hull_traits_2;
    std::vector<Point_2> points_cgal;
    if (points.cols() == 2) {
        points_cgal.reserve(points.rows());
        for (int i = 0; i < points.rows(); i++) {
            points_cgal.emplace_back(points(i, 0), points(i, 1));
        }
    } else if (points.rows() == 2) {
        points_cgal.reserve(points.cols());
        for (int i = 0; i < points.cols(); i++) {
            points_cgal.emplace_back(points(0, i), points(1, i));
        }
    }
    std::vector<std::size_t> indices(points_cgal.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::vector<Point_2> out_points;
    CGAL::ch_bykat(points_cgal.begin(), points_cgal.end(), std::back_inserter(out_points));
    // append the first point to the end, like matlab
    out_points.push_back(out_points[0]);
    out = Eigen::Map<Eigen::MatrixXd>(reinterpret_cast<double *>(out_points.data()), 2,
                                      static_cast<int>(out_points.size()))
              .cast<BasicType>();
}
} // namespace cgal
#endif // SRC_CONVEX_HULL_2D_IMPL_H
