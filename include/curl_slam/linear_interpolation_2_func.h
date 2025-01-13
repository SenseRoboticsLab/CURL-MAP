//
// Created by zkc on 14/03/23.
//

#ifndef CURL_SLAM_LINEAR_INTERPOLATION_2_FUNC_H
#define CURL_SLAM_LINEAR_INTERPOLATION_2_FUNC_H
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Interpolation_traits_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/interpolation_functions.h>
#include <CGAL/natural_neighbor_coordinates_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K_linear;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, K_linear> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Delaunay_triangulation_2<K_linear, Tds> Delaunay_triangulation_linear;
typedef CGAL::Interpolation_traits_2<K_linear> Traits_linear;
typedef K_linear::FT Coord_type_linear;
typedef K_linear::Point_2 Point_linear;
typedef std::map<Point_linear, Coord_type_linear, K_linear::Less_xy_2> Coord_map_linear;
typedef CGAL::Data_access<Coord_map_linear> Value_access_linear;
typedef typename std::pair<Delaunay_triangulation_linear, Coord_map_linear> interp_func_pair;

template <typename DerivedP, typename DerivedValue>
interp_func_pair linear_interpolation_2_func(const Eigen::MatrixBase<DerivedP> &points,
                                             const Eigen::MatrixBase<DerivedValue> &values) {
    //    assert(points.cols() == 2);
    assert(points.rows() == values.rows());
    assert(values.cols() == 1);
    interp_func_pair F;
    std::vector<std::pair<K_linear::Point_2, unsigned int>> P;
    for (int i = 0; i < points.rows(); ++i) {
        K_linear::Point_2 p(points(i, 0), points(i, 1));
        P.emplace_back(p, i);
        //        F.first.insert(p);
        F.second.insert(std::make_pair(p, values(i)));
    }
    F.first.insert(P.begin(), P.end());
    return F;
}

template <typename T>
interp_func_pair linear_interpolation_2_func(const Eigen::VectorX<T> &x,
                                             const Eigen::VectorX<T> &y,
                                             const Eigen::VectorX<T> values) {
    interp_func_pair F;
    std::vector<std::pair<K_linear::Point_2, unsigned int>> P;
    for (int i = 0; i < x.size(); ++i) {
        K_linear::Point_2 p(x(i), y(i));
        P.emplace_back(p, i);
        //        F.first.insert(p);
        F.second.insert(std::make_pair(p, values(i)));
    }
    F.first.insert(P.begin(), P.end());
    return F;
}
#endif // CURL_SLAM_LINEAR_INTERPOLATION_2_FUNC_H
