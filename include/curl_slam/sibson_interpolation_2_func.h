//
// Created by zkc on 14/03/23.
//

#ifndef CURL_SLAM_SIBSON_INTERPOLATION_2_FUNC_H
#define CURL_SLAM_SIBSON_INTERPOLATION_2_FUNC_H
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Interpolation_gradient_fitting_traits_2.h>
#include <CGAL/interpolation_functions.h>
#include <CGAL/natural_neighbor_coordinates_2.h>
#include <CGAL/sibson_gradient_fitting.h>

#include <iostream>
#include <iterator>
#include <map>
#include <utility>
#include <vector>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K_sibson;
typedef CGAL::Delaunay_triangulation_2<K_sibson> Delaunay_triangulation_sibson;
typedef CGAL::Interpolation_gradient_fitting_traits_2<K_sibson> Traits_sibson;
typedef K_sibson::FT Coord_type_sibson;
typedef K_sibson::Point_2 Point_sibson;
typedef std::map<Point_sibson, Coord_type_sibson, K_sibson::Less_xy_2> Point_value_map_sibson;
typedef std::map<Point_sibson, K_sibson::Vector_2, K_sibson::Less_xy_2> Point_vector_map_sibson;

template <typename DerivedP, typename DerivedValue>
CGAL::Triple<Delaunay_triangulation_sibson, Point_value_map_sibson, Point_vector_map_sibson>
sibson_interpolation_2_func(Eigen::MatrixBase<DerivedP> &points,
                            Eigen::MatrixBase<DerivedValue> &values) {
    assert(points.rows() == values.rows());
    assert(values.cols() == 1);

    Delaunay_triangulation_sibson T;
    Point_value_map_sibson value_function;
    Point_vector_map_sibson gradient_function;
    for (int i = 0; i < points.rows(); ++i) {
        K_sibson::Point_2 p(points(i, 0), points(i, 1));
        T.insert(p);
        value_function.insert(std::make_pair(p, values(i)));
    }

    sibson_gradient_fitting_nn_2(T, std::inserter(gradient_function, gradient_function.begin()),
                                 CGAL::Data_access<Point_value_map_sibson>(value_function),
                                 Traits_sibson());

    // T value_function gradient_function
    CGAL::Triple<Delaunay_triangulation_sibson, Point_value_map_sibson, Point_vector_map_sibson> F;
    F = CGAL::make_triple<Delaunay_triangulation_sibson, Point_value_map_sibson,
                          Point_vector_map_sibson>(T, value_function, gradient_function);
    return F;
}
#endif // CURL_SLAM_SIBSON_INTERPOLATION_2_FUNC_H
