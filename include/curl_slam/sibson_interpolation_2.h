//
// Created by zkc on 14/03/23.
//

#ifndef CURL_SLAM_SIBSON_INTERPOLATION_2_H
#define CURL_SLAM_SIBSON_INTERPOLATION_2_H

#include "sibson_interpolation_2_func.h"

template <typename DerivedP>

std::vector<double>
sibson_interpolation_2(Eigen::MatrixBase<DerivedP> &query_points,
                       CGAL::Triple<Delaunay_triangulation_sibson, Point_value_map_sibson,
                                    Point_vector_map_sibson> &F) {
    assert(query_points.cols() == 2);
    std::vector<double> interpolate_value;
    // coordinate computation
    for (int i = 0; i < query_points.rows(); ++i) {
        K_sibson ::Point_2 p(query_points(i, 0), query_points(i, 1));
        std::vector<std::pair<Point_sibson, Coord_type_sibson>> coords;
        Coord_type_sibson norm =
            CGAL::natural_neighbor_coordinates_2(F.first, p, std::back_inserter(coords)).second;
        if (!coords.empty()) {
            // Sibson interpolant: version without sqrt:
            std::pair<Coord_type_sibson, bool> res = CGAL::sibson_c1_interpolation_square(
                coords.begin(), coords.end(), norm, p,
                CGAL::Data_access<Point_value_map_sibson>(F.second),
                CGAL::Data_access<Point_vector_map_sibson>(F.third), Traits_sibson());
            if (res.second)
                interpolate_value.push_back(res.first);
            else
                std::cout << "C^1 Interpolation not successful." << std::endl
                          << " not all gradients are provided." << std::endl
                          << " You may resort to linear interpolation." << std::endl;
        } else {
            std::cout << "Point is outside of the convex hull" << std::endl;
        }
    }
    return interpolate_value;
}
#endif // CURL_SLAM_SIBSON_INTERPOLATION_2_H
