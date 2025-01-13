//
// Created by zkc on 21/07/22.
//

#ifndef EXAMPLE_CONVEX_HULL_2D_H
#define EXAMPLE_CONVEX_HULL_2D_H

#include "Convex_hull_traits_adapter_2.h"
#include "delaunay_triangulation.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/property_map.h>
#include <Eigen/Core>
#include <numeric>
#include <vector>

namespace cgal {
template <typename DerivedV, typename DerivedF>
inline void convex_hull_2D_mesh(const Eigen::MatrixBase<DerivedV> &V,
                                Eigen::PlainObjectBase<DerivedF> &F) {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_2 Point_2;
    typedef CGAL::Convex_hull_traits_adapter_2<K, CGAL::Pointer_property_map<Point_2>::type>
        Convex_hull_traits_2;
    std::vector<Point_2> points(V.rows());
    for (int i = 0; i < V.rows(); i++) {
        points[i] = Point_2(V(i, 0), V(i, 1));
    }
    std::vector<int> indices(points.size()), out;
    std::iota(indices.begin(), indices.end(), 0);
    CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                        Convex_hull_traits_2(CGAL::make_property_map(points)));
    DerivedV V_mid = V(out, Eigen::all);
    DerivedF F_mid;
    cgal::delaunay_triangulation<DerivedV, DerivedF>(V_mid, F_mid);
    F.resizeLike(F_mid);
    for (int i = 0; i < F_mid.rows(); ++i) {
        F(i, 0) = out[F_mid(i, 0)];
        F(i, 1) = out[F_mid(i, 1)];
        F(i, 2) = out[F_mid(i, 2)];
    }
}
} // namespace cgal

#endif // EXAMPLE_CONVEX_HULL_2D_H
