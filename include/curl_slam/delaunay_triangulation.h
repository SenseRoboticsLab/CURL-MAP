//
// Created by zkc on 21/07/22.
//

#ifndef EXAMPLE_DELAUNAY_TRIANGULATION_H
#define EXAMPLE_DELAUNAY_TRIANGULATION_H

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_on_sphere_traits_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <Eigen/Core>
#include <vector>

namespace cgal {
template <typename DerivedV, typename DerivedF>
void delaunay_triangulation(const Eigen::MatrixBase<DerivedV> &V,
                            Eigen::PlainObjectBase<DerivedF> &F) {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel> Vb;
    typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
    typedef CGAL::Delaunay_triangulation_2<Kernel, Tds> Delaunay;
    typedef Kernel::Point_2 Point;
    std::vector<std::pair<Point, unsigned>> points;
    if (V.rows() > V.cols()) {
        points.resize(V.rows());
        for (int i = 0; i < V.rows(); i++) {
            points[i] = std::make_pair(Point(V(i, 0), V(i, 1)), i);
        }
    } else if (V.rows() < V.cols()) {
        points.resize(V.cols());
        for (int i = 0; i < V.cols(); i++) {
            points[i] = std::make_pair(Point(V(0, i), V(1, i)), i);
        }
    }

    Delaunay triangulation;
    triangulation.insert(points.begin(), points.end());
    F.resize(triangulation.number_of_faces(), 3);
    {
        int j = 0;
        for (Delaunay::Finite_faces_iterator fit = triangulation.finite_faces_begin();
             fit != triangulation.finite_faces_end(); ++fit) {
            Delaunay::Face_handle face = fit;
            F(j, 0) = face->vertex(0)->info();
            F(j, 1) = face->vertex(1)->info();
            F(j, 2) = face->vertex(2)->info();
            j++;
        }
    }
}
template <typename DerivedV, typename DerivedF>
void delaunay_triangulation_on_sphere_func(const Eigen::MatrixBase<DerivedV> &V,
                                           Eigen::PlainObjectBase<DerivedF> &F,
                                           Eigen::PlainObjectBase<DerivedV> &V_aft) {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef CGAL::Projection_on_sphere_traits_3<K> Traits;
    typedef CGAL::Delaunay_triangulation_on_sphere_2<Traits> DToS2;
    typedef DToS2::Point_3 Point;
    std::vector<Point> points;
    for (int i = 0; i < V.rows(); i++) {
        points.template emplace_back(V(i, 0), V(i, 1), V(i, 2));
    }
    Traits traits(Point(0, 0, 0), 100); // centered on (0,0,0), with radius 100
    DToS2 dtos(points.begin(), points.end(), traits);
    DToS2::size_type vertices_num = 0;
    std::unordered_map<DToS2::Vertex_handle, DToS2::size_type> index_of_vertex;
    V_aft.resize(dtos.number_of_vertices(), 3);
    for (DToS2::Vertices_iterator it = dtos.vertices_begin(); it != dtos.vertices_end();
         ++it, ++vertices_num) {
        V_aft(vertices_num, 0) = it->point()[0];
        V_aft(vertices_num, 1) = it->point()[1];
        V_aft(vertices_num, 2) = it->point()[2];
        index_of_vertex[it] = vertices_num;
    }
    F.resize(dtos.number_of_faces(), 3);
    DToS2::size_type faces_num = 0;
    for (DToS2::All_faces_iterator fit = dtos.all_faces_begin(); fit != dtos.all_faces_end();
         ++fit, ++faces_num) {
        F(faces_num, 0) = index_of_vertex[fit->vertex(0)];
        F(faces_num, 1) = index_of_vertex[fit->vertex(1)];
        F(faces_num, 2) = index_of_vertex[fit->vertex(2)];
    }
    DToS2::size_type edges_num = 0;
    for (DToS2::All_edges_iterator ed = dtos.all_edges_begin(); ed != dtos.all_edges_end();
         ++ed, ++edges_num) {
    }
    if (vertices_num + faces_num - edges_num != 2) {
        std::cerr << "Not a manifold!" << std::endl;
    } else {
        std::cout << "Manifold" << std::endl;
    }
}
} // namespace cgal

#endif // EXAMPLE_DELAUNAY_TRIANGULATION_H
