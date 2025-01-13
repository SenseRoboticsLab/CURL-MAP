#if !defined(SURFACE_POINTS_APPROXIMATION)
#define SURFACE_POINTS_APPROXIMATION
#include "load_config.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Variational_shape_approximation.h>
#include <CGAL/boost/graph/Named_function_parameters.h>
#include <CGAL/boost/graph/named_params_helper.h>
#include <CGAL/license/Surface_mesh_approximation.h>
#include <CGAL/property_map.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/type_traits/is_same.hpp>
#include <iostream>
#include <limits>

namespace CGAL {
namespace Surface_mesh_approximation {

/// \ingroup PkgTSMARef
/// @brief Verbose level enumeration.
enum Verbose_level {
    /// Silent
    SILENT,
    /// Main steps
    MAIN_STEPS,
    /// Verbose
    VERBOSE
};

// the named parameter header being not documented the doc is put here for now
#ifdef DOXYGEN_RUNNING
namespace parameters {

/*! \ingroup bgl_namedparameters
 * This function is used when default parameters are just fine for approximation or meshing.
 */
unspecified_type all_default();

} // namespace parameters
#endif
template <typename TriangleMesh, typename NamedParameters>
void approximate_points(const TriangleMesh &tm, const NamedParameters &np) {
    using parameters::choose_parameter;
    using parameters::get_parameter;
    using parameters::is_default_parameter;

    typedef typename CGAL::GetGeomTraits<TriangleMesh, NamedParameters>::type Geom_traits;
    typedef typename Geom_traits::FT FT;

    typedef typename CGAL::GetVertexPointMap<TriangleMesh, NamedParameters>::type Vertex_point_map;
    Vertex_point_map point_pmap =
        choose_parameter(get_parameter(np, internal_np::vertex_point),
                         get_property_map(vertex_point, const_cast<TriangleMesh &>(tm)));

    typedef CGAL::Variational_shape_approximation<TriangleMesh, Vertex_point_map> L21_approx;
    typedef typename L21_approx::Error_metric L21_metric;

    const Verbose_level vl =
        choose_parameter(get_parameter(np, internal_np::verbose_level), SILENT);

    const std::size_t number_of_faces = std::distance(faces(tm).first, faces(tm).second);
    const std::size_t number_of_vertices = std::distance(vertices(tm).first, vertices(tm).second);

    if (vl == MAIN_STEPS || vl == VERBOSE) {
        std::cout << "Variational shape approximation:"
                  << "\n#f " << number_of_faces << "\n#v " << number_of_vertices << std::endl;
    }

    L21_metric metric(tm, point_pmap);
    L21_approx approx(tm, point_pmap, metric);

    // hierarchical seeding by default
    const Seeding_method method =
        choose_parameter(get_parameter(np, internal_np::seeding_method), HIERARCHICAL);
    const std::size_t max_nb_of_proxies =
        choose_parameter(get_parameter(np, internal_np::max_number_of_proxies), 0);
    const FT min_error_drop =
        choose_parameter(get_parameter(np, internal_np::min_error_drop), FT(0.0));
    const std::size_t nb_of_relaxations =
        choose_parameter(get_parameter(np, internal_np::number_of_relaxations), 5);

    if (vl == VERBOSE) {
        std::cout << (method == RANDOM ? "Random"
                                       : (method == INCREMENTAL ? "Incremental" : "Hierarchical"))
                  << " seeding.";
        std::cout << "\n#max_nb_of_proxies = " << max_nb_of_proxies
                  << "\n#min_error_drop = " << min_error_drop << "\nnb_of_relaxations "
                  << nb_of_relaxations << std::endl;
    }

    approx.initialize_seeds(np);

    if (vl == MAIN_STEPS || vl == VERBOSE)
        std::cout << "Seeding done." << std::endl;

    const std::size_t nb_of_iterations =
        choose_parameter(get_parameter(np, internal_np::number_of_iterations), 20);

    if (vl == VERBOSE)
        std::cout << "\n#nb_of_iterations = " << nb_of_iterations << std::endl;

    approx.run(nb_of_iterations);

    if (vl == MAIN_STEPS || vl == VERBOSE) {
        std::cout << "Approximation done."
                  << "\n#proxies = " << approx.number_of_proxies() << std::endl;
    }

    // get proxy map
    approx.proxy_map(get_parameter(np, internal_np::face_proxy_map));

    if (!parameters::is_default_parameter(get_parameter(np, internal_np::face_proxy_map)) &&
        (vl == MAIN_STEPS || vl == VERBOSE))
        std::cout << "Filling face proxy map done." << std::endl;
    /*/
    // get proxies
    approx.proxies(get_parameter(np, internal_np::proxies));

    if (!is_default_parameter(get_parameter(np, internal_np::proxies)) && (vl == MAIN_STEPS || vl ==
    VERBOSE)) std::cout << "Get proxies done." << std::endl;

    // meshing
    bool is_manifold = false;
    if (!is_default_parameter(get_parameter(np, internal_np::anchors)) ||
        !is_default_parameter(get_parameter(np, internal_np::triangles))) {
        if (vl == VERBOSE) {
            const FT subdivision_ratio = choose_parameter(get_parameter(np,
    internal_np::subdivision_ratio), FT(5.0)); const bool relative_to_chord =
    choose_parameter(get_parameter(np, internal_np::relative_to_chord), false); const bool
    with_dihedral_angle = choose_parameter(get_parameter(np, internal_np::with_dihedral_angle),
    false); const bool optimize_anchor_location = choose_parameter(get_parameter(np,
    internal_np::optimize_anchor_location), true); const bool pca_plane =
    choose_parameter(get_parameter(np, internal_np::pca_plane), false); std::cout << "Meshing: "
                      << "\nchord_error = " << subdivision_ratio << "\nrelative_to_chord = " <<
    relative_to_chord
                      << "\nwith_dihedral_angle = " << with_dihedral_angle
                      << "\noptimize_anchor_location = " << optimize_anchor_location << "\npca_plane
    = " << pca_plane
                      << std::endl;
        }

        is_manifold = approx.extract_mesh(np);

        if (vl == MAIN_STEPS || vl == VERBOSE)
            std::cout << "Meshing done.\n"
                      << (is_manifold ? "Can" : "Cannot") << " be built into 2-manifold surface." <<
    std::endl;
    }

    // get anchor points
    approx.anchor_points(get_parameter(np, internal_np::anchors));

    if (!is_default_parameter(get_parameter(np, internal_np::anchors)) && (vl == MAIN_STEPS || vl ==
    VERBOSE)) std::cout << "Get anchors done." << std::endl;

    // get indexed triangles
    approx.indexed_triangles(get_parameter(np, internal_np::triangles));

    if (!is_default_parameter(get_parameter(np, internal_np::triangles)) && (vl == MAIN_STEPS || vl
    == VERBOSE)) std::cout << "Get indexed triangles done." << std::endl;
    //*/
}

} // namespace Surface_mesh_approximation
} // namespace CGAL

namespace cgal {
template <typename DerivedV, typename DerivedF>
void vsa_approximation(const Eigen::MatrixBase<DerivedV> &V,
                       const Eigen::PlainObjectBase<DerivedF> &F, const VSA_CONFIG &vsa_config,
                       Eigen::VectorXi &F_seg) {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
    typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
    typedef Mesh::Property_map<face_descriptor, std::size_t> Face_proxy_pmap;
    namespace VSA = CGAL::Surface_mesh_approximation;
    namespace PMP = CGAL::Polygon_mesh_processing;
    Mesh mesh;
    std::vector<Mesh::Vertex_index> vertex_idx;
    for (int i = 0; i < V.rows(); ++i) {
        vertex_idx.push_back(mesh.add_vertex(Kernel::Point_3(V(i, 0), V(i, 1), V(i, 2))));
    }
    for (int i = 0; i < F.rows(); ++i) {
        mesh.add_face(vertex_idx[F(i, 0)], vertex_idx[F(i, 1)], vertex_idx[F(i, 2)]);
    }
    Face_proxy_pmap fpxmap =
        mesh.add_property_map<face_descriptor, std::size_t>("f:proxy_id", 0).first;
    auto VSA_seeding_method = VSA::HIERARCHICAL;
    if (vsa_config.seeding_method == "RANDOM")
        VSA_seeding_method = VSA::RANDOM;
    else if (vsa_config.seeding_method == "INCREMENTAL")
        VSA_seeding_method = VSA::INCREMENTAL;
    VSA::approximate_points(
        mesh,
        CGAL::parameters::seeding_method(VSA_seeding_method) // hierarchical seeding
            .max_number_of_proxies(
                vsa_config.max_number_of_proxies) // seeding with maximum number of proxies
            .number_of_iterations(
                vsa_config.number_of_iterations) // number of clustering iterations after seeding
            .min_error_drop(vsa_config.min_error_drop) // seeding with minimum error drop
            .face_proxy_map(fpxmap));                  // faces index of clustering
    int i = 0;
    F_seg.resize(F.rows());
    for (face_descriptor f : faces(mesh)) {
        F_seg(i) = fpxmap[f];
        ++i;
    }
}
} // namespace cgal

#endif // SURFACE_POINTS_APPROXIMATION
