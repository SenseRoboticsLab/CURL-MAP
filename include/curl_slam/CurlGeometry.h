//
// Created by zkc on 07/02/23.
//
#ifndef CURL_SLAM_CURLGEOMETRY_H
#define CURL_SLAM_CURLGEOMETRY_H
#include "UHEMesh/HEMesh.h"
#include "load_config.h"
#include "string"
#include "types.h"
#include <Eigen/Dense>
#include <iostream>
#include <queue>
#include <utility>

#define ST_INVALID (std::numeric_limits<ST>::max())
#define BT_INVALID (std::numeric_limits<BT>::max())

namespace Ubpa {
class V;
class E;
class P;
class H;
typedef HEMeshTraits_EmptyH<V, E, P> TraitsVEP;
typedef HEMeshTraits<V, E, P, H> TraitsVEPH;

void in_set_values(BT *const ptr, const BT *value_ptr, int dim) {
    for (int i = 0; i < dim; ++i) {
        ptr[i] = value_ptr[i];
    }
}

void in_set_values(BT *const ptr, const Eigen::MatrixX<BT> &vector, int dim) {
    for (int i = 0; i < dim; ++i) {
        ptr[i] = vector(0, i);
    }
}

void in_set_3_value(BT *const ptr, const BT *value_ptr) {
    ptr[0] = value_ptr[0];
    ptr[1] = value_ptr[1];
    ptr[2] = value_ptr[2];
}

void in_set_3_value(BT *const ptr, const Eigen::VectorX<BT> &vector) {
    ptr[0] = vector(0);
    ptr[1] = vector(1);
    ptr[2] = vector(2);
}

/**
 *  Contain six main attributes:
 *  * position
 *  * normal
 *  * color
 *  * semantic
 *  * is_on_boundary
 *  * curvature
 */

class V : public TVertex<TraitsVEPH> {

  public:
    explicit V(int _dim = 3, std::string _name = "NO_NAME", ST _semantic = ST_INVALID);

    explicit V(const BT *_position, int _dim = 3, ST _semantic = ST_INVALID,
               std::string _name = "NO_NAME");

    explicit V(const Eigen::MatrixX<BT> &vector, int _dim = 3, ST _semantic = ST_INVALID,
               std::string _name = "NO_NAME");
    ~V();
    // delete move and copy constructors
    V(const V &other) = delete;
    V &operator=(const V &other) = delete;
    V(V &&other) = delete;
    V &operator=(V &&other) = delete;

  public:
    template <typename T> void set_position(T _position);
    template <typename T> void set_normal(T _normal);
    template <typename T> void set_color(T _color);
    void print() const;
    bool is_vertex_on_boundary();
    Eigen::VectorX<BT> get_position() const;

    Eigen::Vector3<BT> calculate_normal();

  public:
    int dim;
    int signature{-1};
    BT *position{nullptr};
    BT *normal{nullptr};
    BT *color{nullptr};
    ST semantic{ST_INVALID};
    BT gaussian_curvature{BT_INVALID};
    BT mean_curvature{BT_INVALID};
    std::string name;
};

/**
 *  Contain two main attributes:
 *  * color
 *  * is_on_boundary
 */
class E : public TEdge<TraitsVEPH> {
  public:
    explicit E(std::string pre = "E");

    ~E() noexcept;
    // delete move and copy constructors
    E(const E &other) = delete;
    E &operator=(const E &other) = delete;
    E(E &&other) = delete;
    E &operator=(E &&other) = delete;

  public:
    std::string name();
    std::string signature_name();
    template <typename T> void set_color(T _color);
    [[nodiscard]] bool is_edge_on_boundary() const;
    BT calculate_length();

  public:
    std::string pre;
    BT *color{nullptr};
    bool is_on_boundary{false};
    BT length{BT_INVALID};
};

/**
 *  Contain three main attributes:
 *  * normal
 *  * color
 *  * semantic
 *  * Notice: to decide whether a polygon is on boundary or not is based on the edge rather than
 * vertices
 */
class P : public TPolygon<TraitsVEPH> {
  public:
    explicit P(std::string pre = "P");

    ~P();
    // delete move and copy constructors
    P(const P &other) = delete;
    P &operator=(const P &other) = delete;
    P(P &&other) = delete;
    P &operator=(P &&other) = delete;

  public:
    std::string name();
    std::string signature_name();
    template <typename T> void set_normal(T _normal);
    template <typename T> void set_color(T _color);
    template <typename T> void set_angles(T _color);
    template <typename T> void set_lengths(T _color);

    bool is_contain_boundary();

    Eigen::Vector3<BT> calculate_normal();
    Eigen::Vector3<BT> calculate_angles();
    Eigen::Vector3<BT> calculate_lengths();
    BT calculate_area();
    BT calculate_circumference();
    BT calculate_gaussian_curvature();
    BT calculate_mean_curvature();
    Eigen::Vector3<BT> get_centroid();

  public:
    std::string pre;
    BT *normal{nullptr};
    BT *color{nullptr};
    BT *angles{nullptr};
    BT *lengths{nullptr};
    BT area{BT_INVALID};
    ST semantic{ST_INVALID};
    bool is_on_boundary{false};
    bool is_new_seed{false};
    BT circumference{BT_INVALID};
    BT gaussian_curvature{BT_INVALID};
    BT mean_curvature{BT_INVALID};
    bool is_defect{false};
    bool is_remove{false};
};

/**
 * Contain one main attribute
 * Angle
 */
class H : public THalfEdge<TraitsVEPH> {
  public:
    explicit H(std::string _pre = "H");

    // delete move and copy constructors
    H(const H &other) = delete;
    H &operator=(const H &other) = delete;
    H(H &&other) = delete;
    H &operator=(H &&other) = delete;

    void set_triangle_angle();

  public:
    BT get_angle(); // calculate the angle of original vertex
    bool is_halfEdge_on_boundary();

  public:
    std::string pre;
    BT angle{BT_INVALID};
    bool is_on_boundary{false};
};

/**
 * HMesh inheritance
 */
// Todo build a error item class
struct errorItems {
    bool is_on_boundary;
    // face normal angle diff
    BT angle_diff{BT_INVALID};
    // vertex mean normal with face mean normal diff
    BT normal_mean_angle_diff{BT_INVALID};
    BT normal_variance_diff{BT_INVALID};
    // edge length diff
    BT edge_length_mean_diff{BT_INVALID};
    BT edge_length_variance_diff{BT_INVALID};
    // gaussian curvature diff
    BT G_curvature_mean_diff{BT_INVALID};
    BT G_curvature_variance_diff{BT_INVALID};
    // mean curvature diff
    BT M_curvature_mean_diff{BT_INVALID};
    BT M_curvature_variance_diff{BT_INVALID};
};

class HEMeshInherit : public HEMesh<TraitsVEPH> {
  public:
    bool Init(const std::vector<std::vector<BT>> &vertex, int dim,
              const std::vector<std::vector<size_t>> &polygons);
    bool Init(const Eigen::MatrixX<BT> &vertex, int dim,
              const Eigen::Matrix<int, Eigen::Dynamic, 3> &polygons);

  public:
    struct regionElement {
        regionElement(P *_face, errorItems _error) : face(_face), error(_error) {
            // ToDo: here insert rules for whether a new seed to be generated
            if (error.angle_diff > p_b_config.th_normal_angle) {
                set_is_new_seed(true);
                return;
            } else {
                if (p_b_config.is_normal_selected) {
                    if (error.normal_mean_angle_diff > p_b_config.th_diff_normal_norm_mean) {
                        set_is_new_seed(true);
                        return;
                    }
                    if (error.normal_variance_diff > p_b_config.th_diff_normal_variance) {
                        set_is_new_seed(true);
                        return;
                    }
                }
                if (p_b_config.is_length_selected) {
                    if (error.edge_length_mean_diff > p_b_config.diff_length_mean) {
                        set_is_new_seed(true);
                        return;
                    }
                    if (error.edge_length_variance_diff > p_b_config.diff_length_variance) {
                        set_is_new_seed(true);
                        return;
                    }
                }
                if (p_b_config.is_G_curvature_selected) {
                    if (error.G_curvature_mean_diff > p_b_config.th_diff_G_curvature_mean) {
                        set_is_new_seed(true);
                        return;
                    }
                    if (error.G_curvature_variance_diff > p_b_config.th_diff_G_curvature_variance) {
                        set_is_new_seed(true);
                        return;
                    }
                }
                if (p_b_config.is_M_curvature_selected) {
                    if (error.M_curvature_mean_diff > p_b_config.th_diff_M_curvature_mean) {
                        set_is_new_seed(true);
                        return;
                    }
                    if (error.M_curvature_variance_diff > p_b_config.th_diff_M_curvature_variance) {
                        set_is_new_seed(true);
                        return;
                    }
                }
                set_is_new_seed(false);
                return;
            }
        }
        bool operator<(const regionElement &rhs) const {
            return error.angle_diff > rhs.error.angle_diff;
        }                 // will give the lowest error value first
        P *face;
        errorItems error; // face normal angle error
        bool get_is_new_seed() { return face->is_new_seed; }
        void set_is_new_seed(bool val) { face->is_new_seed = val; }
        bool get_is_on_boundary() { return face->is_on_boundary; }
        void set_is_on_boundary(bool val) { face->is_on_boundary = val; }
        [[nodiscard]] ST get_semantic() const { return face->semantic; }
        void set_semantic(ST _semantic) const {
            face->semantic = _semantic;
            face->HalfEdge()->Origin()->semantic = _semantic;
            face->HalfEdge()->End()->semantic = _semantic;
            face->HalfEdge()->Pre()->Origin()->semantic = _semantic;
        }
    };
    struct regionWrapper {
        // constructors
        regionWrapper() = default;
        regionWrapper(P *_seed, ST _region_semantic)
            : seed(_seed), region_semantic(_region_semantic){};
        ~regionWrapper() {}
        // members

        std::set<P *> faces_set;
        std::set<ST> neighbour_set; // Todo: remember that this set could include itself and parent
                                    // idx as well
        std::priority_queue<regionElement> region_queue;
        std::priority_queue<regionElement> potential_seeds_queue;
        P *seed{nullptr};
        ST region_semantic{ST_INVALID};
        // running statistics model
        ST number_faces_in_region{0};
        // normal characteristic
        Eigen::Vector3<BT> normal_mean = Eigen::Vector3<BT>::Zero();
        BT normal_sum_of_diff_norms{0};
        BT normal_variance{0};
        // edge length characteristic
        BT edge_length_mean{0};
        BT edge_length_sum_of_diff_norms{0};
        BT edge_length_variance{0};
        // gaussian curvature characteristic
        BT G_curvature_mean{0};
        BT G_curvature_sum_of_diff_norms{0};
        BT G_curvature_variance{0};
        // mean curvature characteristic
        BT M_curvature_mean{0};
        BT M_curvature_sum_of_diff_norms{0};
        BT M_curvature_variance{0};
    };
    void calculate_face_normal_and_area();
    void calculate_vertex_normal();
    void calculate_vertex_normal_and_curvature(
        const bool &gaussian_curvature_is_selected, const bool &mean_curvature_is_selected,
        const std::string &area_method = "MIXED_VORONOI"); // MIXED_VORONOI or BARYCENTRIC
    void average_face_normal_curvature(bool is_normal_selected, bool is_curvature_selected);
    void probability_clustering(int init_idx, PROBABILITY_CLUSTERING_CONFIG _p_b_config);
    void init_wrapper(ST wrapper_idx, regionElement &new_face_element, ST parent_semantic);
    void update_wrapper(ST wrapper_idx, regionElement &new_face_element);
    bool is_merge_wrappers(ST wrapper_idx_1, ST wrapper_idx_2);
    ST merge_wrappers(ST wrapper_idx_1, ST wrapper_idx_2);
    errorItems
    compute_initial_error(ST wrapper_idx,
                          P *new_face); // only compute face normal difference and add to queue
    /*
     * Remove defect triangles
     */
    void calculate_triangle_properties(const CONFORMAL_MAPPING_CONFIG &conformal_mapping_config,
                                       std::vector<BT> &areas, std::vector<BT> &lengths);
    void remove_defect_triangles(const CONFORMAL_MAPPING_CONFIG &conformal_mapping_config,
                                 std::set<int> &valid_idx);
    void remove_defect_clusters(const CONFORMAL_MAPPING_CONFIG &conformal_mapping_config,
                                std::set<int> &init_valid_face_idx,
                                std::vector<int> &valid_face_idx,
                                std::vector<int> &valid_vertex_idx, std::vector<int> &boundary_idx,
                                std::vector<BT> &boundary_length_segment,
                                std::vector<int> &f_filtered);

    void select_corners_related_with_boundary_length(const std::vector<int> &valid_vertex_idx,
                                                     const std::vector<int> &order_idx,
                                                     const Eigen::Matrix<BT, 2, 4> &corners,
                                                     const Eigen::MatrixX<BT> &projection_2d_pts,
                                                     BT total_boundary_length, std::vector<int> &corner_idx,
                                                     double rectangular_corner_weight);

  public:
    std::unordered_map<ST, regionWrapper> regions_map;
    std::set<ST> regions_idx;
    static PROBABILITY_CLUSTERING_CONFIG p_b_config;
    ST max_cluster_idx{0};
    // debug parameters
  public:
    ST current_seeds{0};
};

std::ostream &operator<<(std::ostream &os, const V *v);

std::ostream &operator<<(std::ostream &os, HEMesh<TraitsVEPH>::H *he);

void Print(const std::shared_ptr<HEMesh<TraitsVEPH>> &mesh);

// template <typename BasicType, typename SemanticType = BasicType> class CurlGeometry {};

} // namespace Ubpa

#endif // CURL_SLAM_CURLGEOMETRY_H
