//
// Created by zkc on 09/02/23.
//
#include "CurlGeometry.h"
#include <utility>

namespace Ubpa {
/**
 * Discrete K-L divergence
 */
BT K_L_divergence(BT mean1, BT std1, BT mean2, BT std2) {
    return (log(std2 / std1) + (std1 * std1 + (mean1 - mean2) * (mean1 - mean2)) / (2 * std2 * std2) - 0.5);
}

BT Jensen_Shannon_divergence(BT mean1, BT std1, BT mean2, BT std2) {
    BT mean_M = (mean1 + mean2) / 2;
    BT std_M = sqrt((mean1 * mean1 + std1 * std1 + mean2 * mean2 + std2 * std2) / 2 - mean_M * mean_M);

    return (0.5 * (K_L_divergence(mean1, std1, mean_M, std_M) + K_L_divergence(mean2, std2, mean_M, std_M)));
}

/*************************************************************************************************/
/*************************************Primitive Algorithms****************************************/
/*************************************************************************************************/

/**
 * V
 */
// constructors
V::V(int _dim, std::string _name, ST _semantic) : name(std::move(_name)), dim(_dim), semantic(_semantic) {}

V::V(const BT *_position, int _dim, ST _semantic, std::string _name)
    : semantic(_semantic), dim(_dim), name(std::move(_name)) {
    position = new BT[dim];
    in_set_values(position, _position, dim);
}

V::V(const Eigen::MatrixX<BT> &vector, int _dim, ST _semantic, std::string _name)
    : semantic(_semantic), dim(_dim), name(std::move(_name)) {
    position = new BT[dim];
    in_set_values(position, vector, dim);
}

V::~V() {
    delete[] position;
    delete[] normal;
    delete[] color;
}

// member functions
template <typename T> void V::set_position(const T _position) {
    if (position == nullptr) {
        position = new BT[3];
    }
    in_set_3_value(position, _position);
}

template <typename T> void V::set_normal(const T _normal) {
    if (normal == nullptr) {
        normal = new BT[3];
    }
    in_set_3_value(normal, _normal);
}

template <typename T> void V::set_color(const T _color) {
    if (color == nullptr) {
        color = new BT[3];
    }
    in_set_3_value(color, _color);
}

void V::print() const {
    std::cout << "{V" << signature << " - ";
    if (name != "NO_NAME") {
        std::cout << name << ": ";
    }
    if (position != nullptr) {
        std::cout << position[0] << " " << position[1] << " " << position[2];
    }
    if (semantic != std::numeric_limits<ST>::max()) {
        std::cout << " S: " << semantic;
    }
    std::cout << "}";
}

bool V::is_vertex_on_boundary() {
    for (auto *he : OutHalfEdges()) {
        if (he->is_halfEdge_on_boundary()) {
            return true;
        }
    }
    return false;
}

Eigen::Vector3<BT> V::calculate_normal() { // ToDo: haven't check correctness
    assert(Degree() > 0);
    if (normal == nullptr) {
        normal = new BT[3];
        Eigen::Vector3<BT> vertex_normal;
        vertex_normal.setZero();
        for (auto *he : this->OutHalfEdges()) {
            vertex_normal += he->Polygon()->calculate_normal() * he->Polygon()->calculate_area();
        }
        this->set_normal(vertex_normal.normalized().data());
    }
    return Eigen::Map<Eigen::Vector3<BT>>(normal);
}

Eigen::VectorX<BT> V::get_position() const {
    Eigen::VectorX<BT> v(this->dim);
    Eigen::Map<const Eigen::VectorX<BT>> v_map(this->position, this->dim);
    v = v_map;
    return v;
}

/**
 *  E
 */
// constructors
E::E(std::string pre) : pre(std::move(pre)) {}

E::~E() noexcept { delete[] color; }

// member functions
std::string E::name() { return "[" + pre + "]" + HalfEdge()->Origin()->name + "-" + HalfEdge()->End()->name; }

std::string E::signature_name() {
    return "[" + pre + "] " + "v" + std::to_string(HalfEdge()->Origin()->signature) + "-" + "v" +
           std::to_string(HalfEdge()->End()->signature);
}

template <typename T> void E::set_color(const T _color) {
    if (color == nullptr) {
        color = new BT[3];
    }
    in_set_3_value(color, _color);
}

bool E::is_edge_on_boundary() const { return is_on_boundary; }

BT E::calculate_length() {
    if (length == BT_INVALID) {
        Eigen::Map<const Eigen::VectorX<BT>> v0(this->HalfEdge()->Origin()->position, this->HalfEdge()->Origin()->dim);
        Eigen::Map<const Eigen::VectorX<BT>> v1(this->HalfEdge()->End()->position, this->HalfEdge()->End()->dim);
        length = (v1 - v0).norm();
    }
    return length;
}

/**
 *  P
 */
// constructors
P::P(std::string pre) : pre(std::move(pre)) {}

P::~P() {
    delete[] normal;
    delete[] color;
    delete[] angles;
}

// member functions
std::string P::name() {
    std::string name = "[" + pre + "]";
    auto *he = HalfEdge();
    name += he->Origin()->name;
    do {
        name += "-" + he->End()->name;
        he = he->Next();
    } while (he != HalfEdge());
    return name;
}

std::string P::signature_name() {
    std::string name = "[" + pre + "]";
    auto *he = HalfEdge();
    name = name + "v" + std::to_string(he->Origin()->signature);
    do {
        name = name + "-" + "v" + std::to_string(he->End()->signature);
        he = he->Next();
    } while (he != HalfEdge());
    return name;
}

template <typename T> void P::set_normal(const T _normal) {
    if (normal == nullptr) {
        normal = new BT[3];
    }
    in_set_3_value(normal, _normal);
}

template <typename T> void P::set_color(const T _color) {
    if (color == nullptr) {
        color = new BT[3];
    }
    in_set_3_value(color, _color);
}

template <typename T> void P::set_angles(T _angles) {
    if (angles == nullptr) {
        angles = new BT[3];
    }
    in_set_3_value(angles, _angles);
}

template <typename T> void P::set_lengths(T _color) {
    if (lengths == nullptr) {
        lengths = new BT[3];
    }
    in_set_3_value(lengths, _color);
}

bool P::is_contain_boundary() {
    for (auto *he : HalfEdge()->NextLoop()) {
        if (he->is_halfEdge_on_boundary()) {
            return true;
        }
    }
    return false;
}

Eigen::Vector3<BT> P::calculate_normal() { // ToDo: haven't check correctness
    assert(Degree() == 3);
    if (normal == nullptr) {
        normal = new BT[3];
        Eigen::Map<const Eigen::VectorX<BT>> v0(this->HalfEdge()->Origin()->position, this->HalfEdge()->Origin()->dim);
        Eigen::Map<const Eigen::Vector3<BT>> v1(this->HalfEdge()->Next()->Origin()->position,
                                                this->HalfEdge()->Next()->Origin()->dim);
        Eigen::Map<const Eigen::Vector3<BT>> v2(this->HalfEdge()->Next()->End()->position,
                                                this->HalfEdge()->Next()->End()->dim);
        this->set_normal(((v1 - v0).cross(v2 - v1)).normalized().data());
    }
    return Eigen::Map<Eigen::Vector3<BT>>(normal);
}

Eigen::Vector3<BT> P::calculate_angles() {
    assert(Degree() == 3);
    if (angles == nullptr) {
        angles = new BT[3];
        Eigen::Vector3<BT> eigen_lengths = calculate_lengths(); // this step is necessary
        angles[0] = acos((pow(eigen_lengths(0), 2) + pow(eigen_lengths(2), 2) - pow(eigen_lengths(1), 2)) /
                         (2 * eigen_lengths(0) * eigen_lengths(2)));
        angles[1] = acos((pow(eigen_lengths(0), 2) + pow(eigen_lengths(1), 2) - pow(eigen_lengths(2), 2)) /
                         (2 * eigen_lengths(0) * eigen_lengths(1)));
        angles[2] = acos((pow(eigen_lengths(1), 2) + pow(eigen_lengths(2), 2) - pow(eigen_lengths(0), 2)) /
                         (2 * eigen_lengths(1) * eigen_lengths(2)));
    }
    return Eigen::Map<Eigen::Vector3<BT>>(angles);
}

Eigen::Vector3<BT> P::calculate_lengths() {
    assert(Degree() == 3);
    if (lengths == nullptr) {
        lengths = new BT[3];
        lengths[0] = this->HalfEdge()->Edge()->calculate_length();
        lengths[1] = this->HalfEdge()->Next()->Edge()->calculate_length();
        lengths[2] = this->HalfEdge()->Next()->Next()->Edge()->calculate_length();
    }
    return Eigen::Map<Eigen::Vector3<BT>>(lengths);
}

BT P::calculate_area() {
    if (area == BT_INVALID) {
        Eigen::Vector3<BT> eigen_lengths = calculate_lengths();
        BT s = eigen_lengths.sum() / 2;
        area = sqrt(s * (s - eigen_lengths(0)) * (s - eigen_lengths(1)) * (s - eigen_lengths(2)));
    }
    return area;
}

BT P::calculate_circumference() { // ToDo: this could be optimized
    assert(Degree() == 3);
    if (circumference == BT_INVALID) {
        circumference = calculate_lengths().sum();
    }
    return circumference;
}

BT P::calculate_gaussian_curvature() { // ToDo: could speed up
    assert(Degree() == 3);
    if (gaussian_curvature == BT_INVALID) {
        gaussian_curvature = (HalfEdge()->Origin()->gaussian_curvature + HalfEdge()->End()->gaussian_curvature +
                              HalfEdge()->Pre()->Origin()->gaussian_curvature) /
                             3;
    }
    return gaussian_curvature;
}

BT P::calculate_mean_curvature() { // ToDo: could speed up
    assert(Degree() == 3);
    if (mean_curvature == BT_INVALID) {
        mean_curvature = (HalfEdge()->Origin()->mean_curvature + HalfEdge()->End()->mean_curvature +
                          HalfEdge()->Pre()->Origin()->mean_curvature) /
                         3;
    }
    return mean_curvature;
}

Eigen::Vector3<BT> P::get_centroid() {
    Eigen::Vector3<BT> centroid;
    centroid = (Eigen::Map<const Eigen::Vector3<BT>>(HalfEdge()->Origin()->position) +
                Eigen::Map<const Eigen::Vector3<BT>>(HalfEdge()->End()->position) +
                Eigen::Map<const Eigen::Vector3<BT>>(HalfEdge()->Pre()->Origin()->position)) /
               3;
    return centroid;
}

/**
 * H
 */
// constructors
H::H(std::string _pre) : pre(std::move(_pre)) {}

// Member functions
void H::set_triangle_angle() { // ToDo: check this algorithm for segmentation, seems wrong
    if (Next()->angle != 0 && Pre()->angle != 0) {
        angle = M_PIl - Next()->angle - Pre()->angle;
    } else {
        Eigen::Map<const Eigen::Vector3<BT>> v0(Origin()->position);
        Eigen::Map<const Eigen::Vector3<BT>> v1(Next()->Origin()->position);
        Eigen::Map<const Eigen::Vector3<BT>> v2(Pre()->Origin()->position);
        BT v1_v0_square = (v1 - v0).squaredNorm();
        BT v2_v1_square = (v2 - v1).squaredNorm();
        BT v0_v2_square = (v0 - v2).squaredNorm();
        angle = acos((v1_v0_square + v0_v2_square - v2_v1_square) / (2 * sqrt(v1_v0_square * v0_v2_square)));
    }
}

BT H::get_angle() {
    assert(Polygon()->Degree() == 3); // must be triangle polygon
    if (angle == BT_INVALID) {
        set_triangle_angle();
        return angle;
    } else {
        return angle;
    }
}

bool H::is_halfEdge_on_boundary() { return (Edge()->is_edge_on_boundary()); }

/*************************************************************************************************/
/***************************************Mesh Algorithms*******************************************/
/*************************************************************************************************/

/**
 * HEMesh
 */
// member functions
PROBABILITY_CLUSTERING_CONFIG
HEMeshInherit::p_b_config; // need to be declared in cpp file because it is static
bool HEMeshInherit::Init(const std::vector<std::vector<BT>> &vertex, int dim,
                         const std::vector<std::vector<size_t>> &polygons) {
    assert(!polygons.empty());

    Clear();

    size_t max = 0;
    size_t min = std::numeric_limits<size_t>::max();
    for (const auto &polygon : polygons) { // find the highest and lowest indices of polygons
        assert(polygon.size() > 2);

        for (auto idx : polygon) {
            if (idx > max)
                max = idx;
            if (idx < min)
                min = idx;
        }
    }

    assert(min == 0);

    for (size_t i = 0; i <= max; i++) // create number of max vertices
        New<V>(vertex[i].data(), dim);

    for (const auto &polygon : polygons) { // extract all vertices from each polygon and connect
                                           // them into half-edges, edges and polygons
        std::vector<H *> heLoop;
        for (size_t i = 0; i < polygon.size(); i++) {
            size_t next = (i + 1) % polygon.size();
            assert(polygon[i] != polygon[next]);
            auto *u = vertices[polygon[i]];
            auto *v = vertices[polygon[next]];
            auto *he = u->HalfEdgeTo(v);
            if (!he)
                he = AddEdge(u, v)->HalfEdge();
            heLoop.push_back(he);
        }
        auto *p = AddPolygon(heLoop);
        assert(p != nullptr);
    }
    return true;
}

// Eigen version
bool HEMeshInherit::Init(const Eigen::MatrixX<BT> &vertex, int dim,
                         const Eigen::Matrix<int, Eigen::Dynamic, 3> &polygons) {
    assert(polygons.size() != 0);
    Clear();
    size_t max = polygons.maxCoeff();
    size_t min = polygons.minCoeff();

    assert(min == 0);

    for (int i = 0; i <= max; i++) { // create number of max vertices
        if (dim == 2) {
            Eigen::Vector2<BT> the_vertex;
            if (vertex.cols() == dim) {
                the_vertex = vertex.row(i);
            } else {
                the_vertex = vertex.col(i);
            }
            New<V>(the_vertex.data(), dim);
        } else if (dim == 3) {
            Eigen::Vector3<BT> the_vertex;
            if (vertex.cols() == dim) {
                the_vertex = vertex.row(i);
            } else {
                the_vertex = vertex.col(i);
            }
            New<V>(the_vertex.data(), dim);
        }
    }
    int k = 0;
    for (const auto &polygon : polygons.rowwise()) { // extract all vertices from each polygon and connect them into
                                                     // half-edges, edges and polygons
        std::vector<H *> heLoop;
        for (size_t i = 0; i < polygon.size(); i++) {
            size_t next = (i + 1) % polygon.size();
            assert(polygon(i) != polygon(next));
            auto *u = vertices[polygon(static_cast<long>(i))];
            auto *v = vertices[polygon(static_cast<long>(next))];
            auto *he = u->HalfEdgeTo(v);
            if (!he)
                he = AddEdge(u, v)->HalfEdge();
            heLoop.push_back(he);
        }
        auto *p = AddPolygon(heLoop);
        assert(p != nullptr);
    }
    return true;
}

/*************************************************************************************************/
/***************************************Mesh Algorithms*******************************************/
/*****************************************Segmentation********************************************/
/*************************************************************************************************/

void HEMeshInherit::calculate_face_normal_and_area() {
    assert(!polygons.empty() && !vertices.empty());
    BT a, b, c, s;
    for (auto *p : Polygons()) {
        Eigen::Map<const Eigen::Vector3<BT>> v0(p->HalfEdge()->Origin()->position);
        Eigen::Map<const Eigen::Vector3<BT>> v1(p->HalfEdge()->Next()->Origin()->position);
        Eigen::Map<const Eigen::Vector3<BT>> v2(p->HalfEdge()->Next()->End()->position);
        p->set_normal(((v1 - v0).cross(v2 - v1)).normalized().data());
        a = (v1 - v0).norm();
        b = (v2 - v1).norm();
        c = (v0 - v2).norm();
        s = (a + b + c) / 2;
        p->area = sqrt(s * (s - a) * (s - b) * (s - c));
    }
}

void HEMeshInherit::calculate_vertex_normal() {
    Eigen::Vector3<BT> vertex_normal;
    for (auto *v : Vertices()) {
        vertex_normal.setZero();
        for (auto *he : v->OutHalfEdges()) {
            vertex_normal += Eigen::Map<Eigen::Vector3<BT>>(he->Polygon()->normal) * he->Polygon()->area;
        }
        v->set_normal(vertex_normal.normalized().data());
    }
}

// according to
// http://rodolphe-vaillant.fr/entry/33/curvature-of-a-triangle-mesh-definition-and-computation
void HEMeshInherit::calculate_vertex_normal_and_curvature(const bool &gaussian_curvature_is_selected,
                                                          const bool &mean_curvature_is_selected,
                                                          const std::string &area_method) {
    Eigen::Vector3<BT> vertex_normal;
    /**
     * Gaussian curvature parameters
     */
    Eigen::Vector3<BT> v0;
    Eigen::Vector3<BT> v1;
    Eigen::Vector3<BT> v2;
    BT angle_theta;
    /**
     * mean curvature parameters
     */
    Eigen::Vector3<BT> pi;
    Eigen::Vector3<BT> pj;
    Eigen::Vector3<BT> p_curvature;
    BT angle_alpha;
    BT angle_beta;
    /**
     * local region area parameter
     */
    BT local_averaging_region;
    BT angle_v1;
    BT angle_v2;
    for (auto *v : Vertices()) {
        vertex_normal.setZero();
        local_averaging_region = 0;

        if (gaussian_curvature_is_selected) {
            angle_theta = 0;
        }
        if (mean_curvature_is_selected) {
            pi = Eigen::Map<Eigen::Vector3<BT>>(v->position);
            p_curvature.setZero();
        }
        if (area_method == "MIXED_VORONOI") {
            v0 = Eigen::Map<Eigen::Vector3<BT>>(v->position);
        }
        for (auto *he : v->OutHalfEdges()) {
            vertex_normal += Eigen::Map<Eigen::Vector3<BT>>(he->Polygon()->normal) * he->Polygon()->area;
            if (gaussian_curvature_is_selected) {
                angle_theta += he->get_angle();
            }
            if (mean_curvature_is_selected) {
                angle_alpha = he->Next()->get_angle();
                angle_beta = he->Pre()->Pair()->Pre()->get_angle();
                pj = Eigen::Map<Eigen::Vector3<BT>>(he->Pre()->Origin()->position);
                p_curvature += ((cos(angle_alpha) / sin(angle_alpha) + cos(angle_beta) / sin(angle_beta)) * (pj - pi));
            }
            if (area_method == "MIXED_VORONOI") {
                if ((he->get_angle() > M_PI_2) || (he->Next()->get_angle() > M_PI_2) ||
                    (he->Pre()->get_angle() > M_PI_2)) {
                    if (he->get_angle() > M_PI_2) {
                        assert(he->Polygon()->area > 0);
                        local_averaging_region += (he->Polygon()->area / 2.0);
                    } else {
                        local_averaging_region += (he->Polygon()->area / 4.0);
                    }
                } else {
                    v1 = Eigen::Map<Eigen::Vector3<BT>>(he->End()->position);
                    v2 = Eigen::Map<Eigen::Vector3<BT>>(he->Pre()->Origin()->position);
                    angle_v1 = he->Next()->get_angle();
                    angle_v2 = he->Pre()->get_angle();
                    local_averaging_region += (0.125 * ((v0 - v2).squaredNorm() * (cos(angle_v1) / sin(angle_v1)) +
                                                        (v1 - v0).squaredNorm() * (cos(angle_v2) / cos(angle_v2))));
                }
            } else {
                local_averaging_region += he->Polygon()->area;
            }
        }
        v->set_normal(vertex_normal.normalized().data());
        if (area_method == "MIXED_VORONOI") {
            local_averaging_region = 0.125 * local_averaging_region;
        } else {
            local_averaging_region = local_averaging_region / 3;
        }
        if (gaussian_curvature_is_selected) {
            v->gaussian_curvature = (2 * M_PI - angle_theta) / local_averaging_region;
            //            if (v->gaussian_curvature > 100) {
            //                v->gaussian_curvature = 100;
            //            } else if (v->gaussian_curvature < -100) {
            //                v->gaussian_curvature = -100;
            //            }
        }
        if (mean_curvature_is_selected) {
            v->mean_curvature = (p_curvature.array() / (2 * local_averaging_region)).matrix().norm() / 2;
        }
    }
}
void clear_queue(std::queue<P *> &q) {
    std::queue<P *> empty;
    q.swap(empty);
}
void clear_set(std::unordered_set<P *> &s) {
    std::unordered_set<P *> empty;
    s.swap(empty);
}
void HEMeshInherit::average_face_normal_curvature(bool is_normal_selected,
                                                  bool is_curvature_selected) { // ToDo: has the problem of double free
    std::unordered_set<P *> is_visited;
    int counter = 0;
    Eigen::Vector3<BT> total_normal;
    Eigen::Vector3<BT> tmp_normal;
    Eigen::Matrix<BT, Eigen::Dynamic, 3> tmp_normal_matrix(Polygons().size(), 3);
    Eigen::Vector<BT, Eigen::Dynamic> tmp_curvature(Polygons().size());
    total_normal.setZero();
    BT total_curvature = 0;
    int idx = 0;
    for (auto *f : Polygons()) {
        std::queue<P *> wait_list;
        wait_list.push(f);
        while ((!wait_list.empty()) && (counter < p_b_config.average_window_size)) {
            P *center_face = wait_list.front();
            wait_list.pop();
            is_visited.insert(center_face);
            if (is_normal_selected) {
                tmp_normal = Eigen::Map<Eigen::Vector3<BT>>(center_face->normal);
                total_normal += tmp_normal;
            }
            if (is_curvature_selected) {
                total_curvature += center_face->calculate_gaussian_curvature();
            }
            ++counter;
            for (auto *he : center_face->HalfEdge()->NextLoop()) {
                if (is_visited.find(he->Pair()->Polygon()) == is_visited.end()) {
                    wait_list.push(he->Pair()->Polygon());
                }
            }
        }
        if (is_normal_selected) {
            tmp_normal_matrix.row(idx) = (total_normal / p_b_config.average_window_size).normalized();
            total_normal.setZero();
        }
        if (is_curvature_selected) {
            tmp_curvature(idx) = total_curvature / p_b_config.average_window_size;
            total_curvature = 0;
        }
        counter = 0;
        clear_set(is_visited);
        clear_queue(wait_list);
        ++idx;
    }
    idx = 0;
    for (auto *f : Polygons()) {
        f->set_normal(tmp_normal_matrix.row(idx));
        f->gaussian_curvature = tmp_curvature(idx);
        ++idx;
    }
}

// ToDo: finish the flow
void HEMeshInherit::probability_clustering(int init_idx, PROBABILITY_CLUSTERING_CONFIG _p_b_config) {
    p_b_config = std::move(_p_b_config);
    for (auto *f : Polygons()) {
        f->semantic = ST_INVALID;
    }
    if (p_b_config.is_average_face_normal_and_curvature == "both") {
        average_face_normal_curvature(true, true);
    } else if (p_b_config.is_average_face_normal_and_curvature == "normal") {
        average_face_normal_curvature(true, false);
    } else if (p_b_config.is_average_face_normal_and_curvature == "curvature") {
        average_face_normal_curvature(false, true);
    }
    // init number of window size of triangles first
    ST parent_idx = 0;
    ST current_idx = 0;
    ST next_idx = current_idx;
    P *init_polygon = Polygons().at(init_idx);
    regionWrapper init_region_wrapper;
    //    regions_map.insert(std::make_pair(parent_idx, init_region_wrapper));
    regions_map[current_idx] = init_region_wrapper;
    regions_idx.insert(current_idx);
    regionElement init_face_element(init_polygon, compute_initial_error(parent_idx, init_polygon));
    init_wrapper(parent_idx, init_face_element, parent_idx);
    for (auto it = regions_idx.begin(); it != regions_idx.end(); ++it) {
        current_idx = *it;
        while (!regions_map.at(current_idx).region_queue.empty() ||
               !regions_map.at(current_idx).potential_seeds_queue.empty()) {
            if (!regions_map.at(current_idx).region_queue.empty()) {
                regionElement center_face = regions_map.at(current_idx).region_queue.top();
                regions_map.at(current_idx).region_queue.pop();
                if (center_face.get_semantic() == ST_INVALID) {
                    center_face.set_semantic(current_idx);
                    regions_map.at(current_idx).faces_set.insert(center_face.face);
                    for (auto *he :
                         center_face.face->HalfEdge()->NextLoop()) { // This place can't check is_new_seed or not
                        if (he->Pair()->Polygon()->semantic == ST_INVALID) {
                            regionElement new_face_element(he->Pair()->Polygon(),
                                                           compute_initial_error(current_idx, he->Pair()->Polygon()));
                            update_wrapper(current_idx, new_face_element);
                        } else if (he->Pair()->Polygon()->semantic != next_idx) { // record neighbours of the cluster
                            regions_map.at(current_idx).neighbour_set.insert(he->Pair()->Polygon()->semantic);
                            regions_map.at(he->Pair()->Polygon()->semantic).neighbour_set.insert(current_idx);
                            // ToDo: write update_wrapper here to remove edge faces from mean and
                            // variance
                        }
                    }
                }
            } else if (!regions_map.at(current_idx)
                            .potential_seeds_queue.empty()) { // only potential seed queue has value
                regionElement potential_seed = regions_map.at(current_idx).potential_seeds_queue.top();
                regions_map.at(current_idx).potential_seeds_queue.pop();
                if (potential_seed.get_semantic() == ST_INVALID) { // this is necessary, because the seed may already
                                                                   // been added in other region
                    next_idx = max_cluster_idx + 1;
                    regionWrapper init_region_wrapper_again;
                    //                    regions_map.insert(std::make_pair(next_idx,
                    //                    init_region_wrapper_again));
                    regions_map[next_idx] = init_region_wrapper;
                    init_wrapper(next_idx, potential_seed, regions_map.at(current_idx).region_semantic);
                    while (!regions_map.at(next_idx).region_queue.empty()) {
                        regionElement center_face = regions_map.at(next_idx).region_queue.top();
                        regions_map.at(next_idx).region_queue.pop();
                        if (center_face.get_semantic() == ST_INVALID) {
                            center_face.set_semantic(next_idx);
                            regions_map.at(next_idx).faces_set.insert(center_face.face);
                            for (auto *he : center_face.face->HalfEdge()->NextLoop()) {
                                if (he->Pair()->Polygon()->semantic == ST_INVALID) {
                                    regionElement new_face_element(
                                        he->Pair()->Polygon(), compute_initial_error(next_idx, he->Pair()->Polygon()));
                                    update_wrapper(next_idx, new_face_element);
                                } else if (he->Pair()->Polygon()->semantic !=
                                           next_idx) { // record neighbours of the cluster
                                    regions_map.at(next_idx).neighbour_set.insert(he->Pair()->Polygon()->semantic);
                                    regions_map.at(he->Pair()->Polygon()->semantic).neighbour_set.insert(next_idx);
                                    // ToDo: write update_wrapper here to remove edge faces from
                                    // mean and variance
                                }
                            }
                        }
                    }
                    if (p_b_config.is_merge) {
                        // start to check whether this region should be merging back or start
                        bool is_merged = false;
                        std::cout << "Current idx: " << current_idx;
                        for (auto &nei_it : regions_map.at(next_idx).neighbour_set) {
                            if (nei_it != next_idx) {
                                std::cout << " Check whether " << nei_it << "-" << next_idx << ": ";
                                if (is_merge_wrappers(nei_it, next_idx)) {
                                    auto it_tmp = regions_idx.find(merge_wrappers(nei_it, next_idx));
                                    if (*it_tmp < *it) {
                                        it = it_tmp;
                                    }
                                    std::cout << "Succeed" << std::endl;
                                    current_idx = *it;
                                    std::cout << "Returned idx: " << current_idx << std::endl;
                                    is_merged = true;
                                    break;
                                }
                            }
                        }
                        if (!is_merged) {
                            ++max_cluster_idx;
                            regions_idx.insert(max_cluster_idx);
                            std::cout << "Failed" << std::endl;
                        }
                    } else {
                        ++max_cluster_idx;
                        regions_idx.insert(max_cluster_idx);
                    }
                }
            }
        }
    }
}

void HEMeshInherit::init_wrapper(ST wrapper_idx, regionElement &new_face_element, ST parent_semantic) {
    assert(new_face_element.get_is_new_seed() == true);
    //    new_face_element.set_semantic(wrapper_idx); // here don't need to preset the semantic
    regionWrapper &region_wrapper = regions_map.at(wrapper_idx);
    //    region_wrapper.region_vec.push_back(new_face_element);
    region_wrapper.region_queue.push(new_face_element);
    region_wrapper.seed = new_face_element.face;
    if (wrapper_idx != parent_semantic) {
        region_wrapper.neighbour_set.insert(parent_semantic);
    }
    region_wrapper.region_semantic = wrapper_idx;
    region_wrapper.number_faces_in_region = 1;
    if (p_b_config.is_normal_selected) {
        assert(new_face_element.face->normal != nullptr);
        region_wrapper.normal_mean << new_face_element.face->normal[0], new_face_element.face->normal[1],
            new_face_element.face->normal[2];
    }
    if (p_b_config.is_length_selected) {
        region_wrapper.edge_length_mean = new_face_element.face->calculate_circumference();
    }
    if (p_b_config.is_G_curvature_selected) {
        region_wrapper.G_curvature_mean = new_face_element.face->calculate_gaussian_curvature();
    }
    if (p_b_config.is_M_curvature_selected) {
        region_wrapper.M_curvature_mean = new_face_element.face->calculate_mean_curvature();
    }
}

// update regionWrapper of unordered_map
void HEMeshInherit::update_wrapper(ST wrapper_idx, regionElement &new_face_element) {
    regionWrapper &region_wrapper = regions_map.at(wrapper_idx);
    //    new_face_element.set_semantic(region_wrapper.region_semantic); do not need to update
    //    semantic here region_wrapper.region_vec.push_back(new_face_element);
    if (new_face_element.get_is_new_seed()) {
        if (region_wrapper.number_faces_in_region > p_b_config.initial_window_size) {
            region_wrapper.potential_seeds_queue.push(new_face_element);
            return; // ToDo: Decide later whether this "return" should be commented
        } else {
            new_face_element.set_is_new_seed(false);
            region_wrapper.region_queue.push(new_face_element);
        }
    } else {
        region_wrapper.region_queue.push(new_face_element);
    }
    ++region_wrapper.number_faces_in_region;
    if (!new_face_element.get_is_on_boundary()) {
        if (p_b_config.is_normal_selected) {
            Eigen::Vector3<BT> face_normal;
            Eigen::Vector3<BT> delta;
            face_normal = Eigen::Map<Eigen::Vector3<BT>>(new_face_element.face->normal);
            delta = face_normal - region_wrapper.normal_mean;
            region_wrapper.normal_mean =
                (region_wrapper.normal_mean + delta / region_wrapper.number_faces_in_region).normalized();
            region_wrapper.normal_sum_of_diff_norms += (delta.transpose() * (face_normal - region_wrapper.normal_mean));
            region_wrapper.normal_variance =
                region_wrapper.normal_sum_of_diff_norms / (static_cast<BT>(region_wrapper.number_faces_in_region) - 1);
        }
        if (p_b_config.is_length_selected) {
            BT delta = new_face_element.face->calculate_circumference() - region_wrapper.edge_length_mean;
            region_wrapper.edge_length_mean += (delta / static_cast<BT>(region_wrapper.number_faces_in_region));
            region_wrapper.edge_length_sum_of_diff_norms +=
                (delta * (new_face_element.face->calculate_circumference() - region_wrapper.edge_length_mean));
            region_wrapper.edge_length_variance = region_wrapper.edge_length_sum_of_diff_norms /
                                                  (static_cast<BT>(region_wrapper.number_faces_in_region) - 1);
        }
        if (p_b_config.is_G_curvature_selected) {
            BT delta = new_face_element.face->calculate_gaussian_curvature() - region_wrapper.G_curvature_mean;
            region_wrapper.G_curvature_mean += (delta / static_cast<BT>(region_wrapper.number_faces_in_region));
            region_wrapper.G_curvature_sum_of_diff_norms +=
                (delta * (new_face_element.face->calculate_gaussian_curvature() - region_wrapper.G_curvature_mean));
            region_wrapper.G_curvature_variance = region_wrapper.G_curvature_sum_of_diff_norms /
                                                  (static_cast<BT>(region_wrapper.number_faces_in_region) - 1);
        }
        if (p_b_config.is_M_curvature_selected) {
            BT delta = new_face_element.face->calculate_mean_curvature() - region_wrapper.M_curvature_mean;
            region_wrapper.M_curvature_mean += (delta / static_cast<BT>(region_wrapper.number_faces_in_region));
            region_wrapper.M_curvature_sum_of_diff_norms +=
                (delta * (new_face_element.face->calculate_gaussian_curvature() - region_wrapper.M_curvature_mean));
            region_wrapper.M_curvature_variance = region_wrapper.M_curvature_sum_of_diff_norms /
                                                  (static_cast<BT>(region_wrapper.number_faces_in_region) - 1);
        }
    }
}
// ToDo: neighbour_set also need to be updated
bool HEMeshInherit::is_merge_wrappers(unsigned long wrapper_idx_1, unsigned long wrapper_idx_2) {
    bool is_merge;
    if (wrapper_idx_1 > wrapper_idx_2) {
        ST tmp = wrapper_idx_1;
        wrapper_idx_1 = wrapper_idx_2;
        wrapper_idx_2 = tmp;
    }
    errorItems error;
    // wrappers
    regionWrapper &region_wrapper_1 = regions_map.at(wrapper_idx_1);
    regionWrapper &region_wrapper_2 = regions_map.at(wrapper_idx_2);
    if (region_wrapper_2.number_faces_in_region < 1) {
        is_merge = true;
        return is_merge;
    }
    if (p_b_config.is_length_selected) {
        error.edge_length_mean_diff = abs(region_wrapper_2.edge_length_mean - region_wrapper_1.edge_length_mean) /
                                      region_wrapper_1.edge_length_mean;
        if (error.edge_length_mean_diff > p_b_config.diff_length_mean) {
            return false;
        }
    }
    if (p_b_config.is_normal_selected) {
        error.normal_mean_angle_diff =
            (region_wrapper_2.normal_mean - region_wrapper_1.normal_mean).norm() / region_wrapper_1.normal_mean.norm();
        error.normal_variance_diff =
            abs(region_wrapper_2.normal_variance - region_wrapper_1.normal_variance) / region_wrapper_1.normal_variance;
        if (error.normal_mean_angle_diff < p_b_config.diff_normal_norm_mean &&
            error.normal_variance_diff < p_b_config.diff_normal_variance) {
            is_merge = true;
            return is_merge;
        } else {
            is_merge = false;
        }
    }
    if (p_b_config.is_G_curvature_selected) {
        if (p_b_config.comparison_metric == "mean") {
            error.G_curvature_mean_diff = abs(region_wrapper_2.G_curvature_mean - region_wrapper_1.G_curvature_mean) /
                                          region_wrapper_1.G_curvature_mean;
            error.G_curvature_variance_diff =
                abs(region_wrapper_2.G_curvature_variance - region_wrapper_1.G_curvature_variance) /
                region_wrapper_1.G_curvature_variance;
            if (error.G_curvature_mean_diff < p_b_config.diff_G_curvature_mean &&
                error.G_curvature_variance_diff < p_b_config.diff_G_curvature_variance) {
                is_merge = true;
                return is_merge;
            } else {
                is_merge = false;
            }
        } else if (p_b_config.comparison_metric == "K_L") {
            BT similarity =
                K_L_divergence(region_wrapper_1.G_curvature_mean, sqrt(region_wrapper_1.G_curvature_variance),
                               region_wrapper_2.G_curvature_mean, sqrt(region_wrapper_2.G_curvature_variance));
            if (similarity < p_b_config.K_L_J_S_threshold) {
                is_merge = true;
                return is_merge;
            } else {
                is_merge = false;
            }
        } else if (p_b_config.comparison_metric == "J_S") {
            BT similarity = Jensen_Shannon_divergence(
                region_wrapper_1.G_curvature_mean, sqrt(region_wrapper_1.G_curvature_variance),
                region_wrapper_2.G_curvature_mean, sqrt(region_wrapper_2.G_curvature_variance));
            if (similarity < p_b_config.K_L_J_S_threshold) {
                is_merge = true;
                return is_merge;
            } else {
                is_merge = false;
            }
        }
    }
    if (p_b_config.is_M_curvature_selected) {
        error.M_curvature_mean_diff = abs(region_wrapper_2.M_curvature_mean - region_wrapper_1.M_curvature_mean) /
                                      region_wrapper_1.M_curvature_mean;
        error.M_curvature_variance_diff =
            abs(region_wrapper_2.M_curvature_variance - region_wrapper_1.M_curvature_variance) /
            region_wrapper_1.M_curvature_variance;
        if (error.M_curvature_mean_diff < p_b_config.diff_M_curvature_mean &&
            error.M_curvature_variance_diff < p_b_config.diff_M_curvature_variance) {
            return true;
        }
    }
    return is_merge;
}
// merge regionWrapper of unordered_map
ST HEMeshInherit::merge_wrappers(ST wrapper_idx_1, ST wrapper_idx_2) {
    if (wrapper_idx_1 > wrapper_idx_2) {
        ST tmp = wrapper_idx_1;
        wrapper_idx_1 = wrapper_idx_2;
        wrapper_idx_2 = tmp;
    }
    // wrappers
    regionWrapper &region_wrapper_1 = regions_map.at(wrapper_idx_1);
    regionWrapper &region_wrapper_2 = regions_map.at(wrapper_idx_2);

    // faces_set
    std::set<P *> &faces_set_1 = region_wrapper_1.faces_set;
    std::set<P *> &faces_set_2 = region_wrapper_2.faces_set;
    // update faces belong to wrapper_idx_2
    for (auto &it : faces_set_2) {
        it->semantic = wrapper_idx_1;
    }
    // move faces_set from wrapper_idx_2 to wrapper_idx_1
    if (!faces_set_2.empty()) {
        if (faces_set_2.size() > faces_set_2.size()) {
            std::swap(faces_set_1, faces_set_2);
        }
        faces_set_1.insert(faces_set_2.begin(), faces_set_2.end());
    }
    // neighbour_set
    std::set<ST> &neighbour_set_1 = region_wrapper_1.neighbour_set;
    std::set<ST> &neighbour_set_2 = region_wrapper_2.neighbour_set;
    // before merge neighbour_set from 2 to 1, we need to update neighbour_set of 2's neighbours
    // first
    for (auto &it : neighbour_set_2) {
        regions_map.at(it).neighbour_set.erase(region_wrapper_2.region_semantic);
        regions_map.at(it).neighbour_set.insert(region_wrapper_1.region_semantic);
    }
    // merge neighbour_set
    if (!neighbour_set_2.empty()) {
        if (neighbour_set_2.size() > neighbour_set_1.size()) {
            std::swap(neighbour_set_1, neighbour_set_2);
        }
        neighbour_set_1.insert(neighbour_set_2.begin(), neighbour_set_2.end());
        if (neighbour_set_1.find(wrapper_idx_1) != neighbour_set_1.end()) {
            neighbour_set_1.erase(wrapper_idx_1);
        }
    }
    // priority queues
    std::priority_queue<regionElement> &region_queue_1 = region_wrapper_1.region_queue;
    std::priority_queue<regionElement> &region_queue_2 = region_wrapper_2.region_queue;
    if (!region_queue_2.empty()) {
        //        regionElement old_seed = region_wrapper_2.region_queue.top();
        //        old_seed.is_new_seed = false;
        if (region_queue_2.size() > region_queue_1.size()) {
            std::swap(region_queue_1, region_queue_2);
        }
        while (!region_queue_2.empty()) {
            region_queue_1.push(region_queue_2.top());
            region_queue_2.pop();
        }
    }
    // potential seeds queues
    std::priority_queue<regionElement> &potential_seeds_queue_1 = region_wrapper_1.potential_seeds_queue;
    std::priority_queue<regionElement> &potential_seeds_queue_2 = region_wrapper_2.potential_seeds_queue;
    if (!potential_seeds_queue_2.empty()) {
        if (potential_seeds_queue_2.size() > potential_seeds_queue_1.size()) {
            std::swap(potential_seeds_queue_1, potential_seeds_queue_2);
        }
        while (!potential_seeds_queue_2.empty()) {
            potential_seeds_queue_1.push(potential_seeds_queue_2.top());
            potential_seeds_queue_2.pop();
        }
    }

    // running statistics model
    region_wrapper_1.number_faces_in_region += region_wrapper_2.number_faces_in_region;
    // normal characteristic
    if (p_b_config.is_normal_selected) {
        region_wrapper_1.normal_mean = ((region_wrapper_1.normal_mean + region_wrapper_2.normal_mean) / 2).normalized();
        region_wrapper_1.normal_sum_of_diff_norms += region_wrapper_2.normal_sum_of_diff_norms;
        region_wrapper_1.normal_variance =
            region_wrapper_1.normal_sum_of_diff_norms / (static_cast<BT>(region_wrapper_1.number_faces_in_region) - 1);
    }

    // edge length characteristic
    if (p_b_config.is_length_selected) {
        region_wrapper_1.edge_length_mean = (region_wrapper_1.edge_length_mean + region_wrapper_2.edge_length_mean) / 2;
        region_wrapper_1.edge_length_sum_of_diff_norms += +region_wrapper_2.edge_length_sum_of_diff_norms;
        region_wrapper_1.edge_length_variance = region_wrapper_1.edge_length_sum_of_diff_norms /
                                                (static_cast<BT>(region_wrapper_1.number_faces_in_region) - 1);
    }

    // gaussian curvature characteristic
    if (p_b_config.is_G_curvature_selected) {
        region_wrapper_1.G_curvature_mean = (region_wrapper_1.G_curvature_mean + region_wrapper_2.G_curvature_mean) / 2;
        region_wrapper_1.G_curvature_sum_of_diff_norms += region_wrapper_2.G_curvature_sum_of_diff_norms;
        region_wrapper_1.G_curvature_variance = region_wrapper_1.G_curvature_sum_of_diff_norms /
                                                (static_cast<BT>(region_wrapper_1.number_faces_in_region) - 1);
    }

    //    mean curvature characteristic
    if (p_b_config.is_M_curvature_selected) {
        region_wrapper_1.M_curvature_mean = (region_wrapper_1.M_curvature_mean + region_wrapper_2.M_curvature_mean) / 2;
        region_wrapper_1.M_curvature_sum_of_diff_norms += region_wrapper_2.M_curvature_sum_of_diff_norms;
        region_wrapper_1.M_curvature_variance = region_wrapper_1.M_curvature_sum_of_diff_norms /
                                                (static_cast<BT>(region_wrapper_1.number_faces_in_region) - 1);
    }

    // delete second one
    regions_map.erase(wrapper_idx_2);
    regions_idx.erase(wrapper_idx_2);
    // return lower index; write a recursive function here

    return wrapper_idx_1;
}

errorItems HEMeshInherit::compute_initial_error(ST wrapper_idx, P *new_face) {
    assert(new_face->Degree() == 3);
    errorItems error;
    Eigen::Vector3<BT> face_normal;
    face_normal = Eigen::Map<Eigen::Vector3<BT>>(new_face->normal);
    regionWrapper &region_wrapper = regions_map.at(wrapper_idx);
    if (region_wrapper.normal_mean.cwiseAbs().sum() == 0) {
        error.angle_diff = BT_INVALID;
    } else {
        error.angle_diff = acos(face_normal.transpose() * region_wrapper.normal_mean);
    }
    if (p_b_config.is_normal_selected) {
        Eigen::Vector3<BT> n0;
        Eigen::Vector3<BT> n1;
        Eigen::Vector3<BT> n2;
        n0 = Eigen::Map<Eigen::Vector3<BT>>(new_face->HalfEdge()->Origin()->normal);
        n1 = Eigen::Map<Eigen::Vector3<BT>>(new_face->HalfEdge()->End()->normal);
        n2 = Eigen::Map<Eigen::Vector3<BT>>(new_face->HalfEdge()->Pre()->Origin()->normal);
        Eigen::Vector3<BT> normal_mean;
        normal_mean = (n0 + n1 + n2) / 3;
        error.normal_mean_angle_diff =
            (normal_mean - region_wrapper.normal_mean).norm() / region_wrapper.normal_mean.norm();
        BT normal_variance =
            ((n0 - normal_mean).squaredNorm() + (n1 - normal_mean).squaredNorm() + (n2 - normal_mean).squaredNorm()) /
            (3);
        error.normal_variance_diff =
            abs(normal_variance - region_wrapper.normal_variance) / region_wrapper.normal_variance;
    }
    if (p_b_config.is_length_selected) {
        error.edge_length_mean_diff = abs(new_face->calculate_circumference() - region_wrapper.edge_length_mean) /
                                      region_wrapper.edge_length_mean;
    }
    if (p_b_config.is_G_curvature_selected) {
        BT G_mean = new_face->calculate_gaussian_curvature();
        error.G_curvature_mean_diff = abs(G_mean - region_wrapper.G_curvature_mean) / region_wrapper.G_curvature_mean;
        BT G_0 = new_face->HalfEdge()->Origin()->gaussian_curvature;
        BT G_1 = new_face->HalfEdge()->End()->gaussian_curvature;
        BT G_2 = new_face->HalfEdge()->Pre()->Origin()->gaussian_curvature;
        BT G_curvature_variance = (pow((G_0 - G_mean), 2) + pow((G_1 - G_mean), 2) + pow((G_2 - G_mean), 2)) / (3);
        error.G_curvature_variance_diff =
            abs(G_curvature_variance - region_wrapper.G_curvature_variance) / region_wrapper.G_curvature_variance;
    }
    if (p_b_config.is_M_curvature_selected) {
        BT M_mean = new_face->calculate_mean_curvature();
        error.M_curvature_mean_diff = abs(M_mean - region_wrapper.M_curvature_mean) / region_wrapper.M_curvature_mean;
        BT M_0 = new_face->HalfEdge()->Origin()->mean_curvature;
        BT M_1 = new_face->HalfEdge()->End()->mean_curvature;
        BT M_2 = new_face->HalfEdge()->Pre()->Origin()->mean_curvature;
        BT M_curvature_variance = (pow((M_0 - M_mean), 2) + pow((M_1 - M_mean), 2) + pow((M_2 - M_mean), 2)) / (3);
        error.M_curvature_variance_diff =
            abs(M_curvature_variance - region_wrapper.M_curvature_variance) / region_wrapper.M_curvature_variance;
    }
    for (auto *he : new_face->HalfEdge()->NextLoop()) {
        if ((he->Pair()->Polygon()->semantic != new_face->semantic) &&
            (he->Pair()->Polygon()->semantic != ST_INVALID) &&
            (he->Pair()->Polygon()->is_new_seed)) { // ToDo: add is new seed here
            new_face->is_on_boundary = true;
            break;
        } else {
            new_face->is_on_boundary = false;
        }
    }
    return error;
}
/*************************************************************************************************/
/***************************************Mesh Algorithms*******************************************/
/************************************Remove defect triangles***************************************/
/*************************************************************************************************/
void HEMeshInherit::calculate_triangle_properties(const CONFORMAL_MAPPING_CONFIG &conformal_mapping_config,
                                                  std::vector<BT> &areas, std::vector<BT> &lengths) {
    assert(!polygons.empty() && !vertices.empty());
    BT s;
    areas.clear();
    lengths.clear();
    for (auto *p : Polygons()) {
        if (p->HalfEdge()->Edge()->length == BT_INVALID) {
            lengths.push_back(p->HalfEdge()->Edge()->calculate_length());
        }
        if (p->HalfEdge()->Next()->Edge()->length == BT_INVALID) {
            lengths.push_back(p->HalfEdge()->Next()->Edge()->calculate_length());
        }
        if (p->HalfEdge()->Next()->Next()->Edge()->length == BT_INVALID) {
            lengths.push_back(p->HalfEdge()->Next()->Next()->Edge()->calculate_length());
        }
        Eigen::Vector3<BT> angles = p->calculate_angles();
        areas.push_back(p->calculate_area());
        if (angles.minCoeff() < conformal_mapping_config.tri_angle_min ||
            angles.maxCoeff() > conformal_mapping_config.tri_angle_max) {
            p->is_defect = true;
        }
    }
}

void HEMeshInherit::remove_defect_triangles(const CONFORMAL_MAPPING_CONFIG &conformal_mapping_config,
                                            std::set<int> &valid_idx) {
    std::deque<size_t> remove_checking_indinces(Polygons().size());
    std::vector<int> pre_remove_checking_indinces;
    std::iota(remove_checking_indinces.begin(), remove_checking_indinces.end(), 0);
    valid_idx.insert(remove_checking_indinces.begin(), remove_checking_indinces.end());
    while (!remove_checking_indinces.empty()) {
        size_t idx = remove_checking_indinces.front();
        remove_checking_indinces.pop_front();
        if (!Polygons()[idx]->is_remove) {
            if ((Polygons()[idx]->calculate_lengths().maxCoeff() > conformal_mapping_config.tri_length_max) ||
                (Polygons()[idx]->calculate_area() > conformal_mapping_config.tri_area_max)) {
                Polygons()[idx]->is_defect = true;
            }
            int num_boundary_edges = 0;
            H *boundary_half_edge = nullptr;
            for (auto *he : Polygons()[idx]->HalfEdge()->NextLoop()) {
                if (he->Pair()->IsOnBoundary() || he->Pair()->Polygon()->is_remove) {
                    ++num_boundary_edges;
                    boundary_half_edge = he;
                    if ((Polygons()[idx]->is_defect) || (num_boundary_edges >= 2)) {
                        Polygons()[idx]->is_remove = true;
                    } else {
                        // remove triangles which connects others only via one vertex
                        pre_remove_checking_indinces.clear();
                        for (int i = 0; i < 2; ++i) {
                            V *v;
                            if (i == 0) {
                                v = he->Origin();
                            } else {
                                v = he->End();
                            }
                            bool last_state;
                            if (v->HalfEdge()->IsOnBoundary()) {
                                last_state = true;
                            } else {
                                pre_remove_checking_indinces.push_back(
                                    static_cast<int>(Index(v->HalfEdge()->Polygon())));
                                last_state = v->HalfEdge()->Polygon()->is_remove;
                            }
                            int reverse_times = 0;
                            bool current_state;
                            for (auto *he_out : v->OutHalfEdges()) {
                                if (he_out->IsOnBoundary()) {
                                    current_state = true;
                                } else {
                                    current_state = he_out->Polygon()->is_remove;
                                    pre_remove_checking_indinces.push_back(static_cast<int>(Index(he_out->Polygon())));
                                }
                                if (last_state != current_state) {
                                    last_state = current_state;
                                    ++reverse_times;
                                }
                            }
                            if (reverse_times > 2) {
                                Polygons()[idx]->is_remove = true;
                            }
                        }
                    }

                    if (Polygons()[idx]->is_remove) {
                        valid_idx.erase(static_cast<int>(idx));
                        // add neighbour polygons indices in to the loop
                        for (auto *he_again : Polygons()[idx]->HalfEdge()->NextLoop()) {
                            if (!he_again->Pair()->IsOnBoundary()) {
                                if (!he_again->Pair()->Polygon()->is_remove) {
                                    remove_checking_indinces.push_back(Index(he_again->Pair()->Polygon()));
                                }
                            }
                        }
                        remove_checking_indinces.insert(remove_checking_indinces.end(),
                                                        pre_remove_checking_indinces.begin(),
                                                        pre_remove_checking_indinces.end());
                        pre_remove_checking_indinces.clear();
                        break;
                    }
                }
            }
        }
    }
}

void HEMeshInherit::remove_defect_clusters(const CONFORMAL_MAPPING_CONFIG &conformal_mapping_config,
                                           std::set<int> &init_valid_face_idx, std::vector<int> &valid_face_idx,
                                           std::vector<int> &valid_vertex_idx, std::vector<int> &boundary_idx,
                                           std::vector<BT> &boundary_length_segment, std::vector<int> &f_filtered) {
    // ToDo: can select the valid cluster by using the half area as the threshold
    // ToDo: build a whole new index system without useless vertices
    int thres_size = static_cast<int>(init_valid_face_idx.size()) / 2;
    std::set<int> boundary_indices;
    while (!init_valid_face_idx.empty()) {
        std::deque<int> unvisited_faces_idx;
        int init_polygon = *init_valid_face_idx.begin();
        unvisited_faces_idx.push_back(init_polygon);
        valid_face_idx.clear();
        valid_vertex_idx.clear();
        boundary_indices.clear();
        f_filtered.clear();
        int idx_inc = 0;
        while (!unvisited_faces_idx.empty()) {
            int current_idx = unvisited_faces_idx.front();
            unvisited_faces_idx.pop_front();
            if ((Polygons()[current_idx]->semantic == ST_INVALID) && (!Polygons()[current_idx]->is_remove)) {
                Polygons()[current_idx]->semantic = 0;
                valid_face_idx.push_back(current_idx);
                init_valid_face_idx.erase(current_idx);
                for (auto *he : Polygons()[current_idx]->HalfEdge()->NextLoop()) {
                    if (he->Origin()->signature == -1) {
                        he->Origin()->signature = idx_inc;
                        valid_vertex_idx.push_back(static_cast<int>(Index(he->Origin())));
                        ++idx_inc;
                    }
                    if (he->End()->signature == -1) {
                        he->End()->signature = idx_inc;
                        valid_vertex_idx.push_back(static_cast<int>(Index(he->End())));
                        ++idx_inc;
                    }
                    f_filtered.push_back(he->Origin()->signature);
                    if (!he->Pair()->IsOnBoundary()) {
                        if ((he->Pair()->Polygon()->semantic == ST_INVALID) && (!he->Pair()->Polygon()->is_remove)) {
                            unvisited_faces_idx.push_back(static_cast<int>(Index(he->Pair()->Polygon())));
                        } else if (he->Pair()->Polygon()->is_remove) {
                            boundary_indices.insert(he->Origin()->signature);
                            boundary_indices.insert(he->End()->signature);
                            he->is_on_boundary = true; // can make sure it is conterclockwise
                            he->is_on_boundary = true;
                        }
                    } else {
                        boundary_indices.insert(he->Origin()->signature);
                        boundary_indices.insert(he->End()->signature);
                        he->is_on_boundary = true; // can make sure it is conterclockwise
                        he->is_on_boundary = true;
                    }
                }
            }
        }
        if (valid_face_idx.size() >= thres_size) {
            break;
        }
    }
    if (valid_face_idx.empty()) {
        return;
    }
    // make sure that boundary points are adjecent
    boundary_idx.clear();
    boundary_idx.reserve(boundary_indices.size());
    int b_idx = *boundary_indices.begin();
    boundary_idx.emplace_back(b_idx);
    boundary_indices.erase(b_idx);
    boundary_length_segment.clear();
    boundary_length_segment.reserve(boundary_indices.size());
    while (!boundary_indices.empty()) {
        for (auto *he : Vertices()[valid_vertex_idx[b_idx]]->OutHalfEdges()) {
            if (he->is_on_boundary) {
                b_idx = he->End()->signature;
                boundary_idx.emplace_back(b_idx);
                boundary_indices.erase(b_idx);
                boundary_length_segment.emplace_back(he->Edge()->calculate_length());
                break;
            }
        }
    }
    // add the last length segment
    for (auto *he : Vertices()[valid_vertex_idx[b_idx]]->OutHalfEdges()) {
        if (he->is_on_boundary) {
            boundary_length_segment.emplace_back(he->Edge()->calculate_length());
            break;
        }
    }
}

void HEMeshInherit::select_corners_related_with_boundary_length(const std::vector<int> &valid_vertex_idx,
                                                                const std::vector<int> &order_idx,
                                                                const Eigen::Matrix<BT, 2, 4> &corners,
                                                                const Eigen::MatrixX<BT> &projection_2d_pts,
                                                                BT total_boundary_length, std::vector<int> &corner_idx,
                                                                double rectangular_corner_weight) {
    // calculate four edges of rectangular
    std::vector<BT> lengths(2);
    for (int i = 0, j = 1; i < 2; ++i, ++j) {
        lengths[i] = (corners(Eigen::all, j) - corners(Eigen::all, i)).norm();
    }
    BT length_scale_factor = total_boundary_length / (lengths[0] + lengths[1]) * 2;
    BT length_percentage = lengths[0] / (lengths[0] + lengths[1]);
    lengths.resize(4);
    for (int i = 0; i < 4; ++i) {
        if ((i % 2) == 0) {
            lengths[i] = length_percentage * total_boundary_length / 2;
        } else {
            lengths[i] = (1 - length_percentage) * total_boundary_length / 2;
        }
    }
    BT init_length = 0.5 * lengths[0];
    BT length_segment = 0.5 * (lengths[0] + lengths[1]);
    int b_idx = corner_idx[order_idx[0]];
    BT passed_length = 0;
    std::vector<BT> cost;
    std::vector<int> idx;
    for (int i = 1, j = order_idx[0] + 1; i < 4; ++i, ++j) { // loop 3 times
        if (j == 4) {
            j = 0;
        }
        BT passed_seg_bdy_length = 0;
        cost.clear();
        idx.clear();
        while (passed_seg_bdy_length <= length_segment) {
            for (auto *he : Vertices()[valid_vertex_idx[b_idx]]->OutHalfEdges()) {
                if (he->is_on_boundary) {
                    b_idx = he->End()->signature;
                    passed_length += he->Edge()->calculate_length();
                    if (passed_length > init_length) {
                        passed_seg_bdy_length += he->Edge()->calculate_length();
                        cost.push_back(
                            abs(length_segment - passed_seg_bdy_length) +
                            rectangular_corner_weight * length_scale_factor *
                                (projection_2d_pts(Eigen::all, valid_vertex_idx[b_idx]) - corners(Eigen::all, i))
                                    .norm());
                        idx.push_back(b_idx);
                    }
                }
            }
        }
        std::vector<int> idx_tmp(idx.size());
        std::iota(idx_tmp.begin(), idx_tmp.end(), 0);
        std::sort(idx_tmp.begin(), idx_tmp.end(),
                  [&](const int &a, const int &b) -> bool { return (cost[a] < cost[b]); });
        corner_idx[j] = idx[idx_tmp[0]];
    }
}

/**
 * Output tools
 */

std::ostream &operator<<(std::ostream &os, const V *v) {
    os << "v" << v->signature;
    return os;
}

std::ostream &operator<<(std::ostream &os, HEMesh<TraitsVEPH>::H *he) {
    os << he->Origin() << "->" << he->End();
    return os;
}

void Print(const std::shared_ptr<HEMesh<TraitsVEPH>> &mesh) {
    std::cout << (mesh->IsValid() ? "[valid]" : "[not valid]") << std::endl;
    std::cout << " V:" << mesh->Vertices().size() << std::endl;
    for (auto *v : mesh->Vertices())
        std::cout << "    " << v << std::endl;

    std::cout << "HE:" << mesh->HalfEdges().size() << std::endl;
    for (auto *he : mesh->HalfEdges())
        std::cout << "    " << he << std::endl;

    std::cout << " E:" << mesh->Edges().size() << std::endl;
    for (auto *e : mesh->Edges())
        std::cout << "    " << e->signature_name() << std::endl;

    std::cout << " P:" << mesh->Polygons().size() << std::endl << std::endl;
    for (auto *p : mesh->Polygons())
        std::cout << "    " << p->signature_name() << std::endl;

    std::cout << " B:" << mesh->Boundaries().size() << std::endl;
    for (auto *b : mesh->Boundaries()) {
        std::cout << "    ";
        for (auto *he : b->NextLoop())
            std::cout << he->Origin() << "-";
        std::cout << b->Origin() << std::endl;
    }
    std::cout << std::endl;
}

} // namespace Ubpa