//
// Created by zkc on 08/06/23.
//

#ifndef SRC_MAP_ATTRIBUTES_IKD_TREE_H
#define SRC_MAP_ATTRIBUTES_IKD_TREE_H

#include "AABB_tree/AABB.h"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "PatchProcession.h"
#include "curl_tools_light.h"
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
// declaration
template <typename BasicType> struct PatchInfo;
// data-association structure

struct MapDataAsso {
    std::vector<int> points_idx_vec;
    std::vector<double> IoU_vec;
    std::vector<double> centroid_distance_vec;
    std::vector<double> scores;
    void calculate_score(double weight_of_IoU, double largest_distance) {
        if (scores.empty()) {
            scores.reserve(points_idx_vec.size());
            for (int i = 0; i < points_idx_vec.size(); ++i) {
                scores.emplace_back(weight_of_IoU * IoU_vec[i] +
                                    (1 - weight_of_IoU) *
                                        (1 - std::min(centroid_distance_vec[i], largest_distance) / largest_distance));
            }
        }
    }
    std::vector<int> sort_according_to_score(double weight_of_IoU, double largest_distance) {
        calculate_score(weight_of_IoU, largest_distance);
        std::vector<int> idx_vec(scores.size());
        std::iota(idx_vec.begin(), idx_vec.end(), 0); // fill with 0, 1, ..., scores.size() - 1
        std::sort(idx_vec.begin(), idx_vec.end(), [&](int a, int b) { return scores[a] > scores[b]; });
        return idx_vec;
    }
    void clear() {
        points_idx_vec.clear();
        IoU_vec.clear();
        centroid_distance_vec.clear();
        scores.clear();
    }
};

template <typename BasicType> struct ScanDataAsso {
    ScanDataAsso() = delete;
    ScanDataAsso(int _point_idx) : point_idx(_point_idx) {}
    int point_idx;
    std::vector<std::shared_ptr<PatchInfo<BasicType>>> patch_info_ptr_vec;
    std::vector<double> scores;
    // this is not calculate score, this is access the score from MapDataAsso
    void calculate_score() {
        if (scores.empty()) {
            scores.reserve(patch_info_ptr_vec.size());
            for (const auto &patch_info_ptr : patch_info_ptr_vec) {
                assert(patch_info_ptr->data_asso.points_idx_vec.size() > 0);
                for (int i = 0; i < patch_info_ptr->data_asso.points_idx_vec.size(); ++i) {
                    if (patch_info_ptr->data_asso.points_idx_vec[i] == point_idx) {
                        scores.push_back(patch_info_ptr->data_asso.scores[i]);
                        break;
                    }
                }
            }
        }
    }
    std::vector<int> sort_according_to_score() {
        calculate_score();
        std::vector<int> idx_vec(patch_info_ptr_vec.size());
        std::iota(idx_vec.begin(), idx_vec.end(), 0);
        std::sort(idx_vec.begin(), idx_vec.end(), [&](int a, int b) { return scores[a] > scores[b]; });
        return idx_vec;
    }
    void clear() {
        for (auto &patch_info_ptr : patch_info_ptr_vec) {
            patch_info_ptr->data_asso.clear();
        }
        patch_info_ptr_vec.clear();
        scores.clear();
    }
};

// map data structure

// define the hash function rules for std::weak_ptr
template <typename BasicType> struct weak_ptr_hash {
    std::size_t operator()(const std::weak_ptr<PatchInfo<BasicType>> &wp) const {
        auto sp = wp.lock();
        if (!sp) {
            // Handle expired weak_ptr
            return 0;
        }
        return std::hash<std::shared_ptr<PatchInfo<BasicType>>>{}(sp);
    }
};

// define the equality std::weak_ptr
template <typename BasicType> struct weak_ptr_equal {
    bool operator()(const std::weak_ptr<PatchInfo<BasicType>> &wp1,
                    const std::weak_ptr<PatchInfo<BasicType>> &wp2) const {
        auto sp1 = wp1.lock();
        auto sp2 = wp2.lock();
        if (!sp1 || !sp2) {
            return false; // Handle expired weak_ptr
        }
        return sp1 == sp2;
    }
};

// use std::weak_ptr rather than std::shared_ptr is trying to avoid the memory leakage
template <typename BasicType> struct KeyframeInfo {
    KeyframeInfo(double _time) : time(_time) {}
    KeyframeInfo(double _time, Eigen::Isometry3d _T_w_lidar) : time(_time), T_w_lidar(_T_w_lidar) {}
    double time;
    Eigen::Isometry3d T_w_lidar;
    std::unordered_set<std::weak_ptr<PatchInfo<BasicType>>, weak_ptr_hash<BasicType>, weak_ptr_equal<BasicType>>
        local_patches; // inilized in the insert
    unsigned long size() {
        update(); // update pointers to remove dead pointers
        return local_patches.size();
    };
    void update() {
        for (auto itr = local_patches.begin(); itr != local_patches.end();) {
            if (itr->lock()) {
                ++itr; // if the object is still alive, move to the next one
            } else {
                itr = local_patches.erase(itr); // if the object is dead, erase it and update the iterator
            }
        }
    }
};

template <typename BasicType> struct key_frame_info_comparator {
    bool operator()(const std::shared_ptr<KeyframeInfo<BasicType>> &lhs,
                    const std::shared_ptr<KeyframeInfo<BasicType>> &rhs) const {
        return lhs->time < rhs->time;
    }
};

template <typename BasicType> struct PatchInfo {
    PatchInfo(const std::shared_ptr<KeyframeInfo<BasicType>> &_keyframe_ptr, Eigen::Isometry3d _T_obj_lidar,
              std::shared_ptr<PatchProcession<BasicType>> _patch_procession_ptr, int _status = curl::GOOD)
        : keyframe_ptr(_keyframe_ptr), T_obj_lidar(std::move(_T_obj_lidar)),
          patch_procession_ptr(_patch_procession_ptr), status(_status) {
        is_map_gen = false;
        is_ground = false;
        color = std::array<float, 3>{1, 0, 0};
        data_asso_counter = 0;
    }
    // for odometry
    PatchInfo(const std::shared_ptr<KeyframeInfo<BasicType>> &_keyframe_ptr, Eigen::Isometry3d _T_obj_lidar,
              std::shared_ptr<PatchProcession<BasicType>> _patch_procession_ptr, bool _is_ground,
              std::array<float, 3> _color = std::array<float, 3>{1, 0, 0}, int _status = curl::GOOD)
        : keyframe_ptr(_keyframe_ptr), T_obj_lidar(std::move(_T_obj_lidar)),
          patch_procession_ptr(_patch_procession_ptr), is_ground(_is_ground), color(_color), status(_status) {
        is_map_gen = false;
        if (is_ground) {
            color = {0.804, 0.522, 0.247};
        }
        data_asso_counter = 0;
    }

    // for map generation
    PatchInfo(Eigen::Isometry3d _map_gen_T_w_obj, std::shared_ptr<PatchProcession<BasicType>> _patch_procession_ptr,
              std::array<float, 3> _color = std::array<float, 3>{1, 0, 0}, int _status = curl::GOOD)
        : map_gen_T_w_obj(std::move(_map_gen_T_w_obj)), patch_procession_ptr(_patch_procession_ptr), color(_color),
          status(_status) {
        is_ground = false;
        is_map_gen = true;
        if (is_ground) {
            color = {0.804, 0.522, 0.247};
        }
        data_asso_counter = 0;
    }

    PatchInfo(Eigen::Isometry3d _map_gen_T_w_obj, std::shared_ptr<PatchProcession<BasicType>> _patch_procession_ptr,
              bool _is_ground, std::array<float, 3> _color = std::array<float, 3>{1, 0, 0}, int _status = curl::GOOD)
        : map_gen_T_w_obj(std::move(_map_gen_T_w_obj)), patch_procession_ptr(_patch_procession_ptr),
          is_ground(_is_ground), color(_color), status(_status) {
        is_map_gen = true;
        if (is_ground) {
            color = {0.804, 0.522, 0.247};
        }
        data_asso_counter = 0;
    }

    std::shared_ptr<KeyframeInfo<BasicType>> keyframe_ptr;
    Eigen::Isometry3d T_obj_lidar;
    Eigen::Isometry3d map_gen_T_w_obj;
    unsigned int key;
    bool is_ground;
    bool is_map_gen;
    int status;
    std::array<float, 3> color;
    std::shared_ptr<PatchProcession<BasicType>> patch_procession_ptr;
    unsigned int data_asso_counter;
    Eigen::Vector3d get_centroid_w() {
        if (!is_map_gen) {
            return ((keyframe_ptr->T_w_lidar * T_obj_lidar.inverse()).translation());
        } else {
            return (map_gen_T_w_obj.translation());
        }
    }
    MapDataAsso data_asso;
};

// Goals: automatically update the all_patches_map vector and kd-tree
// Goals: keyframe can get patches and define the updation interface, keyframe deletion interface
// points

template <typename BasicType> class MapAttributes {
  public:
    MapAttributes(double skin_thickness, double _extra_skin_thickness, double _ground_fatten_scale,
                  bool touch_is_overlap)
        : extra_skin_thickness(_extra_skin_thickness), ground_fatten_scale(_ground_fatten_scale) {
        tree_key = 0;
        // initialize aabb tree
        aabb_tree_ptr = std::make_shared<aabb::Tree>(3, skin_thickness, 1, touch_is_overlap);
    }

    void insert(std::shared_ptr<PatchInfo<BasicType>> &patch, std::vector<double> &lower_bound,
                std::vector<double> &upper_bound) {
        ++tree_key;
        all_patches_vec.push_back(patch);
        patch->key = tree_key;
        all_keyframes_set.insert(patch->keyframe_ptr);
        std::weak_ptr<PatchInfo<BasicType>> weak_patch = patch;
        patch->keyframe_ptr->local_patches.insert(weak_patch);
        // insert the patch into aabb tree
        aabb_tree_ptr->insertParticle(patch->key, extra_skin_thickness, lower_bound, upper_bound);
        assert(!contains(patch->key));
        all_patches_map.emplace(patch->key, static_cast<unsigned int>(all_patches_vec.size() - 1));
    }

    void insert_ground(std::shared_ptr<PatchInfo<BasicType>> &patch, std::vector<double> &lower_bound,
                       std::vector<double> &upper_bound) {
        ++tree_key;
        all_patches_vec.push_back(patch);
        patch->key = tree_key;
        all_keyframes_set.insert(patch->keyframe_ptr);
        std::weak_ptr<PatchInfo<BasicType>> weak_patch = patch;
        patch->keyframe_ptr->local_patches.insert(weak_patch);
        // insert the patch into aabb tree
        aabb_tree_ptr->insertGroundParticle(patch->key, extra_skin_thickness, ground_fatten_scale, lower_bound,
                                            upper_bound);
        assert(!contains(patch->key));
        all_patches_map.emplace(patch->key, static_cast<unsigned int>(all_patches_vec.size() - 1));
    }

    void erase(const std::shared_ptr<PatchInfo<BasicType>> &patch) {
        auto target = all_patches_map.find(patch->key);
        if (target == all_patches_map.end()) {
            std::cerr << "erase target doesn't exist!" << std::endl;
            return;
        }
        aabb_tree_ptr->removeParticle(patch->key);
        unsigned int idx = target->second;
        if (idx != (all_patches_vec.size() - 1)) {
            all_patches_map[all_patches_vec.back()->key] = idx;
            all_patches_vec[idx] = all_patches_vec.back();
        }
        all_patches_vec.pop_back();
        all_patches_map.erase(target);
        std::weak_ptr<PatchInfo<BasicType>> weak_patch = patch;
        patch->keyframe_ptr->local_patches.erase(weak_patch);
        if (patch->keyframe_ptr->local_patches.empty()) {
            all_keyframes_set.erase(patch->keyframe_ptr);
        }
    }

    void erase(int idx) {
        if (idx >= all_patches_vec.size()) {
            std::cerr << "erase target doesn't exist!" << std::endl;
            return;
        }
        std::shared_ptr<PatchInfo<BasicType>> patch = all_patches_vec[idx];
        aabb_tree_ptr->removeParticle(patch->key);
        auto target = all_patches_map.find(patch->key);
        if (idx != (all_patches_vec.size() - 1)) {
            all_patches_map[all_patches_vec.back()->key] = idx;
            all_patches_vec[idx] = all_patches_vec.back();
        }
        all_patches_vec.pop_back();
        all_patches_map.erase(target);
        std::weak_ptr<PatchInfo<BasicType>> weak_patch = patch;
        patch->keyframe_ptr->local_patches.erase(weak_patch);
        if (patch->keyframe_ptr->local_patches.empty()) {
            all_keyframes_set.erase(patch->keyframe_ptr);
        }
    }

    typename std::set<std::shared_ptr<KeyframeInfo<BasicType>>, key_frame_info_comparator<BasicType>>::iterator
    erase_keyframe(const std::shared_ptr<KeyframeInfo<BasicType>> &keyframe_ptr) {
        if (all_keyframes_set.find(keyframe_ptr) == all_keyframes_set.end()) {
            std::cerr << "erase keyframe target doesn't exist!" << std::endl;
            return all_keyframes_set.begin();
        }
        for (auto iter = keyframe_ptr->local_patches.begin(); iter != keyframe_ptr->local_patches.end();) {
            if (auto patch_shared_ptr = iter->lock()) {
                this->erase(patch_shared_ptr);
                iter = keyframe_ptr->local_patches.begin();
            } else {
                ++iter;
            }
        }
        all_keyframes_set.erase(keyframe_ptr);
        typename std::set<std::shared_ptr<KeyframeInfo<BasicType>>, key_frame_info_comparator<BasicType>>::iterator
            iter = all_keyframes_set.begin();
        return iter;
    }

    // ToDo: add method to update the map
    void update(PatchInfo<BasicType> &map_info) {}

    auto begin() noexcept { return all_patches_vec.begin(); }

    auto begin() const noexcept { return all_patches_vec.begin(); }

    auto end() noexcept { return all_patches_vec.end(); }

    auto end() const noexcept { return all_patches_vec.end(); }

    auto back() noexcept { return all_patches_vec.back(); }

    auto back() const noexcept { return all_patches_vec.back(); }

    auto keyframe_begin() noexcept { return all_keyframes_set.begin(); }

    auto keyframe_begin() const noexcept { return all_keyframes_set.begin(); }

    auto keyframe_end() noexcept { return all_keyframes_set.end(); }

    auto keyframe_end() const noexcept { return all_keyframes_set.end(); }

    std::shared_ptr<PatchInfo<BasicType>> &operator[](unsigned int key) {
        unsigned int i = all_patches_map[key];
        return all_patches_vec[i];
    }

    const std::shared_ptr<PatchInfo<BasicType>> &operator[](unsigned int key) const {
        unsigned int i = all_patches_map.at(key);
        return all_patches_vec[i];
    }

    std::shared_ptr<PatchInfo<BasicType>> &at(int i) { return all_patches_vec.at(i); }

    const std::shared_ptr<PatchInfo<BasicType>> &at(int i) const { return all_patches_vec.at(i); }

    unsigned long size() const noexcept { return all_patches_vec.size(); }
    unsigned long keyframe_size() const noexcept { return all_keyframes_set.size(); }

    void reserve(int n) {
        all_patches_vec.reserve(n);
        all_patches_map.reserve(n);
    }

    void clear() noexcept {
        all_patches_vec.clear();
        all_patches_map.clear();
        aabb_tree_ptr->removeAll();
        all_keyframes_set.clear();
        centroids_w_vec.clear();
        kd_tree_idx_ptr.reset();
        tree_key = 0;
    }

    bool empty() const noexcept { return all_patches_vec.empty(); }

    bool contains(const unsigned int &key) const { return all_patches_map.find(key) != all_patches_map.end(); }

    // aabb-tree overlap
    std::vector<unsigned int> query(unsigned int key) { return aabb_tree_ptr->query(key); }

    std::vector<unsigned int> query(aabb::AABB &bounding_box) { return aabb_tree_ptr->query(bounding_box); }

    std::vector<unsigned int> query(std::vector<double> &lower_bound, std::vector<double> &upper_bound) {
        aabb::AABB bounding_box(lower_bound, upper_bound);
        return aabb_tree_ptr->query(bounding_box);
    }

    std::vector<std::pair<unsigned int, double>> query_data_association_overlap(aabb::AABB &bounding_box) {
        std::vector<std::pair<unsigned int, double>> result;
        std::vector<unsigned int> keys = aabb_tree_ptr->query(bounding_box);
        if (!keys.empty()) {
            for (auto &key : keys) {
                aabb::AABB intersected_box = aabb_tree_ptr->getAABB(key);
                // calculate the IoU of two intersected bounding box
                double IoU = curl::IoU_3d(bounding_box.lowerBound, bounding_box.upperBound, intersected_box.lowerBound,
                                          intersected_box.upperBound);
                result.emplace_back(key, IoU);
            }
        }
        if (!result.empty()) {
            // descending order, largest value be the first
            std::sort(result.begin(), result.end(),
                      [](const std::pair<unsigned int, double> &a, const std::pair<unsigned int, double> &b) {
                          return a.second > b.second;
                      });
        }
        return result;
    }

    std::vector<std::pair<unsigned int, double>> query_data_association_overlap(std::vector<double> &lower_bound,
                                                                                std::vector<double> &upper_bound) {
        aabb::AABB bounding_box(lower_bound, upper_bound);
        std::vector<std::pair<unsigned int, double>> result;
        std::vector<unsigned int> keys = aabb_tree_ptr->query(bounding_box);
        if (!keys.empty()) {
            for (auto &key : keys) {
                aabb::AABB intersected_box = aabb_tree_ptr->getAABB(key);
                // calculate the IoU of two intersected bounding box
                double IoU = curl::IoU_3d(bounding_box.lowerBound, bounding_box.upperBound, intersected_box.lowerBound,
                                          intersected_box.upperBound);
                result.emplace_back(key, IoU);
            }
        }
        if (!result.empty()) {
            // descending order, largest value be the first
            std::sort(result.begin(), result.end(),
                      [](const std::pair<unsigned int, double> &a, const std::pair<unsigned int, double> &b) {
                          return a.second > b.second;
                      });
        }
        return result;
    }

    aabb::AABB get_AABB(unsigned int key) { return (aabb_tree_ptr->getAABB(key)); }

    unsigned int get_curr_tree_key() { return tree_key; }

    /***************** for map generation *****************/
    /***************** for map generation *****************/
    /***************** for map generation *****************/
    bool inv_contains(const unsigned int &idx) const {
        return inv_all_patches_map.find(idx) != inv_all_patches_map.end();
    }

    void map_gen_insert(std::shared_ptr<PatchInfo<BasicType>> &patch, std::vector<double> &lower_bound,
                        std::vector<double> &upper_bound) {
        ++tree_key;
        all_patches_vec.push_back(patch);
        patch->key = tree_key;
        // insert the patch into aabb tree
        aabb_tree_ptr->insertParticle(patch->key, extra_skin_thickness, lower_bound, upper_bound);
        assert(!contains(patch->key));
        all_patches_map.emplace(patch->key, static_cast<unsigned int>(all_patches_vec.size() - 1));
        // add inverse map
        assert(!inv_contains(static_cast<unsigned int>(all_patches_vec.size() - 1)));
        inv_all_patches_map.emplace(static_cast<unsigned int>(all_patches_vec.size() - 1), patch->key);
        // insert centroid_w of patch into kd-tree
        centroids_w_vec.push_back((patch->map_gen_T_w_obj.translation()).template cast<BasicType>());
    }

    void map_gen_insert_ground(std::shared_ptr<PatchInfo<BasicType>> &patch, std::vector<double> &lower_bound,
                               std::vector<double> &upper_bound) {
        ++tree_key;
        all_patches_vec.push_back(patch);
        patch->key = tree_key;
        // insert the patch into aabb tree
        aabb_tree_ptr->insertGroundParticle(patch->key, extra_skin_thickness, ground_fatten_scale, lower_bound,
                                            upper_bound);
        assert(!contains(patch->key));
        all_patches_map.emplace(patch->key, static_cast<unsigned int>(all_patches_vec.size() - 1));
        // add inverse map
        assert(!inv_contains(static_cast<unsigned int>(all_patches_vec.size() - 1)));
        inv_all_patches_map.emplace(static_cast<unsigned int>(all_patches_vec.size() - 1), patch->key);
        // insert centroid_w of patch into kd-tree
        centroids_w_vec.push_back((patch->map_gen_T_w_obj.translation()).template cast<BasicType>());
    }

    void map_gen_insert(std::shared_ptr<PatchInfo<BasicType>> &patch) {
        ++tree_key;
        all_patches_vec.push_back(patch);
        patch->key = tree_key;
        assert(!contains(patch->key));
        all_patches_map.emplace(patch->key, static_cast<unsigned int>(all_patches_vec.size() - 1));
        // add inverse map
        assert(!inv_contains(static_cast<unsigned int>(all_patches_vec.size() - 1)));
        inv_all_patches_map.emplace(static_cast<unsigned int>(all_patches_vec.size() - 1), patch->key);
        // insert centroid_w of patch into kd-tree
        centroids_w_vec.push_back((patch->map_gen_T_w_obj.translation()).template cast<BasicType>());
    }

    void map_gen_insert_ground(std::shared_ptr<PatchInfo<BasicType>> &patch) {
        ++tree_key;
        all_patches_vec.push_back(patch);
        patch->key = tree_key;
        assert(!contains(patch->key));
        all_patches_map.emplace(patch->key, static_cast<unsigned int>(all_patches_vec.size() - 1));
        // add inverse map
        assert(!inv_contains(static_cast<unsigned int>(all_patches_vec.size() - 1)));
        inv_all_patches_map.emplace(static_cast<unsigned int>(all_patches_vec.size() - 1), patch->key);
        // insert centroid_w of patch into kd-tree
        centroids_w_vec.push_back((patch->map_gen_T_w_obj.translation()).template cast<BasicType>());
    }

    void initlize_kd_tree() { kd_tree_idx_ptr = std::make_unique<my_kd_tree_type>(3, centroids_w_vec); }

    std::vector<std::shared_ptr<PatchInfo<BasicType>>> radius_search_patches(const Eigen::Vector3<BasicType> &center,
                                                                             BasicType radius) {
        nanoflann::SearchParams params;
        params.sorted = false;
        std::vector<std::pair<size_t, BasicType>> ret_matches;
        kd_tree_idx_ptr->index->radiusSearch(center.data(), radius * radius, ret_matches, params);
        std::vector<std::shared_ptr<PatchInfo<BasicType>>> valid_patches;
        for (auto &pair : ret_matches) {
            valid_patches.push_back((*this)[inv_all_patches_map[pair.first]]);
        }
        return valid_patches;
    }

  private:
    // a map between patch key in aabb tree and the index of all_patches_vec
    std::unordered_map<unsigned int, unsigned int> all_patches_map; // key -> idx
    unsigned int tree_key;
    // aabb tree
    std::shared_ptr<aabb::Tree> aabb_tree_ptr;
    // the benefit of using vector and idx to be the second value of
    // std::unorder_map is it can access patch by using index directly
    std::vector<std::shared_ptr<PatchInfo<BasicType>>> all_patches_vec;
    // the comparator make the keyframes has the order of time
    std::set<std::shared_ptr<KeyframeInfo<BasicType>>, key_frame_info_comparator<BasicType>> all_keyframes_set;

    /***************** for map generation *****************/
    /***************** for map generation *****************/
    /***************** for map generation *****************/
    std::unordered_map<unsigned int, unsigned int> inv_all_patches_map; // idx -> key
    // a kd-tree map for map generation radius search
    std::vector<Eigen::Vector3<BasicType>> centroids_w_vec;
    using my_kd_tree_type = KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3<BasicType>>, BasicType, 3>;
    std::unique_ptr<my_kd_tree_type> kd_tree_idx_ptr;
    double extra_skin_thickness;
    double ground_fatten_scale;
};

#endif // SRC_MAP_ATTRIBUTES_IKD_TREE_H