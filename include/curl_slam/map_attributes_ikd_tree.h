//
// Created by zkc on 08/06/23.
//

#ifndef SRC_MAP_ATTRIBUTES_IKD_TREE_H
#define SRC_MAP_ATTRIBUTES_IKD_TREE_H

#include "PatchProcession.h"
#include "ikd-Tree/ikd_Tree.h"
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
// ToDo: check applicability of this data structure
template <typename BasicType> struct PatchInfo;

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
        update();      // update pointers to remove dead pointers
        return local_patches.size();
    };
    void update() {
        for (auto itr = local_patches.begin(); itr != local_patches.end();) {
            if (itr->lock()) {
                ++itr;                          // if the object is still alive, move to the next one
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
              std::shared_ptr<PatchProcession<BasicType>> _patch_procession)
        : keyframe_ptr(_keyframe_ptr), T_obj_lidar(std::move(_T_obj_lidar)), patch_procession(_patch_procession) {}
    std::shared_ptr<KeyframeInfo<BasicType>> keyframe_ptr;
    Eigen::Isometry3d T_obj_lidar;
    ikdTree_PointType centroid_w;
    // ToDo: this need to be replaced with PatchProcession
    std::shared_ptr<PatchProcession<BasicType>> patch_procession;
};

struct ikdTree_PointType_Hasher {
    std::size_t operator()(const ikdTree_PointType &vec) const {
        std::size_t hx = std::hash<float>()(vec.x);
        std::size_t hy = std::hash<float>()(vec.y) << 1;
        std::size_t hz = std::hash<float>()(vec.z) << 2;
        return (hx ^ hy) ^ hz;
    }
};

// Goals: automatically update the all_patches_map vector and kd-tree
// Goals: Check ikd-tree interface, search radius, search nearest neighbours, add points and remove
// Goals: keyframe can get patches and define the updation interface, keyframe deletion interface
// points

KD_TREE<ikdTree_PointType> kd_tree_patch_centroids;

template <typename BasicType> class MapAttributes {
  public:
    MapAttributes() {
        // initialize the ikd-tree
        KD_TREE<ikdTree_PointType>::PointVector point_clouds;
        ikdTree_PointType pts;
        pts.x = 0;
        pts.y = 0;
        pts.z = 0;
        point_clouds.push_back(pts);
        kd_tree_patch_centroids.Build(point_clouds);
        kd_tree_patch_centroids.Delete_Point(pts);
    }
    void insert(std::shared_ptr<PatchInfo<BasicType>> &patch) {
        all_patches_vec.push_back(patch);
        all_keyframes_set.insert(patch->keyframe_ptr);
        std::weak_ptr<PatchInfo<BasicType>> weak_patch = patch;
        patch->keyframe_ptr->local_patches.insert(weak_patch);
        patch->centroid_w = Eigen::Vector3f(
            (patch->keyframe_ptr->T_w_lidar * patch->T_obj_lidar.inverse()).translation().template cast<float>());
        kd_tree_patch_centroids.Add_Point(patch->centroid_w, false);
        assert(!contains(patch->centroid_w));
        all_patches_map.emplace(patch->centroid_w, static_cast<int>(all_patches_vec.size() - 1));
    }

    void erase(const std::shared_ptr<PatchInfo<BasicType>> &patch) {
        ikdTree_PointType centroid = Eigen::Vector3f(
            (patch->keyframe_ptr->T_w_lidar * patch->T_obj_lidar.inverse()).translation().template cast<float>());
        auto target = all_patches_map.find(centroid);
        if (target == all_patches_map.end()) {
            std::cerr << "erase target doesn't exist!" << std::endl;
            return;
        }
        kd_tree_patch_centroids.Delete_Point(centroid);
        int idx = target->second;
        if (idx != (all_patches_vec.size() - 1)) {
            ikdTree_PointType back_centroid = Eigen::Vector3f(
                (all_patches_vec.back()->keyframe_ptr->T_w_lidar * all_patches_vec.back()->T_obj_lidar.inverse())
                    .translation()
                    .template cast<float>());
            all_patches_map[back_centroid] = idx;
            all_patches_vec[idx] = all_patches_vec.back();
        }
        all_patches_vec.pop_back();
        all_patches_map.erase(target);
        patch->keyframe_ptr->update();
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
        ikdTree_PointType centroid = Eigen::Vector3f(
            (all_patches_vec[idx]->keyframe_ptr->T_w_lidar * all_patches_vec[idx]->T_obj_lidar.inverse())
                .translation()
                .template cast<float>());
        auto target = all_patches_map.find(centroid);
        kd_tree_patch_centroids.Delete_Point(centroid);
        if (idx != (all_patches_vec.size() - 1)) {
            ikdTree_PointType back_centroid = Eigen::Vector3f(
                (all_patches_vec.back()->keyframe_ptr->T_w_lidar * all_patches_vec.back()->T_obj_lidar.inverse())
                    .translation()
                    .template cast<float>());
            all_patches_map[back_centroid] = idx;
            all_patches_vec[idx] = all_patches_vec.back();
        }
        all_patches_vec.pop_back();
        all_patches_map.erase(target);
        patch->keyframe_ptr->update();
        if (patch->keyframe_ptr->local_patches.empty()) {
            all_keyframes_set.erase(patch->keyframe_ptr);
        }
    }

    // ToDo: add method to update the map
    void update(PatchInfo<BasicType> &map_info) {}

    auto begin() noexcept { return all_patches_vec.begin(); }

    auto begin() const noexcept { return all_patches_vec.begin(); }

    auto end() noexcept { return all_patches_vec.end(); }

    auto end() const noexcept { return all_patches_vec.end(); }

    auto back() noexcept { return all_patches_vec.back(); }

    auto back() const noexcept { return all_patches_vec.back(); }

    std::shared_ptr<PatchInfo<BasicType>> &operator[](int i) { return all_patches_vec[i]; }

    const std::shared_ptr<PatchInfo<BasicType>> &operator[](std::size_t i) const { return all_patches_vec[i]; }

    std::shared_ptr<PatchInfo<BasicType>> &at(int i) { return all_patches_vec.at(i); }

    const std::shared_ptr<PatchInfo<BasicType>> &at(int i) const { return all_patches_vec.at(i); }

    unsigned long size() const noexcept { return all_patches_vec.size(); }

    void reserve(int n) {
        all_patches_vec.reserve(n);
        all_patches_map.reserve(n);
    }

    void clear() noexcept {
        for (auto &patch : all_patches_vec) {
            kd_tree_patch_centroids.Delete_Point(patch->centroid_w);
        }
        all_patches_vec.clear();
        all_patches_map.clear();
    }

    bool empty() const noexcept { return all_patches_vec.empty(); }

    bool contains(const ikdTree_PointType &ele) const { return all_patches_map.find(ele) != all_patches_map.end(); }

    // kd-tree search
    void radius_search(const Eigen::Vector3f &point, float radius, std::vector<int> &indices) {
        ikdTree_PointType point_kd = point;
        KD_TREE<ikdTree_PointType>::PointVector results;
        kd_tree_patch_centroids.Radius_Search(point_kd, radius, results);
        if (!results.empty()) {
            for (auto result : results) {
                indices.push_back(all_patches_map[result]);
            }
        }
    }

    void radius_search(const ikdTree_PointType &point_kd, float radius, std::vector<int> &indices) {
        KD_TREE<ikdTree_PointType>::PointVector results;
        kd_tree_patch_centroids.Radius_Search(point_kd, radius, results);
        if (!results.empty()) {
            for (auto result : results) {
                indices.push_back(all_patches_map[result]);
            }
        }
    }

    void nearest_search(const Eigen::Vector3f &point, int k_nearest, std::vector<int> &indices,
                        double max_dist = INFINITY) {
        ikdTree_PointType point_kd = point;
        KD_TREE<ikdTree_PointType>::PointVector results;
        std::vector<float> square_distance;
        kd_tree_patch_centroids.Nearest_Search(point_kd, k_nearest, results, square_distance, max_dist);
        if (!results.empty()) {
            for (auto result : results) {
                indices.push_back(all_patches_map[result]);
            }
        }
    }

    void nearest_search(const ikdTree_PointType &point_kd, int k_nearest, std::vector<int> &indices,
                        double max_dist = INFINITY) {
        KD_TREE<ikdTree_PointType>::PointVector results;
        std::vector<float> square_distance;
        kd_tree_patch_centroids.Nearest_Search(point_kd, k_nearest, results, square_distance, max_dist);
        if (!results.empty()) {
            for (auto result : results) {
                indices.push_back(all_patches_map[result]);
            }
        }
    }

  private:
    std::unordered_map<ikdTree_PointType, int, ikdTree_PointType_Hasher> all_patches_map;
    // the benefit of using vector and idx to be the second value of
    // std::unorder_map is it can access patch by using index directly
    std::vector<std::shared_ptr<PatchInfo<BasicType>>> all_patches_vec;
    // the comparator make the keyframes has the order of time
    std::set<std::shared_ptr<KeyframeInfo<BasicType>>, key_frame_info_comparator<BasicType>> all_keyframes_set;
};

#endif // SRC_MAP_ATTRIBUTES_IKD_TREE_H