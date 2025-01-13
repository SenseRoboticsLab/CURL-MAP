//
// Created by zkc on 05/05/23.
//

#include "ConformalMappingSPH.h"
// constructor for odometry
template <typename BasicType>
ConformalMappingSPH<BasicType>::ConformalMappingSPH(
    ros::NodeHandle *nh, std::shared_ptr<CONFORMAL_MAPPING_CONFIG> _conformal_mapping_config_ptr,
    std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
    std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr, std::shared_ptr<AABB_CONFIG> _aabb_config_ptr,
    std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr)
    : conformal_mapping_config_ptr(_conformal_mapping_config_ptr), SH_table_config_ptr(_SH_table_config_ptr),
      direct_method_config_ptr(_direct_method_config_ptr), aabb_config_ptr(_aabb_config_ptr),
      debug_config_ptr(_debug_config_ptr), rt_segment_pt_sub(*nh, "/patchworkpp/ground_pc", 10),
      rt_non_ground_pt_sub(*nh, "/patchworkpp/nonground_pc", 10), rt_ground_pt_sub(*nh, "/patchworkpp/ground_pc", 10) {
    seg_cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    ground_cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    pub_pt_valid_patches = nh->advertise<sensor_msgs::PointCloud2>("/curl/valid_patches", 10);
    pub_landmark_cloud = nh->advertise<sensor_msgs::PointCloud2>("/curl/landmark_cloud", 10);
    pub_landmark_patch = nh->advertise<visualization_msgs::Marker>("/curl/landmark_patch", 10);
    if (!conformal_mapping_config_ptr->is_rt) {
        segment_pt_sub = nh->subscribe<sensor_msgs::PointCloud2>(
            "/patchworkpp/segmented_pc", 10, &ConformalMappingSPH::callback_point_cloud_segments, this);
    } else {
        sync_ptr = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), rt_segment_pt_sub,
                                                                                 rt_ground_pt_sub);
        non_ground_sync_ptr = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(10), rt_non_ground_pt_sub, rt_ground_pt_sub);
        //        rt_segment_pt_sub = nh->subscribe<sensor_msgs::PointCloud2>(
        //            "/travel/segmented_pc", 10, &ConformalMappingSPH::rt_callback_point_cloud_segments, this);
        if (conformal_mapping_config_ptr->is_even_segmentation) {
            non_ground_sync_ptr->registerCallback(
                boost::bind(&ConformalMappingSPH<BasicType>::rt_callback_point_cloud_segments, this, _1, _2));
        } else {
            sync_ptr->registerCallback(
                boost::bind(&ConformalMappingSPH<BasicType>::rt_callback_point_cloud_segments, this, _1, _2));
        }
        map_attributes_ptr = std::make_shared<MapAttributes<BasicType>>(
            aabb_config_ptr->skin_thickness, aabb_config_ptr->extra_skin_thickness,
            aabb_config_ptr->ground_fatten_scale, aabb_config_ptr->touch_is_overlap);
        omp_init_lock(&writelock);
        if (debug_config_ptr->is_save_middle_results) {
            debug_config_ptr->middle_results_dir = debug_config_ptr->results_dir + "/middle_results";
            curl::create_directory_if_not_exists(debug_config_ptr->middle_results_dir);
            conformal_mapping_config_ptr->thread_num = 1;
            debug_config_ptr->patch_id = 0;
        }
    }
}

// constructor for map generation
template <typename BasicType>
ConformalMappingSPH<BasicType>::ConformalMappingSPH(
    std::shared_ptr<CONFORMAL_MAPPING_CONFIG> _conformal_mapping_config_ptr,
    std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
    std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr, std::shared_ptr<AABB_CONFIG> _aabb_config_ptr,
    std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr)
    : conformal_mapping_config_ptr(_conformal_mapping_config_ptr), SH_table_config_ptr(_SH_table_config_ptr),
      direct_method_config_ptr(_direct_method_config_ptr), aabb_config_ptr(_aabb_config_ptr),
      debug_config_ptr(_debug_config_ptr) {
    map_attributes_ptr = std::make_shared<MapAttributes<BasicType>>(
        aabb_config_ptr->skin_thickness, aabb_config_ptr->extra_skin_thickness, aabb_config_ptr->ground_fatten_scale,
        aabb_config_ptr->touch_is_overlap);
    omp_init_lock(&writelock);
}

template <typename BasicType> ConformalMappingSPH<BasicType>::~ConformalMappingSPH() { omp_destroy_lock(&writelock); }
/****************************************************************************************************/
/*********************************************Algorithms*********************************************/
/****************************************************************************************************/

template <typename BasicType>
void ConformalMappingSPH<BasicType>::divide_patches(Eigen::MatrixXf &patch, std::vector<Eigen::MatrixXf> &patch_vec,
                                                    double cut_threshold, int minimum_size) {
    float min_x = patch.row(0).minCoeff();
    float min_y = patch.row(1).minCoeff();
    float min_z = patch.row(2).minCoeff();
    float max_x = patch.row(0).maxCoeff();
    float max_y = patch.row(1).maxCoeff();
    float max_z = patch.row(2).maxCoeff();
    // for x axis
    int x_seg_num = std::ceil((max_x - min_x) / cut_threshold);
    int y_seg_num = std::ceil((max_y - min_y) / cut_threshold);
    int z_seg_num = std::ceil((max_z - min_z) / cut_threshold);

    if (x_seg_num > 1 || y_seg_num > 1 || z_seg_num > 1) {
        std::vector<std::vector<int>> patch_idx_vec(x_seg_num * y_seg_num * z_seg_num);
        float new_x_cut_threshold = (max_x - min_x) / static_cast<float>(x_seg_num);
        float new_y_cut_threshold = (max_y - min_y) / static_cast<float>(y_seg_num);
        float new_z_cut_threshold = (max_z - min_z) / static_cast<float>(z_seg_num);
        for (int i = 0; i < patch.cols(); ++i) {
            int x_idx = std::floor((patch(0, i) - min_x) / new_x_cut_threshold);
            if (x_idx >= x_seg_num) {
                x_idx = x_seg_num - 1;
            }
            int y_idx = std::floor((patch(1, i) - min_y) / new_y_cut_threshold);
            if (y_idx >= y_seg_num) {
                y_idx = y_seg_num - 1;
            }

            int z_idx = std::floor((patch(2, i) - min_z) / new_z_cut_threshold);
            if (z_idx >= z_seg_num) {
                z_idx = z_seg_num - 1;
            }
            int grid_index = z_idx * (x_seg_num * y_seg_num) + y_idx * x_seg_num + x_idx;
            patch_idx_vec[grid_index].push_back(i);
        }
        patch_vec.clear();
        patch_vec.reserve(x_seg_num * y_seg_num * z_seg_num);
        for (const auto &patch_idx : patch_idx_vec) {
            if (!patch_idx.empty() && patch_idx.size() >= minimum_size) {
                patch_vec.emplace_back(patch(Eigen::all, patch_idx));
            }
        }
    }
}

template <typename BasicType>
template <typename T>
void ConformalMappingSPH<BasicType>::radius_outlier_removal(typename pcl::PointCloud<T>::Ptr cloud_in_ptr,
                                                            float radius_per_meter, int min_neighbours) {
    pcl::KdTreeFLANN<T> kdtree;
    kdtree.setInputCloud(cloud_in_ptr);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    std::vector<int> valid_idx;
    typename pcl::PointCloud<T>::Ptr cloud_out_tmp_ptr(new pcl::PointCloud<T>);
    int i = 0;
    for (auto point : cloud_in_ptr->points) {
        float ranges = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        auto search_radius = static_cast<float>(radius_per_meter * ranges + 1e-06);
        if (kdtree.radiusSearch(point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >=
            min_neighbours) {
            //            cloud_out_tmp_ptr->push_back(point);
            valid_idx.push_back(i);
        }
        ++i;
    }
    pcl::copyPointCloud(*cloud_in_ptr, valid_idx, *cloud_in_ptr);
}

template <typename BasicType>
Eigen::MatrixXf ConformalMappingSPH<BasicType>::radius_outlier_removal(const Eigen::MatrixXf &cloud_in,
                                                                       float radius_per_meter, int min_neighbours) {

    using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>;
    my_kd_tree_t mat_index(3, std::cref(cloud_in), 10 /* max leaf */);
    Eigen::MatrixXf query_pts = cloud_in.transpose();
    nanoflann::SearchParams params;
    params.sorted = false;
    std::vector<std::pair<long int, float>> ret_matches;
    std::vector<int> valid_idx;
    Eigen::VectorXf ranges_square =
        query_pts.row(0).array().pow(2) + query_pts.row(1).array().pow(2) + query_pts.row(2).array().pow(2);
    for (int i = 0; i < query_pts.cols(); ++i) {
        auto radius_square = static_cast<float>(radius_per_meter * radius_per_meter * ranges_square(i) + 1e-06);
        const size_t nMatches =
            mat_index.index->radiusSearch(query_pts.data() + i * 3, radius_square, ret_matches, params);
        if (nMatches >= min_neighbours) {
            valid_idx.push_back(i);
        }
    }
    Eigen::MatrixXf cloud_out;
    if (!valid_idx.empty()) {
        cloud_out = cloud_in(valid_idx, Eigen::all);
    }
    return cloud_out;
}

template <typename BasicType>
BasicType ConformalMappingSPH<BasicType>::IQR_thres(std::vector<BasicType> &vec, BasicType scale_factor) {
    std::sort(vec.begin(), vec.end());
    int n = 0;
    if ((vec.size() % 2) == 0) {
        n = static_cast<int>(vec.size() / 2);
    } else {
        n = static_cast<int>((vec.size() - 1) / 2);
    }
    BasicType Q1, Q3;
    if ((n % 2) != 0) {
        Q1 = vec[n / 2];
        Q3 = vec[(vec.size() * 2 - n) / 2];
    } else {
        Q1 = (vec[n / 2 - 1] + vec[n / 2]) / 2;
        Q3 = (vec[(vec.size() - n + vec.size() - 1) / 2] + vec[(vec.size() - n + vec.size() - 1) / 2 + 1]) / 2;
    }
    BasicType IQR = Q3 - Q1;
    return (Q3 + scale_factor * IQR);
}

template <typename BasicType>
BasicType ConformalMappingSPH<BasicType>::hist_thres(std::vector<BasicType> &vec, BasicType percentage) {
    std::sort(vec.begin(), vec.end());
    int n = 0;
    if ((vec.size() % 2) == 0) {
        n = static_cast<int>(vec.size() / 2);
    } else {
        n = static_cast<int>((vec.size() - 1) / 2);
    }
    BasicType Q1, Q3;
    if ((n % 2) != 0) {
        Q1 = vec[n / 2];
        Q3 = vec[(vec.size() * 2 - n) / 2];
    } else {
        Q1 = (vec[n / 2 - 1] + vec[n / 2]) / 2;
        Q3 = (vec[(vec.size() - n + vec.size() - 1) / 2] + vec[(vec.size() - n + vec.size() - 1) / 2 + 1]) / 2;
    }
    BasicType IQR = Q3 - Q1;
    BasicType bin_width = 2 * IQR * pow(static_cast<BasicType>(vec.size()), -1 / 3);
    int thres_bin_width_idx = ceil(vec[ceil(static_cast<BasicType>(vec.size()) * percentage)] / bin_width);
    BasicType thres = static_cast<BasicType>(thres_bin_width_idx) * bin_width;
    return thres;
}

template <typename BasicType>
std::pair<BasicType, int> ConformalMappingSPH<BasicType>::hist_fall_thres(std::vector<BasicType> &vec,
                                                                          BasicType percentage) {
    std::sort(vec.begin(), vec.end());
    int n = 0;
    if ((vec.size() % 2) == 0) {
        n = static_cast<int>(vec.size() / 2);
    } else {
        n = static_cast<int>((vec.size() - 1) / 2);
    }
    BasicType Q1, Q3;
    if ((n % 2) != 0) {
        Q1 = vec[n / 2];
        Q3 = vec[(vec.size() * 2 - n) / 2];
    } else {
        Q1 = (vec[n / 2 - 1] + vec[n / 2]) / 2;
        Q3 = (vec[(vec.size() - n + vec.size() - 1) / 2] + vec[(vec.size() - n + vec.size() - 1) / 2 + 1]) / 2;
    }
    BasicType IQR = Q3 - Q1;
    BasicType bin_width = 2 * IQR * pow(static_cast<BasicType>(vec.size()), -1 / 3);
    int last_bin_idx = 0;
    int current_bin_idx = 0;
    BasicType last_max_counter = 0.0;
    BasicType current_counter = 0.0;
    percentage = 1 - percentage;
    //    for (auto &item : vec) {
    int i;
    for (i = 0; i < vec.size(); ++i) {
        current_bin_idx = ceil(vec[i] / bin_width);
        if (last_bin_idx == current_bin_idx) {
            ++current_counter;
        } else {
            // always compare with the bin idx contain the largest number of elements in the past
            if ((current_counter / last_max_counter) < percentage) {
                break;
            } else if (current_counter > last_max_counter) {
                last_max_counter = current_counter;
            }
            current_counter = 0;
            last_bin_idx = current_bin_idx;
        }
    }
    BasicType thres = static_cast<BasicType>(current_bin_idx) * bin_width;
    return std::make_pair(thres, i);
}

template <typename BasicType>
void ConformalMappingSPH<BasicType>::cotangent_laplacian(
    std::vector<int> &f_filtered, const std::vector<int> &valid_face_idx, Ubpa::HEMeshInherit &mesh_half_edge,
    Eigen::VectorX<BasicType> &valid_area, Eigen::SparseMatrix<std::complex<BasicType>, Eigen::RowMajor> &L) {
    std::vector<BasicType> l;
    l.reserve(f_filtered.size());
    valid_area.resize(valid_face_idx.size());

    for (int i = 0; i < valid_face_idx.size(); ++i) {
        for (auto *he : mesh_half_edge.Polygons()[valid_face_idx[i]]->HalfEdge()->NextLoop()) {
            l.emplace_back(he->Edge()->calculate_length());
        }
        valid_area(i) = mesh_half_edge.Polygons()[valid_face_idx[i]]->calculate_area();
    }
    Eigen::Map<Eigen::Matrix<BasicType, 3, Eigen::Dynamic>> eigen_l(l.data(), 3, valid_face_idx.size());

    std::vector<Eigen::VectorX<BasicType>> V;
    V.emplace_back(
        (eigen_l.row(1).array().square() + eigen_l.row(2).array().square() - eigen_l.row(0).array().square()).array() /
        valid_area.transpose().array() / 2); // cot12
    V.emplace_back(
        (eigen_l.row(2).array().square() + eigen_l.row(0).array().square() - eigen_l.row(1).array().square()).array() /
        valid_area.transpose().array() / 2); // cot23
    V.emplace_back(
        (eigen_l.row(1).array().square() + eigen_l.row(0).array().square() - eigen_l.row(2).array().square()).array() /
        valid_area.transpose().array() / 2); // cot31
    V.emplace_back(-V[0] - V[2]);            // diag1
    V.emplace_back(-V[0] - V[1]);            // diag2
    V.emplace_back(-V[2] - V[1]);            // diag3
    Eigen::Map<Eigen::Matrix<int, 3, Eigen::Dynamic>> eigen_f_filtered(f_filtered.data(), 3, valid_face_idx.size());
    std::vector<int> II_row_idx{0, 1, 1, 2, 2, 0, 0, 1, 2};
    std::vector<int> JJ_row_idx{1, 0, 2, 1, 0, 2, 0, 1, 2};
    std::vector<int> value_idx{0, 0, 1, 1, 2, 2, 3, 4, 5};
    std::vector<Eigen::Triplet<std::complex<BasicType>>> triple_lists;
    triple_lists.reserve(3 * f_filtered.size());
    for (int i = 0; i < II_row_idx.size(); ++i) {
        for (int j = 0; j < valid_face_idx.size(); ++j) {
            triple_lists.emplace_back(eigen_f_filtered(II_row_idx[i], j), eigen_f_filtered(JJ_row_idx[i], j),
                                      V[value_idx[i]](j));
        }
    }
    L.setFromTriplets(triple_lists.begin(), triple_lists.end());
}

template <typename BasicType>
void ConformalMappingSPH<BasicType>::beltrami_coefficient(const Eigen::MatrixX<BasicType> &eigen_patch_local,
                                                          std::vector<int> &f_filtered,
                                                          const std::vector<int> &valid_face_idx,
                                                          const std::vector<int> &valid_vertex_idx,
                                                          Eigen::VectorX<BasicType> &disk_area,
                                                          const Eigen::Matrix<BasicType, 2, Eigen::Dynamic> &v,
                                                          Eigen::VectorX<std::complex<BasicType>> &mu) {
    Eigen::Map<Eigen::Matrix<int, 3, Eigen::Dynamic>> eigen_f_filtered(f_filtered.data(), 3, valid_face_idx.size());
    Eigen::Matrix<BasicType, 2, Eigen::Dynamic> e1 =
        v(Eigen::all, eigen_f_filtered.row(2)) - v(Eigen::all, eigen_f_filtered.row(1));
    Eigen::Matrix<BasicType, 2, Eigen::Dynamic> e2 =
        v(Eigen::all, eigen_f_filtered.row(0)) - v(Eigen::all, eigen_f_filtered.row(2));
    Eigen::Matrix<BasicType, 2, Eigen::Dynamic> e3 =
        v(Eigen::all, eigen_f_filtered.row(1)) - v(Eigen::all, eigen_f_filtered.row(0));
    // ToDo: think about why triangle area can be negaitive/maybe enhance the triangle removal
    // algorithm
    disk_area = (-e2.row(0).array() * e1.row(1).array() + e1.row(0).array() * e2.row(1).array()) / 2;
    Eigen::Matrix<BasicType, 3, Eigen::Dynamic> area(3, valid_face_idx.size());
    area.row(0) = disk_area;
    area.row(2) = area.row(1) = area.row(0);
    Eigen::Matrix<BasicType, 3, Eigen::Dynamic> Mx_matrix(3, valid_face_idx.size());
    Mx_matrix.row(0) = e1.row(1);
    Mx_matrix.row(1) = e2.row(1);
    Mx_matrix.row(2) = e3.row(1);
    Mx_matrix = Mx_matrix.array() / area.array() / 2;
    Eigen::Matrix<BasicType, 3, Eigen::Dynamic> My_matrix(3, valid_face_idx.size());
    My_matrix.row(0) = e1.row(0);
    My_matrix.row(1) = e2.row(0);
    My_matrix.row(2) = e3.row(0);
    My_matrix = -My_matrix.array() / area.array() / 2;
    Eigen::Map<Eigen::VectorX<BasicType>> Mx(Mx_matrix.data(), 3 * valid_face_idx.size());
    Eigen::Map<Eigen::VectorX<BasicType>> My(My_matrix.data(), 3 * valid_face_idx.size());
    // initlize the index of sparse matrix
    std::vector<int> idx_vec(3 * valid_face_idx.size());
    std::iota(idx_vec.begin(), idx_vec.end(), 0);
    Eigen::Map<Eigen::VectorXi> Mi(idx_vec.data(), static_cast<int>(idx_vec.size()));
    Mi = Mi.unaryExpr([](const int &value) { return value / 3; });
    Eigen::Map<Eigen::VectorXi> Mj(f_filtered.data(), f_filtered.size());
    // initlize the sparse matrix
    std::vector<Eigen::Triplet<BasicType>> Dx_triplets;
    Dx_triplets.reserve(f_filtered.size());
    std::vector<Eigen::Triplet<BasicType>> Dy_triplets;
    Dy_triplets.reserve(f_filtered.size());
    for (int i = 0; i < f_filtered.size(); ++i) {
        Dx_triplets.emplace_back(Mi(i), Mj(i), Mx(i));
        Dy_triplets.emplace_back(Mi(i), Mj(i), My(i));
    }
    Eigen::SparseMatrix<BasicType> Dx(valid_face_idx.size(), valid_vertex_idx.size());
    Eigen::SparseMatrix<BasicType> Dy(valid_face_idx.size(), valid_vertex_idx.size());
    Dx.setFromTriplets(Dx_triplets.begin(), Dx_triplets.end());
    Dy.setFromTriplets(Dy_triplets.begin(), Dy_triplets.end());
    Eigen::VectorX<BasicType> dXdu = Dx * eigen_patch_local(0, valid_vertex_idx).transpose();
    Eigen::VectorX<BasicType> dXdv = Dy * eigen_patch_local(0, valid_vertex_idx).transpose();
    Eigen::VectorX<BasicType> dYdu = Dx * eigen_patch_local(1, valid_vertex_idx).transpose();
    Eigen::VectorX<BasicType> dYdv = Dy * eigen_patch_local(1, valid_vertex_idx).transpose();
    Eigen::VectorX<BasicType> dZdu = Dx * eigen_patch_local(2, valid_vertex_idx).transpose();
    Eigen::VectorX<BasicType> dZdv = Dy * eigen_patch_local(2, valid_vertex_idx).transpose();

    Eigen::VectorX<BasicType> E = dXdu.array().square() + dYdu.array().square() + dZdu.array().square();
    Eigen::VectorX<BasicType> G = dXdv.array().square() + dYdv.array().square() + dZdv.array().square();
    Eigen::VectorX<BasicType> F =
        dXdu.array() * dXdv.array() + dYdu.array() * dYdv.array() + dZdu.array() * dZdv.array();
    mu = (E - G + std::complex<BasicType>(0, 2) * F).array() /
         (E + G + 2 * (E.array() * G.array() - F.array().square()).array().sqrt().matrix()).array();
}

template <typename BasicType>
void ConformalMappingSPH<BasicType>::generalized_laplacian(std::vector<int> &f_filtered,
                                                           const std::vector<int> &valid_face_idx,
                                                           Eigen::VectorX<BasicType> &disk_area,
                                                           const Eigen::Matrix<BasicType, 2, Eigen::Dynamic> &v,
                                                           const Eigen::VectorX<std::complex<BasicType>> &mu,
                                                           Eigen::SparseMatrix<BasicType, Eigen::RowMajor> &Ax) {
    Eigen::Map<Eigen::Matrix<int, 3, Eigen::Dynamic>> eigen_f_filtered(f_filtered.data(), 3, valid_face_idx.size());
    Eigen::VectorX<BasicType> af =
        (1 - 2 * mu.real().array() + mu.array().abs2()).array() / (1 - mu.array().abs2()).array();
    Eigen::VectorX<BasicType> bf = -2 * mu.imag().array() / (1 - mu.array().abs2()).array();
    Eigen::VectorX<BasicType> gf =
        (1 + 2 * mu.real().array() + mu.array().abs2()).array() / (1 - mu.array().abs2()).array();

    Eigen::VectorX<BasicType> uxv0 = v(1, eigen_f_filtered.row(1)) - v(1, eigen_f_filtered.row(2));
    Eigen::VectorX<BasicType> uyv0 = v(0, eigen_f_filtered.row(2)) - v(0, eigen_f_filtered.row(1));
    Eigen::VectorX<BasicType> uxv1 = v(1, eigen_f_filtered.row(2)) - v(1, eigen_f_filtered.row(0));
    Eigen::VectorX<BasicType> uyv1 = v(0, eigen_f_filtered.row(0)) - v(0, eigen_f_filtered.row(2));
    Eigen::VectorX<BasicType> uxv2 = v(1, eigen_f_filtered.row(0)) - v(1, eigen_f_filtered.row(1));
    Eigen::VectorX<BasicType> uyv2 = v(0, eigen_f_filtered.row(1)) - v(0, eigen_f_filtered.row(0));
    disk_area = disk_area.array().abs();
    std::vector<Eigen::VectorX<BasicType>> V;

    V.emplace_back((af.array() * uxv0.array() * uxv0.array() + 2 * bf.array() * uxv0.array() * uyv0.array() +
                    gf.array() * uyv0.array() * uyv0.array())
                       .array() /
                   disk_area.array() / 2);
    V.emplace_back((af.array() * uxv1.array() * uxv1.array() + 2 * bf.array() * uxv1.array() * uyv1.array() +
                    gf.array() * uyv1.array() * uyv1.array())
                       .array() /
                   disk_area.array() / 2);
    V.emplace_back((af.array() * uxv2.array() * uxv2.array() + 2 * bf.array() * uxv2.array() * uyv2.array() +
                    gf.array() * uyv2.array() * uyv2.array())
                       .array() /
                   disk_area.array() / 2);
    V.emplace_back((af.array() * uxv1.array() * uxv0.array() + bf.array() * uxv1.array() * uyv0.array() +
                    bf.array() * uxv0.array() * uyv1.array() + gf.array() * uyv1.array() * uyv0.array())
                       .array() /
                   disk_area.array() / 2);
    V.emplace_back((af.array() * uxv2.array() * uxv1.array() + bf.array() * uxv2.array() * uyv1.array() +
                    bf.array() * uxv1.array() * uyv2.array() + gf.array() * uyv2.array() * uyv1.array())
                       .array() /
                   disk_area.array() / 2);
    V.emplace_back((af.array() * uxv0.array() * uxv2.array() + bf.array() * uxv0.array() * uyv2.array() +
                    bf.array() * uxv2.array() * uyv0.array() + gf.array() * uyv0.array() * uyv2.array())
                       .array() /
                   disk_area.array() / 2);
    std::vector<int> I_idx{0, 1, 2, 0, 1, 1, 2, 2, 0};
    std::vector<int> J_idx{0, 1, 2, 1, 0, 2, 1, 0, 2};
    std::vector<int> V_idx{0, 1, 2, 3, 3, 4, 4, 5, 5};
    std::vector<Eigen::Triplet<BasicType>> triple_lists;
    triple_lists.reserve(3 * f_filtered.size());
    for (int i = 0; i < I_idx.size(); ++i) {
        for (int j = 0; j < valid_face_idx.size(); ++j) {
            triple_lists.emplace_back(eigen_f_filtered(I_idx[i], j), eigen_f_filtered(J_idx[i], j), -V[V_idx[i]](j));
        }
    }
    Ax.setFromTriplets(triple_lists.begin(), triple_lists.end());
}

/****************************************************************************************************/
/*********************************************Procedures*********************************************/
/****************************************************************************************************/

template <typename BasicType>
void ConformalMappingSPH<BasicType>::callback_point_cloud_segments(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    segment_pt_queue_lock.lock();
    seg_cloud_queue.push(msg);
    segment_pt_queue_lock.unlock();
}

template <typename BasicType> bool ConformalMappingSPH<BasicType>::preprocessing() {
    //    Timer preprocessing_timer;
    //    preprocessing_timer.start();
    if (!seg_cloud_queue.empty()) {
        seg_cloud_ptr->clear();
        seg_patches_idx.clear();
        const sensor_msgs::PointCloud2::ConstPtr msg = seg_cloud_queue.front();
        segment_pt_queue_lock.lock();
        seg_cloud_queue.pop();
        segment_pt_queue_lock.unlock();
        pcl::fromROSMsg(*msg, *seg_cloud_ptr);
        // filter patches size haven't achieved requirement
        float last_intensity = seg_cloud_ptr->points[0].intensity;
        std::vector<int> idx;
        last_intensity = seg_cloud_ptr->points[0].intensity;
        std::vector<int> seg_patches_idx_tmp;
        idx.clear();
        for (int i = 0; i < seg_cloud_ptr->size(); ++i) {
            if (last_intensity == seg_cloud_ptr->points[i].intensity) {
                seg_patches_idx_tmp.push_back(i);
            }
            if (last_intensity != seg_cloud_ptr->points[i].intensity || (i == seg_cloud_ptr->size() - 1)) {
                if (seg_patches_idx_tmp.size() >= conformal_mapping_config_ptr->patch_minimum_pts_num) {
                    seg_patches_idx.push_back(seg_patches_idx_tmp);
                    idx.insert(idx.end(), seg_patches_idx_tmp.begin(), seg_patches_idx_tmp.end());
                    seg_patches_idx_tmp.clear();
                    seg_patches_idx_tmp.push_back(i);
                } else {
                    seg_patches_idx_tmp.clear();
                }
                last_intensity = seg_cloud_ptr->points[i].intensity;
            }
        }
        // publish valid patches clouds
        pcl::PointCloud<PointT> filter_cloud;
        pcl::copyPointCloud(*seg_cloud_ptr, idx, filter_cloud);
        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(filter_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header = msg->header;
        filtered_cloud_msg.header.stamp = ros::Time::now();
        pub_pt_valid_patches.publish(filtered_cloud_msg);
        //        std::cout << "*********************************************************" << std::endl;
        //        std::cout << "*********************************************************" << std::endl;
        //        std::cout << "Preprocessing time: " << preprocessing_timer.elapsedMilliseconds() << " ms" <<
        //        std::endl;
        return true;
    } else {
        return false;
    }
}

template <typename BasicType> void ConformalMappingSPH<BasicType>::conformal_mapping() {
    while (ros::ok()) {
        if (preprocessing()) {
//            Timer time;
//            time.start();
#pragma omp parallel for num_threads(conformal_mapping_config_ptr->thread_num) default(none)                           \
    shared(conformal_mapping_config_ptr, SH_table_config_ptr, debug_config_ptr, seg_cloud_ptr, seg_patches_idx,        \
               std::cout, Eigen::Dynamic)
            for (const auto &patch_idx : seg_patches_idx) {
                Ubpa::HEMeshInherit mesh_half_edge;
                BasicType boundary_length;
                std::vector<BasicType> boundary_length_segment;
                Eigen::Vector3<BasicType> centroid;
                Eigen::Isometry3d T_obj_lidar;
                Eigen::MatrixX<BasicType> eigen_patch_local; // DIM: 3*N
                Eigen::Matrix<int, Eigen::Dynamic, 3> f;
                std::vector<int> valid_face_idx, valid_vertex_idx, boundary_idx, corner_idx, f_filtered;
                Eigen::MatrixX<BasicType> projection_2d_pts; // DIM: 2*N
                Eigen::VectorX<BasicType> valid_area;
                Eigen::VectorX<BasicType> disk_area;
                Eigen::VectorX<BasicType> square_x;
                Eigen::VectorX<BasicType> square_y;
                Eigen::VectorX<BasicType> sph_coeff;
                // extract a patch
                //                Timer sub_timer;
                //                sub_timer.start();
                pcl::PointCloud<PointT>::Ptr patch_cloud_ptr(new pcl::PointCloud<PointT>);
                int ID = omp_get_thread_num();
                printf("ID(%d)", ID);
                pcl::copyPointCloud(*seg_cloud_ptr, patch_idx, *patch_cloud_ptr);
                // filter
                if (conformal_mapping_config_ptr->is_voxel_grid_filter) {
                    //                    std::cout << "points number before voxel grid filter: " <<
                    //                    patch_cloud_ptr->size() << std::endl;

                    pcl::VoxelGrid<PointT> sor;
                    sor.setInputCloud(patch_cloud_ptr);
                    sor.setLeafSize(conformal_mapping_config_ptr->leaf_size, conformal_mapping_config_ptr->leaf_size,
                                    conformal_mapping_config_ptr->leaf_size);
                    sor.filter(*patch_cloud_ptr);
                    //                    std::cout << "points number after voxel grid filter: " <<
                    //                    patch_cloud_ptr->size() << std::endl;
                }
                Eigen::Map<Eigen::MatrixXf, 0, Eigen::Stride<8, 0>> eigen_patch_ori(
                    reinterpret_cast<float *>(patch_cloud_ptr->points.data()), 3,
                    static_cast<int>(patch_cloud_ptr->size()), Eigen::Stride<8, 0>());
                if (conformal_mapping_config_ptr->is_StatisticalOutlierRemoval) {
                    pcl::StatisticalOutlierRemoval<PointT> sor;
                    sor.setInputCloud(patch_cloud_ptr);
                    // search query point nearest 50 points as a Gaussian distribution
                    sor.setMeanK(conformal_mapping_config_ptr->outlier_removing_neighbour_num);
                    // if the point is out of certain std, then remove it
                    sor.setStddevMulThresh(conformal_mapping_config_ptr->outlier_removing_std);
                    sor.filter(*patch_cloud_ptr);
                }
                Eigen::MatrixXf eigen_patch;
                if (conformal_mapping_config_ptr->is_RadiusOutlierRemoval) {
                    //                    std::cout << "points number before radius filter: " << eigen_patch_ori.cols()
                    //                    << std::endl;
                    eigen_patch = radius_outlier_removal(eigen_patch_ori.transpose(),
                                                         conformal_mapping_config_ptr->outlier_removing_radius,
                                                         conformal_mapping_config_ptr->outlier_removing_min_nei);
                    eigen_patch.transposeInPlace();
                    //                    std::cout << "points number after radius filter: " << eigen_patch.cols() <<
                    //                    std::endl;
                } else {
                    eigen_patch = eigen_patch_ori;
                }
                // if after filtering, the points number is very few, skip this patch
                if (eigen_patch.cols() < conformal_mapping_config_ptr->patch_minimun_pts_num_after_filter) {
                    continue;
                }
                // copy the eigen_patch points to pcl type
                patch_cloud_ptr->resize(eigen_patch.cols());
                Eigen::Map<Eigen::MatrixXf, 0, Eigen::Stride<8, 0>>(
                    reinterpret_cast<float *>(patch_cloud_ptr->points.data()), 3, static_cast<int>(eigen_patch.cols()),
                    Eigen::Stride<8, 0>()) = eigen_patch;
                //                std::cout << "Patch extraction and filtering timer: " <<
                //                sub_timer.elapsedMilliseconds() << std::endl;
                // projection
                //                sub_timer.start();
                projection(eigen_patch, T_obj_lidar, eigen_patch_local);
                centroid = T_obj_lidar.inverse().translation().cast<BasicType>();
                //                std::cout << "Projection time: " << sub_timer.elapsedMilliseconds() << "ms" <<
                //                std::endl;
                // meshing
                //                sub_timer.start();
                perspective_view(eigen_patch_local, centroid, projection_2d_pts);
                //                std::cout << "Perspective: " << sub_timer.elapsedMilliseconds() << " ms" << std::endl;
                //                sub_timer.start();
                bool is_triangulation_sucess = triangulation_patch(
                    eigen_patch_local, projection_2d_pts, f, mesh_half_edge, valid_face_idx, valid_vertex_idx,
                    boundary_idx, f_filtered, boundary_length_segment, boundary_length);
                //                std::cout << "Triangulation: " << sub_timer.elapsedMilliseconds() << " ms" <<
                //                std::endl;
                if (valid_face_idx.size() >= conformal_mapping_config_ptr->minimum_triangles_after_filter &&
                    is_triangulation_sucess) {
                    //                    sub_timer.start();
                    corner_selection(projection_2d_pts, valid_vertex_idx, boundary_idx, boundary_length, mesh_half_edge,
                                     corner_idx);
                    //                    std::cout << "Corner selection: " << sub_timer.elapsedMilliseconds() << " ms"
                    //                    << std::endl;

                    // patch_rectangular_conformal_map
                    //                    sub_timer.start();
                    bool is_success =
                        rectangular_conformal_map(eigen_patch_local, boundary_idx, corner_idx, boundary_length_segment,
                                                  boundary_length, valid_vertex_idx, f_filtered, valid_face_idx,
                                                  mesh_half_edge, valid_area, disk_area, square_x, square_y);
                    //                    std::cout << "Conformal Mapping: " << sub_timer.elapsedMilliseconds() << " ms"
                    //                    << std::endl;
                    if (is_success) {
                        // conformal_interpolation
                        //                        sub_timer.start();
                        int status =
                            conformal_interpolation_extraction(square_x, square_y, valid_vertex_idx, eigen_patch_local,
                                                               sph_coeff, SH_table_config_ptr->max_SH_degree, false);
                        if (status != curl::BAD) {
                            is_success = true;
                        } else {
                            is_success = false;
                        }
                        if (is_success) {
                        }
                        //                        std::cout << "Sampling and SPH Extraction: " <<
                        //                        sub_timer.elapsedMilliseconds() << " ms"
                        //                                  << std::endl;
                    }
                }
            }
            //            std::cout << "Total Time: " << time.elapsedMilliseconds() << " ms" << std::endl;
            //            std::cout << '\n' << std::endl;
        }
        seg_patches_idx.clear();
    }
}

template <typename BasicType>
void ConformalMappingSPH<BasicType>::get_local_coordinate(Eigen::MatrixXf &eigen_patch,
                                                          Eigen::Isometry3d &T_obj_lidar) {
    Eigen::Vector3<BasicType> centroid = eigen_patch.rowwise().mean().cast<BasicType>();
    Eigen::MatrixX<BasicType> cen_sph = curl::cart2sph<BasicType>(centroid.transpose());
    //    transformed_centroid = ((rotz(pi / 2) * roty(pi / 2) * rotz(-cen_azi)) * centroid')';
    Eigen::Matrix3<BasicType> rot = (Eigen::AngleAxis<BasicType>(M_PI_2, Eigen::Vector3<BasicType>::UnitZ()) *
                                     Eigen::AngleAxis<BasicType>(M_PI_2, Eigen::Vector3<BasicType>::UnitY()) *
                                     Eigen::AngleAxis<BasicType>(-cen_sph(0), Eigen::Vector3<BasicType>::UnitZ()))
                                        .toRotationMatrix();
    Eigen::Vector3<BasicType> transformed_centroid = rot * centroid;
    T_obj_lidar.setIdentity();
    T_obj_lidar.matrix()(Eigen::seq(0, 2), Eigen::seq(0, 2)) = rot.template cast<double>();
    T_obj_lidar.matrix()(Eigen::seq(0, 2), 3) = -transformed_centroid.template cast<double>();
}

template <typename BasicType>
void ConformalMappingSPH<BasicType>::get_ground_local_coordinate(Eigen::MatrixXf &eigen_patch,
                                                                 const Eigen::Matrix3d &eigen_rot,
                                                                 Eigen::Isometry3d &T_obj_lidar) {
    Eigen::Isometry3d T_lidar_obj;
    T_lidar_obj.setIdentity();
    T_lidar_obj.matrix()(Eigen::seq(0, 2), Eigen::seq(0, 2)) = eigen_rot;
    T_lidar_obj.matrix()(Eigen::seq(0, 2), 3) = eigen_patch.rowwise().mean().cast<double>();
    T_obj_lidar = T_lidar_obj.inverse();
}

template <typename BasicType>
void ConformalMappingSPH<BasicType>::projection(Eigen::MatrixXf &eigen_patch, Eigen::Isometry3d &T_obj_lidar,
                                                Eigen::MatrixX<BasicType> &eigen_patch_local) {
    get_local_coordinate(eigen_patch, T_obj_lidar);
    eigen_patch_local = (T_obj_lidar.rotation().matrix().cast<BasicType>() * eigen_patch.cast<BasicType>()).colwise() +
                        T_obj_lidar.translation().matrix().cast<BasicType>();
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/v_local.txt",
                                       Eigen::MatrixXd(eigen_patch_local.transpose().template cast<double>()));
    }
}

template <typename BasicType>
void ConformalMappingSPH<BasicType>::perspective_view(const Eigen::MatrixX<BasicType> &eigen_patch_local,
                                                      const Eigen::Vector3<BasicType> &centroid,
                                                      Eigen::MatrixX<BasicType> &projection_2d_pts) {
    Eigen::MatrixX<BasicType> projection_init = eigen_patch_local;
    projection_init.row(2) = eigen_patch_local.row(2).array() + centroid.norm();
    projection_2d_pts =
        projection_init(Eigen::seq(0, 1), Eigen::all).array().rowwise() / projection_init.row(2).array();
}

template <typename BasicType>
bool ConformalMappingSPH<BasicType>::triangulation_patch(
    const Eigen::MatrixX<BasicType> &eigen_patch_local, const Eigen::MatrixX<BasicType> &projection_2d_pts,
    Eigen::Matrix<int, Eigen::Dynamic, 3> &f, Ubpa::HEMeshInherit &mesh_half_edge, std::vector<int> &valid_face_idx,
    std::vector<int> &valid_vertex_idx, std::vector<int> &boundary_idx, std::vector<int> &f_filtered,
    std::vector<BasicType> &boundary_length_segment, BasicType &boundary_length) {
    cgal::delaunay_triangulation<Eigen::MatrixX<BasicType>, Eigen::Matrix<int, Eigen::Dynamic, 3>>(projection_2d_pts,
                                                                                                   f);
    if (f.size() == 0) {
        return false;
    }
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/v.txt",
                                       Eigen::MatrixXd(projection_2d_pts.transpose().template cast<double>()));
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/f.txt",
                                       Eigen::MatrixXd(f.cast<double>()));
    }
    if (conformal_mapping_config_ptr->defect_triangulation_type == "2D") {
        mesh_half_edge.Init(projection_2d_pts, 2, f);
    } else if (conformal_mapping_config_ptr->defect_triangulation_type == "3D") {
        mesh_half_edge.Init(eigen_patch_local, 3, f);
    }
    std::vector<BasicType> areas, lengths;
    mesh_half_edge.calculate_triangle_properties(*conformal_mapping_config_ptr, areas, lengths);
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_vector_txt_file(debug_config_ptr->middle_patch_dir + "/areas.txt", areas);
        FileReaderBase::write_vector_txt_file(debug_config_ptr->middle_patch_dir + "/lengths.txt", lengths);
    }
    std::set<int> init_valid_face_idx;
    if (conformal_mapping_config_ptr->is_IQR) {
        conformal_mapping_config_ptr->tri_area_max =
            IQR_thres(areas, conformal_mapping_config_ptr->tri_area_max_scale_factor);
        conformal_mapping_config_ptr->tri_length_max =
            IQR_thres(lengths, conformal_mapping_config_ptr->tri_length_max_scale_factor);
    } else if (conformal_mapping_config_ptr->is_hist) {
        conformal_mapping_config_ptr->tri_area_max =
            hist_thres(areas, conformal_mapping_config_ptr->tri_area_percentage);
        conformal_mapping_config_ptr->tri_length_max =
            hist_thres(lengths, conformal_mapping_config_ptr->tri_length_percentage);
    } else if (conformal_mapping_config_ptr->is_hist_fall) {

        std::pair<BasicType, int> thres_area_pair =
            hist_fall_thres(areas, conformal_mapping_config_ptr->tri_area_percentage);
        areas.resize(thres_area_pair.second);
        conformal_mapping_config_ptr->tri_area_max =
            hist_fall_thres(areas, conformal_mapping_config_ptr->tri_area_percentage).first;

        std::pair<BasicType, int> thres_length_pair =
            hist_fall_thres(lengths, conformal_mapping_config_ptr->tri_length_percentage);
        lengths.resize(thres_length_pair.second);
        conformal_mapping_config_ptr->tri_length_max =
            hist_fall_thres(lengths, conformal_mapping_config_ptr->tri_length_percentage).first;
    }
    mesh_half_edge.remove_defect_triangles(*conformal_mapping_config_ptr, init_valid_face_idx);
    mesh_half_edge.remove_defect_clusters(*conformal_mapping_config_ptr, init_valid_face_idx, valid_face_idx,
                                          valid_vertex_idx, boundary_idx, boundary_length_segment, f_filtered);
    if (valid_face_idx.size() < conformal_mapping_config_ptr->minimum_triangles_after_filter) {
        return false;
    }
    boundary_length = std::accumulate(boundary_length_segment.begin(), boundary_length_segment.end(), 0.0);
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(
            debug_config_ptr->middle_patch_dir + "/f_filtered.txt",
            Eigen::MatrixXd(Eigen::Map<Eigen::MatrixX<int>>(f_filtered.data(), 3, valid_face_idx.size())
                                .transpose()
                                .cast<double>()));

        FileReaderBase::write_vector_txt_file(debug_config_ptr->middle_patch_dir + "/valid_vertex_idx.txt",
                                              valid_vertex_idx);
    }
    return true;
}

template <typename BasicType>
void ConformalMappingSPH<BasicType>::corner_selection(const Eigen::MatrixX<BasicType> &projection_2d_pts,
                                                      std::vector<int> &valid_vertex_idx,
                                                      const std::vector<int> &boundary_idx,
                                                      const BasicType &boundary_length,
                                                      Ubpa::HEMeshInherit &mesh_half_edge,
                                                      std::vector<int> &corner_idx) {
    // minimum bounding box
    //    Timer convex_timer;
    //    convex_timer.start();
    //    std::vector<int> convex_idx;
    //    cgal::convex_hull_2d_methods<BasicType>(projection(Eigen::all, boundary_idx), convex_idx);
    Eigen::Map<Eigen::VectorXi> eigen_valid_vertex_idx(valid_vertex_idx.data(), valid_vertex_idx.size());
    Eigen::MatrixX<BasicType> convex_pt;
    cgal::convex_hull_2d_ch_bykat<BasicType>(projection_2d_pts(Eigen::all, eigen_valid_vertex_idx(boundary_idx)),
                                             convex_pt);
    //    std::cout << "Convex hull timer: " << convex_timer.elapsedMilliseconds() << " ms" << std::endl;
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/convex.txt",
                                       Eigen::MatrixXd(convex_pt.transpose().template cast<double>()));
        FileReaderBase::write_txt_file(
            debug_config_ptr->middle_patch_dir + "/v_boundary.txt",
            Eigen::MatrixXd(projection_2d_pts(Eigen::all, eigen_valid_vertex_idx(boundary_idx))
                                .transpose()
                                .template cast<double>()));
    }
    std::vector<int> front_idx;
    std::vector<int> back_idx;
    Eigen::MatrixX<BasicType> E = convex_pt(Eigen::all, Eigen::seq(1, convex_pt.cols() - 1)) -
                                  convex_pt(Eigen::all, Eigen::seq(0, convex_pt.cols() - 2));
    Eigen::VectorX<BasicType> T =
        E.row(1).binaryExpr(E.row(0), [](BasicType y, BasicType x) { return std::atan2(y, x); });
    // Compute mod(T, pi/2)
    T = (T.array() - (T.array() / (M_PI / 2)).floor() * (M_PI / 2)).eval();
    std::vector<BasicType> T_std(T.data(), T.data() + T.size());
    std::sort(T_std.begin(), T_std.end());
    T_std.erase(std::unique(T_std.begin(), T_std.end()), T_std.end());
    T = Eigen::Map<Eigen::VectorX<BasicType>>(T_std.data(), T_std.size());
    // create rotation matrix
    int n = T.size();
    // Duplicate angles in T and reshape
    Eigen::MatrixX<BasicType> T_duplicate(2 * n, 2);
    Eigen::MatrixX<BasicType> angle_shift(2 * n, 2);
    for (int i = 0; i < n; i++) {
        T_duplicate.row(i * 2) << T(i), T(i);
        T_duplicate.row(i * 2 + 1) << T(i), T(i);
        angle_shift.row(i * 2) << 0, -M_PI_2;
        angle_shift.row(i * 2 + 1) << M_PI_2, 0;
    }
    // Compute R
    Eigen::MatrixX<BasicType> R = (T_duplicate + angle_shift).array().cos();
    Eigen::MatrixX<BasicType> RCH = R * convex_pt;
    Eigen::VectorX<BasicType> bsize = RCH.rowwise().maxCoeff() - RCH.rowwise().minCoeff();
    Eigen::VectorX<BasicType> area =
        bsize(Eigen::seq(0, Eigen::last - 1, 2)).array() * bsize(Eigen::seq(1, Eigen::last, 2)).array();
    int min_idx;
    area.minCoeff(&min_idx);
    Eigen::MatrixX<BasicType> Rf = R(Eigen::seq(2 * min_idx, 2 * min_idx + 1), Eigen::all);
    Eigen::MatrixX<BasicType> bound = Rf * convex_pt;
    Eigen::VectorX<BasicType> bmin = bound.rowwise().minCoeff();
    Eigen::VectorX<BasicType> bmax = bound.rowwise().maxCoeff();
    Rf.transposeInPlace();
    Eigen::Matrix<BasicType, 2, 4> corners;
    // conterclockwise
    corners(Eigen::all, 0) = bmax(0) * Rf(Eigen::all, 0) + bmin(1) * Rf(Eigen::all, 1);
    corners(Eigen::all, 1) = bmax(0) * Rf(Eigen::all, 0) + bmax(1) * Rf(Eigen::all, 1);
    corners(Eigen::all, 2) = bmin(0) * Rf(Eigen::all, 0) + bmax(1) * Rf(Eigen::all, 1);
    corners(Eigen::all, 3) = bmin(0) * Rf(Eigen::all, 0) + bmin(1) * Rf(Eigen::all, 1);
    // save txt file
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/rect_corners.txt",
                                       Eigen::MatrixXd(corners.template cast<double>().transpose()));
    }
    // get indices which are the closest to rectangular corners
    Eigen::MatrixX<BasicType> boundary_pts =
        projection_2d_pts(Eigen::all, eigen_valid_vertex_idx(boundary_idx)).transpose();
    using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixX<BasicType>>;
    my_kd_tree_t mat_index(2, std::cref(boundary_pts), 5);
    std::vector<size_t> ret_indexes(1);
    std::vector<BasicType> out_dists_sqr(1);
    nanoflann::KNNResultSet<BasicType> resultSet(1);
    corner_idx.resize(corners.cols());
    std::vector<BasicType> corner_distances(corners.cols());
    for (int i = 0; i < corners.cols(); ++i) {
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
        Eigen::Vector2<BasicType> query = corners.col(i);
        mat_index.index->findNeighbors(resultSet, corners.data() + i * 2, nanoflann::SearchParams());
        corner_idx[i] = (boundary_idx[static_cast<int>(ret_indexes[0])]);
        corner_distances[i] = out_dists_sqr[0];
    }
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(
            debug_config_ptr->middle_patch_dir + "/old_corners.txt",
            Eigen::MatrixXd(
                projection_2d_pts(Eigen::all, eigen_valid_vertex_idx(corner_idx)).template cast<double>().transpose()));
    }
    // find the nearest boundary point to bounding box corens as the start point
    std::vector<int> tmp_idx(corners.cols());
    std::iota(tmp_idx.begin(), tmp_idx.end(), 0);
    std::sort(tmp_idx.begin(), tmp_idx.end(),
              [&](int &a, int &b) -> bool { return (corner_distances[a] < corner_distances[b]); });
    // reorder corners points to make tmp_idx[0] at the beginning
    if (tmp_idx[0] > 0) {
        Eigen::MatrixX<BasicType> corners_upper = corners(Eigen::all, Eigen::seq(tmp_idx[0], Eigen::last));
        Eigen::MatrixX<BasicType> corners_lower = corners(Eigen::all, Eigen::seq(0, tmp_idx[0] - 1));
        corners(Eigen::all, Eigen::seq(0, corners_upper.cols() - 1)) = corners_upper;
        corners(Eigen::all, Eigen::seq(corners_upper.cols(), Eigen::last)) = corners_lower;
    }
    // reselect corner points combining with length
    //    mesh_half_edge.select_corners_related_with_boundary_length(
    //        valid_vertex_idx, tmp_idx, corners.template cast<BT>(), projection_2d_pts,
    //        static_cast<BT>(boundary_length), corner_idx,
    //        conformal_mapping_config_ptr->rectangular_corner_weight);
    // rotate corners by one to make sure longer edge is on height
    if ((projection_2d_pts.col(valid_vertex_idx[corner_idx[0]]) -
         projection_2d_pts.col(valid_vertex_idx[corner_idx[1]]))
            .norm() > (projection_2d_pts.col(valid_vertex_idx[corner_idx[1]]) -
                       projection_2d_pts.col(valid_vertex_idx[corner_idx[2]]))
                          .norm()) {
        std::rotate(corner_idx.begin(), corner_idx.begin() + 1, corner_idx.end());
    }
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(
            debug_config_ptr->middle_patch_dir + "/corners.txt",
            Eigen::MatrixXd(
                projection_2d_pts(Eigen::all, eigen_valid_vertex_idx(corner_idx)).template cast<double>().transpose()));
        FileReaderBase::write_vector_txt_file(debug_config_ptr->middle_patch_dir + "/corners_idx.txt", corner_idx);
    }
}

template <typename BasicType>
bool ConformalMappingSPH<BasicType>::rectangular_conformal_map(
    const Eigen::MatrixX<BasicType> &eigen_patch_local, std::vector<int> &boundary_idx,
    const std::vector<int> &corner_idx, std::vector<BasicType> &boundary_length_segment,
    const BasicType &boundary_length, const std::vector<int> &valid_vertex_idx, std::vector<int> &f_filtered,
    const std::vector<int> &valid_face_idx, Ubpa::HEMeshInherit &mesh_half_edge, Eigen::VectorX<BasicType> &valid_area,
    Eigen::VectorX<BasicType> &disk_area, Eigen::VectorX<BasicType> &square_x, Eigen::VectorX<BasicType> &square_y) {
    // make the first corner to the first
    auto it = std::find(boundary_idx.begin(), boundary_idx.end(), corner_idx[0]);
    int first_corner_idx = it - boundary_idx.begin();
    Eigen::Map<Eigen::VectorXi> eigen_boundary_idx(boundary_idx.data(), boundary_idx.size());
    Eigen::VectorXi eigen_boundary_idx_tmp = eigen_boundary_idx(Eigen::seq(0, first_corner_idx - 1));
    eigen_boundary_idx(Eigen::seq(0, Eigen::last - first_corner_idx)) =
        eigen_boundary_idx(Eigen::seq(first_corner_idx, Eigen::last)).eval();
    eigen_boundary_idx(Eigen::seq(Eigen::last - first_corner_idx + 1, Eigen::last)) = eigen_boundary_idx_tmp;
    std::vector<int> corner_idx_in_bdy(4);
    corner_idx_in_bdy[0] = 0;
    for (int i = 1; i < 4; ++i) {
        corner_idx_in_bdy[i] = std::distance(
            eigen_boundary_idx.begin(), std::find(eigen_boundary_idx.begin(), eigen_boundary_idx.end(), corner_idx[i]));
    }
    if (corner_idx_in_bdy[0] >= corner_idx_in_bdy[1] || corner_idx_in_bdy[1] >= corner_idx_in_bdy[2] ||
        corner_idx_in_bdy[2] >= corner_idx_in_bdy[3]) {
        //        std::cerr << "Wrong corner points" << std::endl;
        return false;
    }
    // make the first corner length segement to be the first
    Eigen::Map<Eigen::VectorX<BasicType>> eigen_boundary_length_segment(boundary_length_segment.data(),
                                                                        boundary_length_segment.size());
    Eigen::VectorX<BasicType> eigen_boundary_segment_tmp =
        eigen_boundary_length_segment(Eigen::seq(0, first_corner_idx - 1));
    eigen_boundary_length_segment(Eigen::seq(0, Eigen::last - first_corner_idx)) =
        eigen_boundary_length_segment(Eigen::seq(first_corner_idx, Eigen::last)).eval();
    eigen_boundary_length_segment(Eigen::seq(Eigen::last - first_corner_idx + 1, Eigen::last)) =
        eigen_boundary_segment_tmp;
    Eigen::VectorX<BasicType> partial_dege_sum = Eigen::VectorX<BasicType>::Zero(eigen_boundary_length_segment.size());
    for (int i = 1; i < partial_dege_sum.size(); ++i) {
        for (int j = 0; j < i; ++j) {
            partial_dege_sum(i) += eigen_boundary_length_segment(j);
        }
    }
    Eigen::VectorX<BasicType> theta = 2 * M_PI * partial_dege_sum / boundary_length;
    // this creates a complex number with magnitude 1 and angle 'value'
    Eigen::VectorX<std::complex<BasicType>> bdy =
        theta.unaryExpr([](BasicType value) { return std::polar(static_cast<BasicType>(1.0), value); });
    Eigen::SparseMatrix<std::complex<BasicType>, Eigen::RowMajor> L(valid_vertex_idx.size(), valid_vertex_idx.size());
    cotangent_laplacian(f_filtered, valid_face_idx, mesh_half_edge, valid_area, L);
    // Assuming bdy_index is std::vector<int> and already populated

    // Create a triplet list for the sparse matrix modifications
    std::vector<Eigen::Triplet<std::complex<BasicType>>> triple_lists;
    Eigen::SparseMatrix<std::complex<BasicType>, Eigen::RowMajor> L_counter(valid_vertex_idx.size(),
                                                                            valid_vertex_idx.size());
    for (int &bdy_idx : boundary_idx) {
        // Iterate over the row specified by bdy_index[i]
        for (typename Eigen::SparseMatrix<std::complex<BasicType>, Eigen::RowMajor>::InnerIterator iter(L, bdy_idx);
             iter; ++iter) {
            // Subtract mval from existing value
            triple_lists.emplace_back(bdy_idx, iter.col(), -iter.value());
            // If the column index is also a boundary index, add 1
            if (bdy_idx == iter.col()) {
                triple_lists.emplace_back(bdy_idx, iter.col(), 1.0);
            }
        }
    }
    // Apply the changes to the matrix
    L_counter.setFromTriplets(triple_lists.begin(), triple_lists.end());
    L += L_counter;
    // solve least square
    Eigen::VectorX<std::complex<BasicType>> c(valid_vertex_idx.size());
    c.setZero();
    c(boundary_idx) = bdy;

    // Create the solver object
    Eigen::SparseLU<Eigen::SparseMatrix<std::complex<BasicType>, Eigen::RowMajor>, Eigen::COLAMDOrdering<int>>
        complex_solver;
    // Compute the decomposition
    complex_solver.analyzePattern(L);
    complex_solver.factorize(L);
    // Check for success
    if (complex_solver.info() != Eigen::Success) {
        std::cerr << "Decomposition failed" << std::endl;
        return false;
    }
    // Solve the system
    Eigen::VectorX<std::complex<BasicType>> z = complex_solver.solve(c); // L need to be complex as well
    // Check for success
    if (complex_solver.info() != Eigen::Success) {
        std::cerr << "Solving failed" << std::endl;
        return false;
    }
    Eigen::Matrix<BasicType, 2, Eigen::Dynamic> disk(2, valid_vertex_idx.size());
    disk.row(0) = z.real();
    disk.row(1) = z.imag();
    Eigen::VectorX<std::complex<BasicType>> mu;
    beltrami_coefficient(eigen_patch_local, f_filtered, valid_face_idx, valid_vertex_idx, disk_area, disk, mu);
    Eigen::SparseMatrix<BasicType, Eigen::RowMajor> Ax(valid_vertex_idx.size(), valid_vertex_idx.size());
    generalized_laplacian(f_filtered, valid_face_idx, disk_area, disk, mu, Ax);
    Eigen::SparseMatrix<BasicType, Eigen::RowMajor> Ay = Ax;
    Eigen::VectorXi bottom = eigen_boundary_idx(Eigen::seq(corner_idx_in_bdy[0], corner_idx_in_bdy[1]));
    Eigen::VectorXi right = eigen_boundary_idx(Eigen::seq(corner_idx_in_bdy[1], corner_idx_in_bdy[2]));
    Eigen::VectorXi top = eigen_boundary_idx(Eigen::seq(corner_idx_in_bdy[2], corner_idx_in_bdy[3]));
    Eigen::VectorXi left = eigen_boundary_idx(Eigen::seq(corner_idx_in_bdy[3], Eigen::last));
    left.conservativeResize(left.size() + 1);
    left(left.size() - 1) = eigen_boundary_idx(0);
    Eigen::VectorX<BasicType> bx = Eigen::VectorX<BasicType>::Zero(valid_vertex_idx.size());
    Eigen::VectorX<BasicType> by = bx;
    bx(right) = Eigen::VectorX<BasicType>::Ones(right.size());
    by(top) = Eigen::VectorX<BasicType>::Ones(top.size());
    // Modify Ax
    Eigen::SparseMatrix<BasicType, Eigen::RowMajor> Ax_mod(valid_vertex_idx.size(), valid_vertex_idx.size());
    std::vector<Eigen::Triplet<BasicType>> Ax_mod_triplelets;
    std::vector<Eigen::VectorXi *> left_right(2);
    left_right[0] = &left;
    left_right[1] = &right;
    for (auto &vec : left_right) {
        for (int &idx : (*vec)) {
            for (typename Eigen::SparseMatrix<BasicType, Eigen::RowMajor>::InnerIterator iter(Ax, idx); iter; ++iter) {
                // Subtract mval from existing value
                Ax_mod_triplelets.emplace_back(idx, iter.col(), -iter.value());
                // If the column index is also a boundary index, add 1
                if (idx == iter.col()) {
                    Ax_mod_triplelets.emplace_back(idx, iter.col(), 1.0);
                }
            }
        }
    }
    Ax_mod.setFromTriplets(Ax_mod_triplelets.begin(), Ax_mod_triplelets.end());
    Ax += Ax_mod;
    // modify Ay
    Eigen::SparseMatrix<BasicType, Eigen::RowMajor> Ay_mod(valid_vertex_idx.size(), valid_vertex_idx.size());
    std::vector<Eigen::Triplet<BasicType>> Ay_mod_triplelets;
    std::vector<Eigen::VectorXi *> top_bottom(2);
    top_bottom[0] = &top;
    top_bottom[1] = &bottom;
    for (auto &vec : top_bottom) {
        for (int &idx : (*vec)) {
            for (typename Eigen::SparseMatrix<BasicType, Eigen::RowMajor>::InnerIterator iter(Ay, idx); iter; ++iter) {
                // Subtract mval from existing value
                Ay_mod_triplelets.emplace_back(idx, iter.col(), -iter.value());
                // If the column index is also a boundary index, add 1
                if (idx == iter.col()) {
                    Ay_mod_triplelets.emplace_back(idx, iter.col(), 1.0);
                }
            }
        }
    }
    Ay_mod.setFromTriplets(Ay_mod_triplelets.begin(), Ay_mod_triplelets.end());
    Ay += Ay_mod;
    // slove the generalized Laplace equation
    // Create the solver object
    Eigen::SparseLU<Eigen::SparseMatrix<BasicType, Eigen::RowMajor>, Eigen::COLAMDOrdering<int>> solver;
    // Compute the decomposition
    solver.analyzePattern(Ax);
    solver.factorize(Ax);
    // Check for success
    if (solver.info() != Eigen::Success) {
        std::cerr << "Decomposition failed" << std::endl;
        return false;
    }
    // Solve the system
    square_x = solver.solve(bx); // L need to be complex as well
    // Check for success
    if (solver.info() != Eigen::Success) {
        std::cerr << "Solving failed" << std::endl;
        return false;
    }
    // slove Ay
    // Compute the decomposition
    solver.analyzePattern(Ay);
    solver.factorize(Ay);
    // Check for success
    if (solver.info() != Eigen::Success) {
        std::cerr << "Decomposition failed" << std::endl;
        return false;
    }
    // Solve the system
    square_y = solver.solve(by); // L need to be complex as well
    // Check for success
    if (solver.info() != Eigen::Success) {
        std::cerr << "Solving failed" << std::endl;
        return false;
    }
    if (((square_x.maxCoeff() - 1) > 0.01) || ((0 - square_x.minCoeff() > 0.01)) ||
        ((square_y.maxCoeff() - 1) > 0.01) || ((0 - square_y.minCoeff()) > 0.01)) {
        //        std::cerr << "Invalid conformal mapping" << std::endl;
        return false;
    }
    auto opt_height = [&](BasicType h_opt) -> BasicType {
        Eigen::VectorX<std::complex<BasicType>> mu_results;
        Eigen::Matrix<BasicType, 2, Eigen::Dynamic> square(2, valid_vertex_idx.size());
        square.row(0) = square_x;
        square.row(1) = square_y * h_opt;
        beltrami_coefficient(eigen_patch_local, f_filtered, valid_face_idx, valid_vertex_idx, disk_area, square,
                             mu_results);
        return mu_results.array().abs2().sum();
    };
    const int precision_bits = std::numeric_limits<BasicType>::digits;
    std::uintmax_t max_iter = conformal_mapping_config_ptr->height_opt_iters;
    BasicType min_bound = 0.0;
    BasicType max_bound = 5.0;
    std::pair<BasicType, BasicType> r =
        boost::math::tools::brent_find_minima(opt_height, min_bound, max_bound, precision_bits, max_iter);
    //    std::cout << "x at minimum = " << r.first << ", f(" << r.first << ") = " << r.second << " after " << max_iter
    //              << " iterations. " << std::endl;
    square_y = square_y * r.first;
    Eigen::Matrix<BasicType, 2, Eigen::Dynamic> map;
    map.resize(2, valid_vertex_idx.size());
    map.row(0) = square_x;
    map.row(1) = square_y;
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/map.txt",
                                       Eigen::MatrixXd(map.template cast<double>().transpose()));
    }
    return true;
}

template <typename BasicType>
int ConformalMappingSPH<BasicType>::conformal_interpolation_extraction(
    const Eigen::VectorX<BasicType> &square_x, const Eigen::VectorX<BasicType> &square_y,
    const std::vector<int> &valid_vertex_idx, const Eigen::MatrixX<BasicType> &eigen_patch_local,
    Eigen::VectorX<BasicType> &sph_coeff, int degree, bool is_ground) {
    int sampling_rate;
    if (is_ground) {
        sampling_rate = (SH_table_config_ptr->max_SH_degree + 1) * SH_table_config_ptr->sampling_factor;
    } else {
        sampling_rate = (SH_table_config_ptr->max_SH_degree + 1) * 2 * SH_table_config_ptr->sampling_factor;
    }

    // generate query points
    Eigen::MatrixX<BasicType> sampling_grid;
    curl::mesh_grid<BasicType>(0, square_x.maxCoeff(), sampling_rate, 0, square_y.maxCoeff(), sampling_rate,
                               sampling_grid);
    // interpolation
    BasicType z_boundary[2] = {eigen_patch_local(2, valid_vertex_idx).minCoeff(),
                               eigen_patch_local(2, valid_vertex_idx).maxCoeff()};
    //    Timer timer;
    //    timer.start();
    interp_func_pair F = linear_interpolation_2_func<BasicType>(square_x, square_y,
                                                                eigen_patch_local.row(2).transpose()(valid_vertex_idx));
    //    std::cout << "Interp Func: " << timer.elapsedMilliseconds() << " ms" << std::endl;
    //    timer.start();
    std::pair<std::vector<BasicType>, std::vector<int>> interp_result =
        linear_interpolation_2<BasicType>(sampling_grid, F, z_boundary);
    //    std::cout << "Interp Points: " << timer.elapsedMilliseconds() << " ms" << std::endl;
    // SPH coefficients extraction
    Eigen::MatrixX<BasicType> sph_dirs(interp_result.second.size(), 2);
    sph_dirs.col(0) = ((sampling_grid.col(0) / square_x.maxCoeff() * 2 * M_PI * SH_table_config_ptr->SH_scale).array() +
                       M_PI * (1 - SH_table_config_ptr->SH_scale))(interp_result.second);
    sph_dirs.col(1) = ((sampling_grid.col(1) / square_y.maxCoeff() * M_PI * SH_table_config_ptr->SH_scale).array() +
                       M_PI_2 * (1 - SH_table_config_ptr->SH_scale))(interp_result.second);
    //    timer.start();
    int status;
    sph_coeff = curl::IRF_least_square_table<BasicType>(
        degree,
        Eigen::Map<Eigen::VectorX<BasicType>>(interp_result.first.data(),
                                              static_cast<int>(interp_result.first.size()))(interp_result.second),
        sph_dirs, SH_table_config_ptr->init_SPH_pass, 1, *SH_table_config_ptr, SH_table_config_ptr->medium_thres,
        SH_table_config_ptr->bad_thres, status, SH_table_config_ptr->inv_method);

    //    sph_coeff = curl::IRF_least_square_table<BasicType>(
    //        SH_table_config_ptr->max_SH_degree,
    //        Eigen::Map<Eigen::VectorX<BasicType>>(interp_result.first.data(),
    //                                              static_cast<int>(interp_result.first.size()))(interp_result.second),
    //        sph_dirs, SH_table_config_ptr->init_SPH_pass, 1, *SH_table_config_ptr,
    //        SH_table_config_ptr->inv_method);
    //    std::cout << "SPH Extraction: " << timer.elapsedMilliseconds() << " ms"
    //              << ". Is successful: " << is_successful << std::endl;
    if (debug_config_ptr->is_save_middle_results) {
        FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/sph_coeff.txt",
                                       Eigen::MatrixXd(sph_coeff.template cast<double>()));
    }
    return status;
}

/************************ real time ************************/
/************************ real time ************************/
/************************ real time ************************/

template <typename BasicType>
void ConformalMappingSPH<BasicType>::rt_callback_point_cloud_segments(
    const sensor_msgs::PointCloud2::ConstPtr &segment_msg, const sensor_msgs::PointCloud2::ConstPtr &ground_msg) {
    Timer preprocessing_timer;
    preprocessing_timer.start();
    seg_cloud_ptr->clear();
    seg_patches_idx.clear();
    pcl::fromROSMsg(*segment_msg, *seg_cloud_ptr);
    pcl::fromROSMsg(*ground_msg, *ground_cloud_ptr);
    int original_size = seg_cloud_ptr->size();
    if (conformal_mapping_config_ptr->is_even_segmentation) {
        for (auto &point : seg_cloud_ptr->points) {
            point.intensity = 1;
        }
    }
    *seg_cloud_ptr += *ground_cloud_ptr;
    // filter patches size haven't achieved requirement
    float last_intensity = seg_cloud_ptr->points[0].intensity;
    std::vector<int> idx;
    last_intensity = seg_cloud_ptr->points[0].intensity;
    std::vector<int> seg_patches_idx_tmp;
    for (int i = 0; i < original_size; ++i) {
        if (last_intensity == seg_cloud_ptr->points[i].intensity) {
            seg_patches_idx_tmp.push_back(i);
        }
        if ((last_intensity != seg_cloud_ptr->points[i].intensity) || (i == original_size - 1)) {
            if (seg_patches_idx_tmp.size() >= conformal_mapping_config_ptr->patch_minimum_pts_num) {
                seg_patches_idx.push_back(seg_patches_idx_tmp);
                idx.insert(idx.end(), seg_patches_idx_tmp.begin(), seg_patches_idx_tmp.end());
                seg_patches_idx_tmp.clear();
                seg_patches_idx_tmp.push_back(i);
            } else {
                seg_patches_idx_tmp.clear();
            }
            last_intensity = seg_cloud_ptr->points[i].intensity;
        }
    }
    bool has_ground = false;
    if (!ground_cloud_ptr->empty()) {
        std::vector<int> ground_cloud_idx(ground_cloud_ptr->size());
        std::iota(ground_cloud_idx.begin(), ground_cloud_idx.end(), original_size);
        seg_patches_idx.push_back(ground_cloud_idx);
        has_ground = true;
    }
    if (!idx.empty() || !ground_cloud_ptr->empty()) {
        // publish valid patches clouds
        pcl::PointCloud<PointT> filter_cloud;
        pcl::copyPointCloud(*seg_cloud_ptr, idx, filter_cloud);
        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(filter_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header = segment_msg->header;
        filtered_cloud_msg.header.stamp = ros::Time::now();
        pub_pt_valid_patches.publish(filtered_cloud_msg);
        // inilize the vector
        std::vector<Eigen::MatrixX<BasicType>> point_cloud_vec;
        PointCloudInfo point_cloud_info;
        point_cloud_info.time = ros::Time::now().toSec();
        // filter point cloud
        for (const auto &patch_idx : seg_patches_idx) {
            pcl::PointCloud<PointT>::Ptr patch_cloud_ptr(new pcl::PointCloud<PointT>);
            pcl::copyPointCloud(*seg_cloud_ptr, patch_idx, *patch_cloud_ptr);
            if ((&patch_idx == &seg_patches_idx.back()) && has_ground) {
                if (conformal_mapping_config_ptr->is_voxel_grid_filter) {
                    //                std::cout << "points number before voxel grid filter: " << patch_cloud_ptr->size()
                    //                << std::endl;
                    pcl::VoxelGrid<PointT> sor;
                    sor.setInputCloud(patch_cloud_ptr);
                    sor.setLeafSize(conformal_mapping_config_ptr->leaf_size * 2,
                                    conformal_mapping_config_ptr->leaf_size * 2,
                                    conformal_mapping_config_ptr->leaf_size * 2);
                    sor.filter(*patch_cloud_ptr);
                    //                std::cout << "points number after voxel grid filter: " << patch_cloud_ptr->size()
                    //                << std::endl;
                }
                if (patch_cloud_ptr->size() < conformal_mapping_config_ptr->patch_minimun_pts_num_after_filter) {
                    continue;
                }
            } else {
                if (conformal_mapping_config_ptr->is_voxel_grid_filter) {
                    //                std::cout << "points number before voxel grid filter: " << patch_cloud_ptr->size()
                    //                << std::endl;
                    pcl::VoxelGrid<PointT> sor;
                    sor.setInputCloud(patch_cloud_ptr);
                    sor.setLeafSize(conformal_mapping_config_ptr->leaf_size, conformal_mapping_config_ptr->leaf_size,
                                    conformal_mapping_config_ptr->leaf_size);
                    sor.filter(*patch_cloud_ptr);
                    //                std::cout << "points number after voxel grid filter: " << patch_cloud_ptr->size()
                    //                << std::endl;
                }
                if (patch_cloud_ptr->size() < conformal_mapping_config_ptr->patch_minimun_pts_num_after_filter) {
                    continue;
                }
            }

            Eigen::Map<Eigen::MatrixXf, 0, Eigen::Stride<8, 0>> eigen_patch_ori(
                reinterpret_cast<float *>(patch_cloud_ptr->points.data()), 3, static_cast<int>(patch_cloud_ptr->size()),
                Eigen::Stride<8, 0>());

            Eigen::MatrixXf eigen_patch = eigen_patch_ori; // dim: 3*N

            // if the patch is large, divide it into smaller pieces

            if ((&patch_idx == &seg_patches_idx.back()) && has_ground) {
                std::vector<Eigen::MatrixXf> patch_vec;
                divide_patches(eigen_patch, patch_vec, conformal_mapping_config_ptr->ground_cut_threshold,
                               conformal_mapping_config_ptr->patch_minimun_pts_num_after_filter / 2);
                // Assume eigen_patch is an Eigen::Matrix<double, 3, Eigen::Dynamic>
                Eigen::MatrixXd v_mid = (eigen_patch.colwise() - eigen_patch.rowwise().mean()).cast<double>();
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(v_mid * v_mid.transpose(),
                                                      Eigen::ComputeThinU | Eigen::ComputeThinV);
                Eigen::Matrix3d eigen_rot = svd.matrixV();
                eigen_rot.col(2) = Eigen::Vector3d(eigen_rot.col(0)).cross(Eigen::Vector3d(eigen_rot.col(1)));
                eigen_rot = Eigen::Quaterniond(eigen_rot).normalized().toRotationMatrix();
                if (patch_vec.empty()) {
                    Eigen::Isometry3d T_obj_lidar;
                    get_ground_local_coordinate(eigen_patch, eigen_rot, T_obj_lidar);
                    point_cloud_vec.push_back(eigen_patch.cast<BasicType>());
                    point_cloud_info.T_obj_lidar_vec.push_back(T_obj_lidar);
                    point_cloud_info.is_ground_cloud_vec.push_back(true);
                } else {
                    for (auto &small_patch : patch_vec) {
                        Eigen::Isometry3d T_obj_lidar;
                        get_ground_local_coordinate(small_patch, eigen_rot, T_obj_lidar);
                        point_cloud_vec.push_back(small_patch.cast<BasicType>());
                        point_cloud_info.T_obj_lidar_vec.push_back(T_obj_lidar);
                        point_cloud_info.is_ground_cloud_vec.push_back(true);
                    }
                }
            } else {
                std::vector<Eigen::MatrixXf> patch_vec;
                divide_patches(eigen_patch, patch_vec, conformal_mapping_config_ptr->cut_threshold,
                               conformal_mapping_config_ptr->patch_minimun_pts_num_after_filter / 2);
                if (patch_vec.empty()) {
                    Eigen::Isometry3d T_obj_lidar;
                    get_local_coordinate(eigen_patch, T_obj_lidar);
                    point_cloud_vec.push_back(eigen_patch.cast<BasicType>());
                    point_cloud_info.T_obj_lidar_vec.push_back(T_obj_lidar);
                    point_cloud_info.is_ground_cloud_vec.push_back(false);
                } else {
                    for (auto &small_patch : patch_vec) {
                        Eigen::Isometry3d T_obj_lidar;
                        get_local_coordinate(small_patch, T_obj_lidar);
                        point_cloud_vec.push_back(small_patch.cast<BasicType>());
                        point_cloud_info.T_obj_lidar_vec.push_back(T_obj_lidar);
                        point_cloud_info.is_ground_cloud_vec.push_back(false);
                    }
                }
            }
        }
        if ((!point_cloud_vec.empty()) && (!point_cloud_info.T_obj_lidar_vec.empty())) {
            preprocessing_queue_lock.lock();
            point_cloud_queue.push(point_cloud_vec);
            //            std::cout << "here size:" << point_cloud_vec.size() << std::endl;
            point_cloud_info_queue.push(point_cloud_info);
            preprocessing_queue_lock.unlock();
        }
        //        std::cout << "*********************************************************" << std::endl;
        //        std::cout << "*********************************************************" << std::endl;
        std::cout << "Preprocessing time: " << preprocessing_timer.elapsedMilliseconds() << " ms" << std::endl;
    }
}

// Notice that patches_local_vec is in local coordinate
template <typename BasicType>
void ConformalMappingSPH<BasicType>::rt_conformal_mapping(std::vector<Eigen::MatrixX<BasicType>> &patches_local_vec,
                                                          PointCloudInfo &point_cloud_info,
                                                          Eigen::Isometry3d &T_w_lidar) {
    std::shared_ptr<KeyframeInfo<BasicType>> keyframe_info_ptr =
        std::make_shared<KeyframeInfo<BasicType>>(point_cloud_info.time, T_w_lidar);
    // for visualization
    std::vector<Eigen::MatrixX<BasicType>> patches_w_vec;
    std::vector<std::shared_ptr<PatchInfo<BasicType>>> patch_info_ptr_vec;

    // for debug
    //    std::vector<Eigen::MatrixX<BasicType>> patches_lidar_vec;
    //    std::vector<unsigned int> keys;

    // parallel for loop
#pragma omp parallel for num_threads(conformal_mapping_config_ptr->thread_num) default(none)                           \
    shared(conformal_mapping_config_ptr, SH_table_config_ptr, debug_config_ptr, patches_local_vec, point_cloud_info,   \
               map_attributes_ptr, keyframe_info_ptr, writelock, std::cout, patches_w_vec, patch_info_ptr_vec,         \
               Eigen::all, Eigen::Dynamic, __gnu_cxx::__default_lock_policy)
    for (int i = 0; i < patches_local_vec.size(); ++i) {
        Ubpa::HEMeshInherit mesh_half_edge;
        BasicType boundary_length;
        std::vector<BasicType> boundary_length_segment;
        Eigen::Isometry3d &T_obj_lidar = point_cloud_info.T_obj_lidar_vec[i];
        Eigen::Vector3<BasicType> centroid = T_obj_lidar.inverse().translation().cast<BasicType>();
        Eigen::MatrixX<BasicType> &eigen_patch_local = patches_local_vec[i]; // DIM: 3*N
        Eigen::Matrix<int, Eigen::Dynamic, 3> f;
        std::vector<int> valid_face_idx, valid_vertex_idx, boundary_idx, corner_idx, f_filtered;
        Eigen::MatrixX<BasicType> projection_2d_pts; // DIM: 2*N
        Eigen::VectorX<BasicType> valid_area;
        Eigen::VectorX<BasicType> disk_area;
        Eigen::VectorX<BasicType> square_x;
        Eigen::VectorX<BasicType> square_y;
        Eigen::VectorX<BasicType> sph_coeff;
        if (eigen_patch_local.cols() < 3) {
            continue;
        }
        assert(eigen_patch_local.cols() >= 3);
        std::vector<std::array<float, 3>> colors = {
            //            {1.0f, 0.0f, 0.0f}, // Red
            //            {0.0f, 1.0f, 0.0f}, // Green
            {0.0f, 0.0f, 1.0f} // Blue
                               //            {1.0f, 1.0f, 0.0f}, // Yellow
                               //            {0.0f, 1.0f, 1.0f}  // Cyan
        };
        if (debug_config_ptr->is_save_middle_results) {
            debug_config_ptr->middle_patch_dir =
                debug_config_ptr->middle_results_dir + "/patch_id_" + std::to_string(debug_config_ptr->patch_id);
            curl::create_directory_if_not_exists(debug_config_ptr->middle_patch_dir);
            FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/v_local.txt",
                                           Eigen::MatrixXd(eigen_patch_local.transpose().template cast<double>()));
            FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/T_obj_lidar.txt",
                                           Eigen::MatrixXd(T_obj_lidar.matrix()));
        }
        //        Timer sub_timer;
        // meshing
        //        sub_timer.start();
        perspective_view(eigen_patch_local, centroid, projection_2d_pts);
        //        std::cout << "Perspective: " << sub_timer.elapsedMilliseconds() << " ms" << std::endl;
        //        sub_timer.start();
        bool is_triangulation_success =
            triangulation_patch(eigen_patch_local, projection_2d_pts, f, mesh_half_edge, valid_face_idx,
                                valid_vertex_idx, boundary_idx, f_filtered, boundary_length_segment, boundary_length);
        //        std::cout << "Triangulation: " << sub_timer.elapsedMilliseconds() << " ms" << std::endl;
        if (valid_face_idx.size() >= conformal_mapping_config_ptr->minimum_triangles_after_filter &&
            is_triangulation_success) {
            //            sub_timer.start();
            corner_selection(projection_2d_pts, valid_vertex_idx, boundary_idx, boundary_length, mesh_half_edge,
                             corner_idx);
            //            std::cout << "Corner selection: " << sub_timer.elapsedMilliseconds() << " ms" << std::endl;
            // patch_rectangular_conformal_map
            //            sub_timer.start();
            bool is_success = rectangular_conformal_map(
                eigen_patch_local, boundary_idx, corner_idx, boundary_length_segment, boundary_length, valid_vertex_idx,
                f_filtered, valid_face_idx, mesh_half_edge, valid_area, disk_area, square_x, square_y);
            //            std::cout << "Conformal Mapping: " << sub_timer.elapsedMilliseconds() << " ms" << std::endl;
            if (is_success) {
                // conformal_interpolation
                //                sub_timer.start();
                int status, current_degree;
                if (!point_cloud_info.is_ground_cloud_vec[i]) {
                    current_degree = SH_table_config_ptr->warping_SH_degree;
                    status = conformal_interpolation_extraction(square_x, square_y, valid_vertex_idx, eigen_patch_local,
                                                                sph_coeff, SH_table_config_ptr->max_SH_degree, false);
                } else {
                    current_degree = SH_table_config_ptr->ground_SH_degree;
                    status = conformal_interpolation_extraction(square_x, square_y, valid_vertex_idx, eigen_patch_local,
                                                                sph_coeff, SH_table_config_ptr->ground_SH_degree, true);
                }

                if (status != curl::BAD) {
                    is_success = true;
                } else {
                    is_success = false;
                }
                //                std::cout << "Sampling and SPH Extraction: " << sub_timer.elapsedMilliseconds() << "
                //                ms" << std::endl;
                if (is_success) {
                    // forward conformal mapping
                    interp_func_pair F0 = linear_interpolation_2_func(
                        eigen_patch_local(Eigen::seq(0, 1), valid_vertex_idx).transpose(), square_x);
                    interp_func_pair F1 = linear_interpolation_2_func(
                        eigen_patch_local(Eigen::seq(0, 1), valid_vertex_idx).transpose(), square_y);
                    // inverse conformal mapping
                    interp_func_pair B0 = linear_interpolation_2_func<BasicType>(
                        square_x, square_y, eigen_patch_local.row(0).transpose()(valid_vertex_idx));
                    interp_func_pair B1 = linear_interpolation_2_func<BasicType>(
                        square_x, square_y, eigen_patch_local.row(1).transpose()(valid_vertex_idx));
                    //                    sub_timer.start();
                    std::shared_ptr<PatchProcession<BasicType>> patch_procession_ptr =
                        std::make_shared<PatchProcession<BasicType>>(
                            direct_method_config_ptr, SH_table_config_ptr, debug_config_ptr, std::make_pair(F0, F1),
                            std::make_pair(B0, B1), eigen_patch_local(Eigen::all, valid_vertex_idx).transpose(),
                            square_x, square_y, sph_coeff, current_degree);

                    //                    std::cout << "Points Num: " << valid_vertex_idx.size() * 4 << std::endl;
                    //                    std::cout << "Patch Procession: " << sub_timer.elapsedMilliseconds() << " ms"
                    //                    << std::endl;

                    std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr = std::make_shared<PatchInfo<BasicType>>(
                        keyframe_info_ptr, T_obj_lidar, patch_procession_ptr, point_cloud_info.is_ground_cloud_vec[i],
                        colors[i % colors.size()]);
                    // this is for initialization map_attributes and debug
                    std::vector<double> lower_bound_w(3), upper_bound_w(3);
                    Eigen::Isometry3d T_w_obj = patch_info_ptr->keyframe_ptr->T_w_lidar * T_obj_lidar.inverse();
                    Eigen::MatrixX<BasicType> eigen_patch_world =
                        (T_w_obj.rotation().cast<BasicType>() * eigen_patch_local(Eigen::all, valid_vertex_idx))
                            .colwise() +
                        T_w_obj.translation().cast<BasicType>();
                    lower_bound_w[0] = eigen_patch_world.row(0).minCoeff();
                    lower_bound_w[1] = eigen_patch_world.row(1).minCoeff();
                    lower_bound_w[2] = eigen_patch_world.row(2).minCoeff();
                    upper_bound_w[0] = eigen_patch_world.row(0).maxCoeff();
                    upper_bound_w[1] = eigen_patch_world.row(1).maxCoeff();
                    upper_bound_w[2] = eigen_patch_world.row(2).maxCoeff();
                    // for visualization
                    //                    std::vector<double> lower_bound_lidar(3), upper_bound_lidar(3);
                    //                    Eigen::Isometry3d T_lidar_obj = T_obj_lidar.inverse();
                    //                    Eigen::MatrixX<BasicType> eigen_patch_lidar =
                    //                        (T_lidar_obj.rotation().cast<BasicType>() * eigen_patch_local(Eigen::all,
                    //                        valid_vertex_idx))
                    //                            .colwise() +
                    //                        T_lidar_obj.translation().cast<BasicType>();
                    //                    lower_bound_lidar[0] = eigen_patch_lidar.row(0).minCoeff();
                    //                    lower_bound_lidar[1] = eigen_patch_lidar.row(1).minCoeff();
                    //                    lower_bound_lidar[2] = eigen_patch_lidar.row(2).minCoeff();
                    //                    upper_bound_lidar[0] = eigen_patch_lidar.row(0).maxCoeff();
                    //                    upper_bound_lidar[1] = eigen_patch_lidar.row(1).maxCoeff();
                    //                    upper_bound_lidar[2] = eigen_patch_lidar.row(2).maxCoeff();
                    // multi-thread lock
                    omp_set_lock(&writelock);
                    if (!point_cloud_info.is_ground_cloud_vec[i]) {
                        map_attributes_ptr->insert(patch_info_ptr, lower_bound_w, upper_bound_w);
                    } else {
                        map_attributes_ptr->insert_ground(patch_info_ptr, lower_bound_w, upper_bound_w);
                    }
                    //                    if (debug_config_ptr->is_save_final_results) {
                    //                        keys.push_back(patch_info_ptr->key);
                    //                        // save information for debug
                    //                    }
                    // for visualization
                    patches_w_vec.push_back(eigen_patch_world);
                    patch_info_ptr_vec.push_back(patch_info_ptr);
                    //                    patches_lidar_vec.push_back(eigen_patch_lidar);
                    omp_unset_lock(&writelock);
                    if (debug_config_ptr->is_save_middle_results) {
                        ++debug_config_ptr->patch_id;
                    }
                }
            }
        }
    }
    rt_publish_landmark_cloud(patches_w_vec);
    for (auto &patch_info_ptr : patch_info_ptr_vec) {
        rt_publish_landmark_patch_recons(patch_info_ptr);
    }
    // maintain a fixed size of the map
    while (map_attributes_ptr->keyframe_size() > conformal_mapping_config_ptr->map_window_size &&
           conformal_mapping_config_ptr->map_window_size != -1) {
        for (auto iter = map_attributes_ptr->keyframe_begin(); iter != map_attributes_ptr->keyframe_end(); ++iter) {
            if (map_attributes_ptr->keyframe_size() > conformal_mapping_config_ptr->map_window_size) {
                iter = map_attributes_ptr->erase_keyframe(*iter);
            } else {
                break;
            }
        }
    }
    //    if (debug_config_ptr->is_save_final_results) {
    //        // ToDo: save point clouds of successfully stored in map_attributes
    //        int N_size = 0;
    //        for (auto &patch : patches_w_vec) {
    //            N_size += patch.cols();
    //        }
    //        Eigen::MatrixX<BasicType> patches_lidar_points(3, N_size);
    //        int start_idx = 0;
    //        for (auto &patch : patches_w_vec) {
    //            patches_lidar_points(Eigen::all, Eigen::seq(start_idx, start_idx + patch.cols() - 1)) = patch;
    //            start_idx += patch.cols();
    //        }
    //        // get upper bound and lower bound
    //        Eigen::MatrixX<BasicType> bounding_matrix(keys.size(), 6);
    //        for (int i = 0; i < keys.size(); ++i) {
    //            bounding_matrix(i, 0) = static_cast<BasicType>(map_attributes_ptr->get_AABB(keys[i]).lowerBound[0]);
    //            bounding_matrix(i, 1) = static_cast<BasicType>(map_attributes_ptr->get_AABB(keys[i]).lowerBound[1]);
    //            bounding_matrix(i, 2) = static_cast<BasicType>(map_attributes_ptr->get_AABB(keys[i]).lowerBound[2]);
    //            bounding_matrix(i, 3) = static_cast<BasicType>(map_attributes_ptr->get_AABB(keys[i]).upperBound[0]);
    //            bounding_matrix(i, 4) = static_cast<BasicType>(map_attributes_ptr->get_AABB(keys[i]).upperBound[1]);
    //            bounding_matrix(i, 5) = static_cast<BasicType>(map_attributes_ptr->get_AABB(keys[i]).upperBound[2]);
    //        }
    //        FileReaderBase::write_txt_file("/home/user/project/ros/slam/src/CURL_SLAM_ros/tmp_data/patches_lidar_points.txt",
    //                                       Eigen::MatrixXd(patches_lidar_points.transpose().template cast<double>()));
    //        FileReaderBase::write_txt_file("/home/user/project/ros/slam/src/CURL_SLAM_ros/tmp_data/bounding_matrix.txt",
    //                                       Eigen::MatrixXd(bounding_matrix.template cast<double>()));
    //    }

    //    std::cout << "here" << std::endl;
}

template <typename BasicType>
void ConformalMappingSPH<BasicType>::rt_publish_landmark_cloud(
    const std::vector<Eigen::MatrixX<BasicType>> &landmark_cloud_vec) {
    landmark_cloud.clear();
    pcl::PointXYZI point;
    for (int i = 0; i < landmark_cloud_vec.size(); ++i) {
        point.intensity = static_cast<float>(i);
        for (int j = 0; j < landmark_cloud_vec[i].cols(); ++j) {
            point.x = landmark_cloud_vec[i](0, j);
            point.y = landmark_cloud_vec[i](1, j);
            point.z = landmark_cloud_vec[i](2, j);
            landmark_cloud.push_back(point);
        }
    }
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(landmark_cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    pub_landmark_cloud.publish(cloud_msg);
}

template <typename BasicType>
void ConformalMappingSPH<BasicType>::rt_publish_landmark_patch_recons(
    std::shared_ptr<PatchInfo<BasicType>> &patch_info_ptr) {
    visualization_msgs::Marker patch_recons =
        set_default_marker("map", "recons_patch", patch_info_ptr->key, patch_info_ptr->color);
    Eigen::Isometry3d T_w_obj = patch_info_ptr->keyframe_ptr->T_w_lidar * patch_info_ptr->T_obj_lidar.inverse();
    patch_info_ptr->patch_procession_ptr->I_lock.lock();
    Eigen::MatrixX<BasicType> recons_v_world =
        (T_w_obj.rotation().cast<BasicType>() * patch_info_ptr->patch_procession_ptr->recons_v_local.transpose())
            .colwise() +
        T_w_obj.translation().cast<BasicType>();
    patch_info_ptr->patch_procession_ptr->I_lock.unlock();

    for (int j = 0; j < recons_v_world.cols(); ++j) {
        geometry_msgs::Point pt;
        pt.x = recons_v_world(0, j);
        pt.y = recons_v_world(1, j);
        pt.z = recons_v_world(2, j);
        patch_recons.points.push_back(pt);
    }
    pub_landmark_patch.publish(patch_recons);
}

template <typename BasicType>
visualization_msgs::Marker ConformalMappingSPH<BasicType>::set_default_marker(
    const std::string &frame_id, const std::string &ns, const int &id, const std::array<float, 3> &color, int32_t type,
    const float scale[3], const float position[3], const std::string &text) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1;
    marker.type = type;
    marker.scale.x = scale[0];
    marker.scale.y = scale[1];
    marker.scale.z = scale[2];
    if (type == visualization_msgs::Marker::TEXT_VIEW_FACING) {
        marker.pose.position.x = position[0];
        marker.pose.position.y = position[1];
        marker.pose.position.z = position[2];
        marker.text = text;
    }

    marker.pose.orientation.w = 1.0;
    return marker;
}

/************************ map generation ************************/
/************************ map generation ************************/
/************************ map generation ************************/
template <typename BasicType>
void ConformalMappingSPH<BasicType>::map_gen_conformal_mapping(
    std::vector<Eigen::Isometry3d> T_w_lidar_vec, std::vector<std::vector<Eigen::Isometry3d>> T_obj_lidar_vec_vec,
    std::vector<std::vector<bool>> is_ground_cloud_vec_vec,
    std::vector<std::vector<Eigen::MatrixX<BasicType>>> local_point_cloud_vec_vec, BasicType outlier_patch_thres) {
    // parallel for loop
    for (int i = 0; i < T_w_lidar_vec.size(); ++i) {
#pragma omp parallel for num_threads(omp_get_num_procs()) default(none)                                                \
    shared(conformal_mapping_config_ptr, SH_table_config_ptr, debug_config_ptr, map_attributes_ptr, i, T_w_lidar_vec,  \
               T_obj_lidar_vec_vec, is_ground_cloud_vec_vec, local_point_cloud_vec_vec, outlier_patch_thres,           \
               writelock, std::cout, Eigen::all, Eigen::Dynamic, __gnu_cxx::__default_lock_policy)
        for (int j = 0; j < local_point_cloud_vec_vec[i].size(); ++j) {
            Ubpa::HEMeshInherit mesh_half_edge;
            BasicType boundary_length;
            std::vector<BasicType> boundary_length_segment;
            Eigen::Isometry3d &T_obj_lidar = T_obj_lidar_vec_vec[i][j];
            Eigen::Vector3<BasicType> centroid = T_obj_lidar.inverse().translation().cast<BasicType>();
            Eigen::MatrixX<BasicType> &eigen_patch_local = local_point_cloud_vec_vec[i][j]; // DIM: 3*N
            Eigen::Matrix<int, Eigen::Dynamic, 3> f;
            std::vector<int> valid_face_idx, valid_vertex_idx, boundary_idx, corner_idx, f_filtered;
            Eigen::MatrixX<BasicType> projection_2d_pts; // DIM: 2*N
            Eigen::VectorX<BasicType> valid_area;
            Eigen::VectorX<BasicType> disk_area;
            Eigen::VectorX<BasicType> square_x;
            Eigen::VectorX<BasicType> square_y;
            Eigen::VectorX<BasicType> sph_coeff;
            if (eigen_patch_local.cols() < 3) {
                continue;
            }
            assert(eigen_patch_local.cols() >= 3);
            std::vector<std::array<float, 3>> colors = {
                //            {1.0f, 0.0f, 0.0f}, // Red
                //            {0.0f, 1.0f, 0.0f}, // Green
                {0.0f, 0.0f, 1.0f} // Blue
                                   //            {1.0f, 1.0f, 0.0f}, // Yellow
                                   //            {0.0f, 1.0f, 1.0f}  // Cyan
            };
            if (debug_config_ptr->is_save_middle_results) {
                FileReaderBase::write_txt_file(debug_config_ptr->middle_patch_dir + "/v_local.txt",
                                               Eigen::MatrixXd(eigen_patch_local.transpose().template cast<double>()));
            }
            Timer sub_timer;
            perspective_view(eigen_patch_local, centroid, projection_2d_pts);
            bool is_triangulation_success = triangulation_patch(eigen_patch_local, projection_2d_pts, f, mesh_half_edge,
                                                                valid_face_idx, valid_vertex_idx, boundary_idx,
                                                                f_filtered, boundary_length_segment, boundary_length);
            if (valid_face_idx.size() >= conformal_mapping_config_ptr->minimum_triangles_after_filter &&
                is_triangulation_success) {
                corner_selection(projection_2d_pts, valid_vertex_idx, boundary_idx, boundary_length, mesh_half_edge,
                                 corner_idx);
                bool is_success =
                    rectangular_conformal_map(eigen_patch_local, boundary_idx, corner_idx, boundary_length_segment,
                                              boundary_length, valid_vertex_idx, f_filtered, valid_face_idx,
                                              mesh_half_edge, valid_area, disk_area, square_x, square_y);
                if (is_success) {
                    int status, current_degree;
                    if (!is_ground_cloud_vec_vec[i][j]) {
                        current_degree = SH_table_config_ptr->max_SH_degree;
                        status =
                            conformal_interpolation_extraction(square_x, square_y, valid_vertex_idx, eigen_patch_local,
                                                               sph_coeff, SH_table_config_ptr->max_SH_degree, false);
                    } else {
                        current_degree = SH_table_config_ptr->ground_SH_degree;
                        status =
                            conformal_interpolation_extraction(square_x, square_y, valid_vertex_idx, eigen_patch_local,
                                                               sph_coeff, SH_table_config_ptr->ground_SH_degree, true);
                    }

                    if (status != curl::BAD) {
                        is_success = true;
                    } else {
                        is_success = false;
                    }
                    if (is_success) {
                        // forward conformal mapping
                        interp_func_pair F0 = linear_interpolation_2_func(
                            eigen_patch_local(Eigen::seq(0, 1), valid_vertex_idx).transpose(), square_x);
                        interp_func_pair F1 = linear_interpolation_2_func(
                            eigen_patch_local(Eigen::seq(0, 1), valid_vertex_idx).transpose(), square_y);
                        // inverse conformal mapping
                        interp_func_pair B0 = linear_interpolation_2_func<BasicType>(
                            square_x, square_y, eigen_patch_local.row(0).transpose()(valid_vertex_idx));
                        interp_func_pair B1 = linear_interpolation_2_func<BasicType>(
                            square_x, square_y, eigen_patch_local.row(1).transpose()(valid_vertex_idx));

                        std::shared_ptr<PatchProcession<BasicType>> patch_procession_ptr =
                            std::make_shared<PatchProcession<BasicType>>(
                                direct_method_config_ptr, SH_table_config_ptr, debug_config_ptr, std::make_pair(F0, F1),
                                std::make_pair(B0, B1), sph_coeff, current_degree);

                        std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr = std::make_shared<PatchInfo<BasicType>>(
                            T_w_lidar_vec[i] * T_obj_lidar.inverse(), patch_procession_ptr,
                            is_ground_cloud_vec_vec[i][j], colors[j % colors.size()]);
                        // this is for initialization map_attributes and debug
                        std::vector<double> lower_bound_w(3), upper_bound_w(3);
                        Eigen::Isometry3d T_w_obj = patch_info_ptr->map_gen_T_w_obj;
                        Eigen::MatrixX<BasicType> eigen_patch_world =
                            (T_w_obj.rotation().cast<BasicType>() * eigen_patch_local(Eigen::all, valid_vertex_idx))
                                .colwise() +
                            T_w_obj.translation().cast<BasicType>();
                        lower_bound_w[0] = eigen_patch_world.row(0).minCoeff();
                        lower_bound_w[1] = eigen_patch_world.row(1).minCoeff();
                        lower_bound_w[2] = eigen_patch_world.row(2).minCoeff();
                        upper_bound_w[0] = eigen_patch_world.row(0).maxCoeff();
                        upper_bound_w[1] = eigen_patch_world.row(1).maxCoeff();
                        upper_bound_w[2] = eigen_patch_world.row(2).maxCoeff();
                        omp_set_lock(&writelock);
                        std::vector<std::pair<unsigned int, double>> intersected_pairs =
                            map_attributes_ptr->query_data_association_overlap(lower_bound_w, upper_bound_w);
                        omp_unset_lock(&writelock);
                        // multi-thread lock
                        if (!intersected_pairs.empty()) {
                            if (new_landmark_add_judge(intersected_pairs[0].first, intersected_pairs[0].second,
                                                       lower_bound_w, upper_bound_w,
                                                       (*map_attributes_ptr)[intersected_pairs[0].first],
                                                       T_w_lidar_vec[i], T_obj_lidar, outlier_patch_thres)) {
                                omp_set_lock(&writelock);
                                if (!is_ground_cloud_vec_vec[i][j]) {
                                    patch_info_ptr->data_asso_counter = outlier_patch_thres;
                                    map_attributes_ptr->map_gen_insert(patch_info_ptr, lower_bound_w, upper_bound_w);
                                } else {
                                    map_attributes_ptr->map_gen_insert_ground(patch_info_ptr, lower_bound_w,
                                                                              upper_bound_w);
                                }
                                omp_unset_lock(&writelock);
                            }
                        } else {
                            omp_set_lock(&writelock);
                            if (!is_ground_cloud_vec_vec[i][j]) {
                                map_attributes_ptr->map_gen_insert(patch_info_ptr, lower_bound_w, upper_bound_w);
                            } else {
                                map_attributes_ptr->map_gen_insert_ground(patch_info_ptr, lower_bound_w, upper_bound_w);
                            }
                            omp_unset_lock(&writelock);
                        }
                    }
                }
            }
        }
        std::cout << "Loading middle map: " << std::to_string(double(i) / double(T_w_lidar_vec.size() - 1) * 100) << "%"
                  << std::endl;
    }
}

template <typename BasicType>
bool ConformalMappingSPH<BasicType>::new_landmark_add_judge(
    const unsigned int &key, const double &IoU, const std::vector<double> &query_lower_bound,
    const std::vector<double> &query_upper_bound, std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr,
    const Eigen::Isometry3d T_w_j, const Eigen::Isometry3d &T_obj_lidar, BasicType outlier_patch_thres) {
    if (IoU > 0.8) {
        if (!patch_info_ptr->is_ground && patch_info_ptr->data_asso_counter < outlier_patch_thres) {
            return false;
        }
        // since the IoU is very high, it could be the same object in different view, so they should be different
        // patch
        Eigen::Vector3d centroid_w = (T_w_j * T_obj_lidar.inverse()).translation();
        if ((centroid_w - patch_info_ptr->get_centroid_w()).norm() < 0.5) {
            return false;
        } else {
            return true;
        }
    } else {
        std::vector<double> map_lower_bound;
        std::vector<double> map_upper_bound;
        if (!patch_info_ptr->is_ground) {
            // get the original box size
            map_lower_bound = map_attributes_ptr->get_AABB(key).lowerBound;
            map_lower_bound[0] += aabb_config_ptr->extra_skin_thickness;
            map_lower_bound[1] += aabb_config_ptr->extra_skin_thickness;
            map_lower_bound[2] += aabb_config_ptr->extra_skin_thickness;
            map_upper_bound = map_attributes_ptr->get_AABB(key).upperBound;
            map_upper_bound[0] -= aabb_config_ptr->extra_skin_thickness;
            map_upper_bound[1] -= aabb_config_ptr->extra_skin_thickness;
            map_upper_bound[2] -= aabb_config_ptr->extra_skin_thickness;
            std::vector<double> size(3);
            size[0] = map_upper_bound[0] - map_lower_bound[0];
            size[1] = map_upper_bound[1] - map_lower_bound[1];
            size[2] = map_upper_bound[2] - map_lower_bound[2];
            map_lower_bound[0] +=
                (size[0] / (1 - 2 * aabb_config_ptr->skin_thickness) * aabb_config_ptr->skin_thickness);
            map_lower_bound[1] +=
                (size[1] / (1 - 2 * aabb_config_ptr->skin_thickness) * aabb_config_ptr->skin_thickness);
            map_lower_bound[2] +=
                (size[2] / (1 - 2 * aabb_config_ptr->skin_thickness) * aabb_config_ptr->skin_thickness);
            map_upper_bound[0] -=
                (size[0] / (1 - 2 * aabb_config_ptr->skin_thickness) * aabb_config_ptr->skin_thickness);
            map_upper_bound[1] -=
                (size[1] / (1 - 2 * aabb_config_ptr->skin_thickness) * aabb_config_ptr->skin_thickness);
            map_upper_bound[2] -=
                (size[2] / (1 - 2 * aabb_config_ptr->skin_thickness) * aabb_config_ptr->skin_thickness);
        } else {
            map_lower_bound = map_attributes_ptr->get_AABB(key).lowerBound;
            map_upper_bound = map_attributes_ptr->get_AABB(key).upperBound;
        }
        bool is_query_box_in_map_box =
            query_lower_bound[0] > map_lower_bound[0] && query_lower_bound[1] > map_lower_bound[1] &&
            query_lower_bound[2] > map_lower_bound[2] && query_upper_bound[0] < map_upper_bound[0] &&
            query_upper_bound[1] < map_upper_bound[1] && query_upper_bound[2] < map_upper_bound[2];
        bool is_map_box_in_query_box =
            query_lower_bound[0] < map_lower_bound[0] && query_lower_bound[1] < map_lower_bound[1] &&
            query_lower_bound[2] < map_lower_bound[2] && query_upper_bound[0] > map_upper_bound[0] &&
            query_upper_bound[1] > map_upper_bound[1] && query_upper_bound[2] > map_upper_bound[2];
        if (is_query_box_in_map_box || is_map_box_in_query_box) {
            return false; // if the newer box is inside of the map box
        } else {
            // calculate the IoU
            double IoU_value = curl::IoU_3d(map_lower_bound, map_upper_bound, query_lower_bound, query_upper_bound);
            if (!patch_info_ptr->is_ground && patch_info_ptr->data_asso_counter < outlier_patch_thres &&
                IoU_value > 0) {
                return false;
            }
            // decide whether this should be initlize as a new key frame
            bool is_add_new_landmark;
            if (IoU_value < debug_config_ptr->middle_map_highest_IoU_new_landmark_thres) {
                return true;
            } else {
                return false;
            }
        }
    }
}

template class ConformalMappingSPH<BT>; // this is very important