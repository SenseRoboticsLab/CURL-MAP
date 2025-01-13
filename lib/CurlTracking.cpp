//
// Created by zkc on 06/06/23.
//

#include "CurlTracking.h"

template <typename BasicType>
CurlTracking<BasicType>::CurlTracking(ros::NodeHandle *nh,
                                      std::shared_ptr<CURL_TRACKING_CONFIG> _curl_tracking_config_ptr,
                                      std::shared_ptr<AABB_CONFIG> _aabb_config_ptr,
                                      std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                                      std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                                      std::shared_ptr<ConformalMappingSPH<BasicType>> _conformal_mapping_ptr)
    : curl_tracking_config_ptr(_curl_tracking_config_ptr), aabb_config_ptr(_aabb_config_ptr),
      SH_table_config_ptr(_SH_table_config_ptr), conformal_mapping_ptr(_conformal_mapping_ptr),
      debug_config_ptr(_debug_config_ptr) {
    pub_trajectory = nh->advertise<nav_msgs::Path>("/curl/trajectory", 1, true);
    pub_cloud = nh->advertise<sensor_msgs::PointCloud2>("/curl/cloud", 10);
    pub_used_cloud = nh->advertise<sensor_msgs::PointCloud2>("/curl/used_cloud", 10);
    pub_bounding_box = nh->advertise<visualization_msgs::Marker>("/curl/bounding_box", 10);
    trajectory.header.frame_id = "map";
    T_w_j.setIdentity();
    //    T_w_j_tangent = Sophus::SE3<double>(T_w_j).log().matrix();
    T_w_j_1.setIdentity();
    T_j_1_j.setIdentity();
    is_initial = true;
    frame_id = 0;
    number_patches = 0;
    if (debug_config_ptr->is_save_middle_results) {
        debug_config_ptr->middle_results_dir = debug_config_ptr->results_dir + "/middle_results";
        curl::create_directory_if_not_exists(debug_config_ptr->middle_results_dir);
    }
}

template <typename BasicType> void CurlTracking<BasicType>::run() {
    ros::Rate rate(curl_tracking_config_ptr->frequency);
    while (ros::ok()) {
        // initialize a keyframe before entering following optimization
        if (!conformal_mapping_ptr->point_cloud_queue.empty() &&
            !conformal_mapping_ptr->point_cloud_info_queue.empty() && is_initial) {
            std::vector<Eigen::MatrixX<BasicType>> point_cloud_vec = conformal_mapping_ptr->point_cloud_queue.front();
            PointCloudInfo point_cloud_info = conformal_mapping_ptr->point_cloud_info_queue.front();
            PointCloudInfo new_point_cloud_info;
            // update the queue
            conformal_mapping_ptr->preprocessing_queue_lock.lock();
            conformal_mapping_ptr->point_cloud_queue.pop();
            conformal_mapping_ptr->point_cloud_info_queue.pop();
            conformal_mapping_ptr->preprocessing_queue_lock.unlock();
            // transform to local coordinate
            std::vector<Eigen::MatrixX<BasicType>> point_cloud_local_vec;
            point_cloud_local_vec.reserve(point_cloud_vec.size());
            for (int i = 0; i < point_cloud_vec.size(); ++i) {
                if (point_cloud_vec[i].cols() < curl_tracking_config_ptr->minimum_points_to_add_new_patch) {
                    continue;
                }
                point_cloud_local_vec.push_back(
                    (point_cloud_info.T_obj_lidar_vec[i].rotation().matrix().cast<BasicType>() * point_cloud_vec[i])
                        .colwise() +
                    point_cloud_info.T_obj_lidar_vec[i].translation().matrix().cast<BasicType>());
                new_point_cloud_info.T_obj_lidar_vec.push_back(point_cloud_info.T_obj_lidar_vec[i]);
                new_point_cloud_info.is_ground_cloud_vec.push_back(point_cloud_info.is_ground_cloud_vec[i]);
            }
            // do conformal mapping generate a keyframe
            Eigen::Isometry3d T_w_lidar = Eigen::Isometry3d::Identity();
            Timer keyframe_init_timer;
            keyframe_init_timer.start();
            conformal_mapping_ptr->rt_conformal_mapping(point_cloud_local_vec, new_point_cloud_info, T_w_lidar);
            std::cout << "Keyframe initialization: " << keyframe_init_timer.elapsedMilliseconds() << " ms" << std::endl;
            //            std::cout << point_cloud_local_vec.size() << std::endl;
            is_initial = false;
            std::cout << "frame id: " << frame_id << std::endl;
            if (debug_config_ptr->is_save_final_results) {
                // save trajectory
                std::string trajectory_folder = debug_config_ptr->results_dir + "/" + debug_config_ptr->data_set +
                                                "_trajectory_" + debug_config_ptr->seq;
                curl::create_directory_if_not_exists(trajectory_folder);
                FileReaderBase::write_txt_file(trajectory_folder + "/" + std::to_string(frame_id) + "_T_w_j.txt",
                                               Eigen::MatrixXd(T_w_j));
                //                std::cout << "here" << std::endl;
            }
            ++frame_id;
        }
        // read new poind cloud patches
        // initialize the data asscoiation
        // start a pose optimization
        if (!conformal_mapping_ptr->point_cloud_queue.empty() &&
            !conformal_mapping_ptr->point_cloud_info_queue.empty() && !is_initial) {
            //            Timer tracking_timer;
            if (debug_config_ptr->is_evaluate_time) {
                total_timer.start();
            }
            if (debug_config_ptr->is_save_middle_results) {
                debug_config_ptr->middle_data_asso_dir =
                    debug_config_ptr->middle_results_dir + "/data_asso_frame_" + std::to_string(frame_id);
                curl::create_directory_if_not_exists(debug_config_ptr->middle_data_asso_dir);
            }
            // read a new point cloud patches
            std::vector<Eigen::MatrixX<BasicType>> point_cloud_vec = conformal_mapping_ptr->point_cloud_queue.front();
            PointCloudInfo point_cloud_info = conformal_mapping_ptr->point_cloud_info_queue.front();
            assert(point_cloud_vec.size() == point_cloud_info.is_ground_cloud_vec.size() &&
                   point_cloud_vec.size() == point_cloud_info.T_obj_lidar_vec.size());
            // update the queue
            conformal_mapping_ptr->preprocessing_queue_lock.lock();
            conformal_mapping_ptr->point_cloud_queue.pop();
            conformal_mapping_ptr->point_cloud_info_queue.pop();
            conformal_mapping_ptr->preprocessing_queue_lock.unlock();
            // get the threshold for the max distance of the new landmark
            double max_new_landmark_distance;
            if (curl_tracking_config_ptr->IQR_max_distance_of_landmarks != -100) {
                std::vector<double> centroid_distance_to_lidar;
                for (const auto &T_obj_lidar : point_cloud_info.T_obj_lidar_vec) {
                    centroid_distance_to_lidar.push_back(T_obj_lidar.inverse().translation().norm());
                }
                max_new_landmark_distance = curl::IQR_thres_with_bdy(
                    centroid_distance_to_lidar, curl_tracking_config_ptr->IQR_max_distance_of_landmarks, 5);
            }
            // generate the keyframe
            // update the current pose
            // transform to local coordinate
            // data-association
            // add to optimization block
            //            tracking_timer.start();
            // define the optimization problem
            ceres::Problem problem;
            // two problem for two step optimization
            ceres::Problem problem_ground;
            ceres::Problem problem_seg;
            // for optimization
            SE3_manifold_x_y_yall = SE3ManifoldXYYall::Create();
            SE3_manifold_z_roll_pitch = SE3ManifoldZRollPitch::Create();
            //            SE3_manifold_x_y_yall = SE3Manifold::Create();
            //            SE3_manifold_z_roll_pitch = SE3Manifold::Create();
            SE3_manifold = SE3Manifold::Create();
            ground_loss_function = new ceres::LossFunctionWrapper(
                new ceres::HuberLoss(curl_tracking_config_ptr->kernel_threshold), ceres::DO_NOT_TAKE_OWNERSHIP);
            seg_loss_function = new ceres::LossFunctionWrapper(
                new ceres::HuberLoss(curl_tracking_config_ptr->kernel_threshold), ceres::DO_NOT_TAKE_OWNERSHIP);
            loss_function = new ceres::LossFunctionWrapper(
                new ceres::HuberLoss(curl_tracking_config_ptr->kernel_threshold), ceres::DO_NOT_TAKE_OWNERSHIP);

            // provide initial guess for the current T_w_j
            T_w_j = T_w_j_1 * T_j_1_j;
            //            T_w_j_tangent = Sophus::SE3<double>(T_w_j).log().matrix();

            // this part of the code doing data-association only
            if (debug_config_ptr->is_evaluate_time) {
                data_asso_timer.start();
            }
            // new data-association starts here
            std::vector<double> seg_scores;
            std::vector<double> ground_scores;

            for (int i = 0; i < point_cloud_vec.size(); ++i) {
                Eigen::MatrixX<BasicType> point_cloud_obj =
                    (point_cloud_info.T_obj_lidar_vec[i].rotation().matrix().cast<BasicType>() * point_cloud_vec[i])
                        .colwise() +
                    point_cloud_info.T_obj_lidar_vec[i].translation().matrix().cast<BasicType>();
                Eigen::MatrixX<BasicType> point_cloud_w =
                    (T_w_j(Eigen::seq(0, 2), Eigen::seq(0, 2)).cast<BasicType>() * point_cloud_vec[i]).colwise() +
                    T_w_j(Eigen::seq(0, 2), 3).cast<BasicType>();

                std::vector<double> lower_bound_w(3);
                lower_bound_w[0] = point_cloud_w.row(0).minCoeff();
                lower_bound_w[1] = point_cloud_w.row(1).minCoeff();
                lower_bound_w[2] = point_cloud_w.row(2).minCoeff();
                std::vector<double> upper_bound_w(3);
                upper_bound_w[0] = point_cloud_w.row(0).maxCoeff();
                upper_bound_w[1] = point_cloud_w.row(1).maxCoeff();
                upper_bound_w[2] = point_cloud_w.row(2).maxCoeff();
                point_cloud_info.bounding_box_pair_vec.emplace_back(lower_bound_w, upper_bound_w);
                omp_set_lock(&conformal_mapping_ptr->writelock);
                std::vector<std::pair<unsigned int, double>> intersected_pairs =
                    conformal_mapping_ptr->map_attributes_ptr->query_data_association_overlap(lower_bound_w,
                                                                                              upper_bound_w);
                omp_unset_lock(&conformal_mapping_ptr->writelock);
                Eigen::Vector3d centroid_w = (T_w_j * point_cloud_info.T_obj_lidar_vec[i].inverse()).translation();
                std::vector<std::pair<unsigned int, DataInfo>> intersected_score_pairs_vec_tmp;
                for (auto pair : intersected_pairs) {
                    if (conformal_mapping_ptr->map_attributes_ptr->get_curr_tree_key() < pair.first) {
                        std::cerr << "Error: current tree key should be the largest number" << std::endl;
                    }
                    if ((conformal_mapping_ptr->map_attributes_ptr->get_curr_tree_key() - pair.first) <
                            curl_tracking_config_ptr->largest_tree_key_difference ||
                        curl_tracking_config_ptr->largest_tree_key_difference == -1) {
                        std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr =
                            (*conformal_mapping_ptr->map_attributes_ptr)[pair.first];
                        if (patch_info_ptr->is_ground != point_cloud_info.is_ground_cloud_vec[i]) {
                            continue;
                        }
                        double centroid_distance = (centroid_w - patch_info_ptr->get_centroid_w()).norm();
                        if (point_cloud_info.is_ground_cloud_vec[i]) {
                            auto the_score = curl::calculate_score<double>(
                                curl_tracking_config_ptr->weight_of_IoU,
                                curl_tracking_config_ptr->largest_centroid_distance, pair.second, centroid_distance);
                            ground_scores.push_back(the_score);
                            intersected_score_pairs_vec_tmp.emplace_back(pair.first, DataInfo(the_score, pair.second));
                        } else {
                            auto the_score = curl::calculate_score<double>(
                                curl_tracking_config_ptr->weight_of_IoU,
                                curl_tracking_config_ptr->largest_centroid_distance, pair.second, centroid_distance);
                            seg_scores.push_back(the_score);
                            intersected_score_pairs_vec_tmp.emplace_back(pair.first, DataInfo(the_score, pair.second));
                        }
                    }
                }
                point_cloud_info.intersected_score_pairs_vec.push_back(intersected_score_pairs_vec_tmp);
            }
            // calculate dynamic threshold
            double seg_score_thres = 0;
            if (seg_scores.size() > 1) {
                seg_score_thres =
                    curl::IQR_thres_with_bdy<double>(seg_scores, curl_tracking_config_ptr->IQR_scale_factor,
                                                     curl_tracking_config_ptr->min_num_seg_pairs);
            }
            double ground_score_thres = 0;
            if (ground_scores.size() > 1) {
                ground_score_thres =
                    curl::IQR_thres_with_bdy<double>(ground_scores, curl_tracking_config_ptr->IQR_scale_factor,
                                                     curl_tracking_config_ptr->min_num_ground_pairs);
            }

            double weight_corrector = 1;
            if (seg_score_thres != 0 && ground_score_thres != 0) {
                weight_corrector = seg_score_thres / ground_score_thres;
            }
            // for new landmark addition
            //            std::vector<int> tmp_idx(point_cloud_vec.size());
            //            std::iota(tmp_idx.begin(), tmp_idx.end(), 0);
            //            std::set<int> new_box_idx(tmp_idx.begin(), tmp_idx.end());
            // TODO: 1. add middle_map_new_box_map to detect which patch need to be added into the middle map
            std::unordered_map<int, double> new_box_map;
            std::set<int> middle_map_new_box_set;
            for (int idx = 0; idx < point_cloud_vec.size(); ++idx) {
                new_box_map[idx] = 0;
                middle_map_new_box_set.insert(idx);
            }
            // for visualization
            std::vector<std::pair<std::vector<double>, std::vector<double>>> initial_query_bounding_box_vec(
                point_cloud_vec.size());
            std::vector<ScanDataAsso<BasicType>> initial_scan_data_asso_vec(point_cloud_vec.size(), -1);
            // detect whether has ground or seg pairs
            // reject invalid overlap pairs
            // TODO: 2. modify the following for loop to make sure middle map can work well
            for (int i = 0; i < point_cloud_vec.size(); ++i) {
                if (!point_cloud_info.intersected_score_pairs_vec[i].empty()) {
                    // add information to ScanDataAsso
                    initial_scan_data_asso_vec[i].point_idx = i;
                    for (auto &pair : point_cloud_info.intersected_score_pairs_vec[i]) {
                        // for new landmark addition
                        if (new_box_map.find(i) != new_box_map.end()) {
                            if (pair.second.IoU > new_box_map[i]) {
                                new_box_map[i] = pair.second.IoU;
                            }
                        }
                        std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr =
                            (*conformal_mapping_ptr->map_attributes_ptr)[pair.first];
                        double score_thres =
                            point_cloud_info.is_ground_cloud_vec[i] ? ground_score_thres : seg_score_thres;
                        if (pair.second.score > score_thres) {
                            if (!new_landmark_add_judge(
                                     pair.first, pair.second.IoU, point_cloud_info.bounding_box_pair_vec[i].first,
                                     point_cloud_info.bounding_box_pair_vec[i].second, patch_info_ptr,
                                     point_cloud_info.T_obj_lidar_vec[i], max_new_landmark_distance)
                                     .first) {
                                if (!curl_tracking_config_ptr->is_frame_to_frame) {
                                    new_box_map.erase(i);
                                }
                                middle_map_new_box_set.erase(i);
                                // for visualization
                                initial_query_bounding_box_vec[i] = point_cloud_info.bounding_box_pair_vec[i];
                                // add information to ScanDataAsso
                                initial_scan_data_asso_vec[i].patch_info_ptr_vec.push_back(patch_info_ptr);
                                // add information to patch_info_ptr
                                patch_info_ptr->data_asso.points_idx_vec.push_back(i);
                                patch_info_ptr->data_asso.scores.push_back(pair.second.score);
                                if (curl_tracking_config_ptr->is_select_the_best_pair_only) {
                                    break;
                                }
                            } else {
                                if (new_box_map.find(i) != new_box_map.end()) {
                                    new_box_map[i] = 0; // must give its 0 since original IoU is very large, this
                                                        // priority need to increase
                                }
                            }
                            //                            std::cout << "successed association" << std::endl;
                        } else {
                            //                            std::cout << "failed association" << std::endl;
                            // further detection to check whether it can be add as a new keyframe
                            std::pair<bool, bool> judge_pair = new_landmark_add_judge(
                                pair.first, pair.second.IoU, point_cloud_info.bounding_box_pair_vec[i].first,
                                point_cloud_info.bounding_box_pair_vec[i].second, patch_info_ptr,
                                point_cloud_info.T_obj_lidar_vec[i], max_new_landmark_distance);
                            if (!judge_pair.first) {
                                if (!curl_tracking_config_ptr->is_frame_to_frame) {
                                    new_box_map.erase(i);
                                }
                            }
                            if (!judge_pair.second || curl_tracking_config_ptr->is_frame_to_frame) {
                                middle_map_new_box_set.erase(i);
                            }
                            break;
                        }
                    }
                } else {
                    middle_map_new_box_set.erase(i);
                    if (point_cloud_info.T_obj_lidar_vec[i].inverse().translation().norm() >
                            max_new_landmark_distance &&
                        curl_tracking_config_ptr->IQR_max_distance_of_landmarks != -100) {
                        if (!curl_tracking_config_ptr->is_frame_to_frame) {
                            new_box_map.erase(i);
                        }
                    }
                }
            }
            // update this to avoid too many patches
            int true_ground_num = 0;
            int true_seg_num = 0;
            std::set<std::tuple<int, double, std::shared_ptr<PatchInfo<BasicType>>>, TupleCompare<BasicType>>
                ground_sets;
            std::set<std::tuple<int, double, std::shared_ptr<PatchInfo<BasicType>>>, TupleCompare<BasicType>> seg_sets;
            // update data-association
            for (int i = 0; i < initial_scan_data_asso_vec.size(); ++i) {
                if (!initial_scan_data_asso_vec[i].patch_info_ptr_vec.empty()) {
                    if (curl_tracking_config_ptr->is_select_the_best_pair_only) {
                        std::vector<int> sorted_scan_data_asso_idx =
                            initial_scan_data_asso_vec[i].sort_according_to_score();
                        std::shared_ptr<PatchInfo<BasicType>> highest_score_patch_info_ptr =
                            initial_scan_data_asso_vec[i].patch_info_ptr_vec[sorted_scan_data_asso_idx[0]];
                        std::vector<int> sorted_map_data_asso =
                            highest_score_patch_info_ptr->data_asso.sort_according_to_score(
                                curl_tracking_config_ptr->weight_of_IoU,
                                curl_tracking_config_ptr->largest_centroid_distance);
                        // if both are the best selection for each other then data-association successed
                        if (initial_scan_data_asso_vec[i].point_idx ==
                            highest_score_patch_info_ptr->data_asso.points_idx_vec[sorted_map_data_asso[0]]) {
                            // update data-association information for update sph coeff and visualization
                            if (point_cloud_info.is_ground_cloud_vec[i]) {
                                ground_sets.insert(std::make_tuple(
                                    i, highest_score_patch_info_ptr->data_asso.scores[sorted_map_data_asso[0]],
                                    highest_score_patch_info_ptr));
                            } else {
                                seg_sets.insert(std::make_tuple(
                                    i, highest_score_patch_info_ptr->data_asso.scores[sorted_map_data_asso[0]],
                                    highest_score_patch_info_ptr));
                            }
                        }
                    } else {
                        for (auto &patch_info_ptr : initial_scan_data_asso_vec[i].patch_info_ptr_vec) {
                            // update data-association information for update sph coeff and visualization
                            if (point_cloud_info.is_ground_cloud_vec[i]) {
                                auto it = std::find(patch_info_ptr->data_asso.points_idx_vec.begin(),
                                                    patch_info_ptr->data_asso.points_idx_vec.end(), i);
                                assert(it != patch_info_ptr->data_asso.points_idx_vec.end());
                                ground_sets.insert(std::make_tuple(
                                    i,
                                    patch_info_ptr->data_asso
                                        .scores[std::distance(patch_info_ptr->data_asso.points_idx_vec.begin(), it)],
                                    patch_info_ptr));
                            } else {
                                auto it = std::find(patch_info_ptr->data_asso.points_idx_vec.begin(),
                                                    patch_info_ptr->data_asso.points_idx_vec.end(), i);
                                assert(it != patch_info_ptr->data_asso.points_idx_vec.end());
                                seg_sets.insert(std::make_tuple(
                                    i,
                                    patch_info_ptr->data_asso
                                        .scores[std::distance(patch_info_ptr->data_asso.points_idx_vec.begin(), it)],
                                    patch_info_ptr));
                            }
                        }
                    }
                }
            }
            // control the number of pairs, patch with higher IoU gets higher priority
            //            std::vector<int> init_succeed_idx;
            std::vector<std::tuple<int, double, std::shared_ptr<PatchInfo<BasicType>>>> succeed_tuple_vec;
            int tmp_counter = 0;
            for (const auto &ground_set : ground_sets) {
                if (tmp_counter < curl_tracking_config_ptr->max_ground_pairs ||
                    curl_tracking_config_ptr->max_ground_pairs == -1) {
                    succeed_tuple_vec.push_back(ground_set);
                    ++tmp_counter;
                    ++true_ground_num;
                } else {
                    break;
                }
            }
            tmp_counter = 0;
            for (const auto &seg_set : seg_sets) {
                if (tmp_counter < curl_tracking_config_ptr->max_seg_pairs ||
                    curl_tracking_config_ptr->max_seg_pairs == -1) {
                    succeed_tuple_vec.push_back(seg_set);
                    ++tmp_counter;
                    ++true_seg_num;
                } else {
                    break;
                }
            }
            number_patches += (true_ground_num + true_seg_num);
            std::cout << "Average associated patches per scan:  " << double(number_patches) / double(frame_id)
                      << std::endl;
            //            std::cout << "true ground: " << true_ground_num << " true seg: " << true_seg_num << std::endl;
            // add these into residual block
            // update this for visualization
            std::vector<int> succeed_idx;
            //            // for sph coeff updation
            //            std::vector<std::vector<unsigned int>> data_association_keys(point_cloud_vec.size());
            //            std::vector<unsigned int> data_association_keys_tmp;
            // for sph coeff updation
            std::vector<std::shared_ptr<PatchInfo<BasicType>>> patch_info_ptr_vec;
            std::vector<Eigen::MatrixX<BasicType>> point_cloud_obj_vec;
            // for visualization
            std::vector<std::pair<std::vector<double>, std::vector<double>>> query_bounding_box_vec;
            std::vector<std::pair<std::vector<double>, std::vector<double>>> map_bounding_box_vec;
            // if patches number of problem_seg or problem_ground is too small, use one-step optimization
            for (auto &succeed_tuple : succeed_tuple_vec) {
                int i = std::get<0>(succeed_tuple);
                double weight = std::get<1>(succeed_tuple);
                std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr = std::get<2>(succeed_tuple);
                int opt_step = 1;
                if (point_cloud_info.is_ground_cloud_vec[i]) {
                    opt_step = curl_tracking_config_ptr->ground_opt_step;
                    weight *= weight_corrector;
                }
                // add pairs into residual block
                Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_o_w =
                    (patch_info_ptr->T_obj_lidar * patch_info_ptr->keyframe_ptr->T_w_lidar.inverse()).matrix();
                for (int k = 0; k < point_cloud_vec[i].cols(); k += opt_step) {
                    Eigen::Vector3d pts = (point_cloud_vec[i].col(k)).template cast<double>();
                    ceres::CostFunction *couple_fix_sph_cost_func;
                    if (curl_tracking_config_ptr->is_two_step_opt &&
                        (true_ground_num > curl_tracking_config_ptr->min_num_ground_pairs) &&
                        (true_seg_num > curl_tracking_config_ptr->min_num_seg_pairs)) {
                        if (curl_tracking_config_ptr->is_use_score_as_weight) {
                            couple_fix_sph_cost_func = CoupleFixSphFactor<BasicType>::Create(
                                T_o_w, pts, patch_info_ptr->patch_procession_ptr.get(),
                                curl_tracking_config_ptr->max_valid_residual, weight);
                        } else {
                            couple_fix_sph_cost_func = CoupleFixSphFactor<BasicType>::Create(
                                T_o_w, pts, patch_info_ptr->patch_procession_ptr.get(),
                                curl_tracking_config_ptr->max_valid_residual);
                        }
                        if (curl_tracking_config_ptr->is_robust_kernel) {
                            if (point_cloud_info.is_ground_cloud_vec[i]) {
                                problem_ground.AddResidualBlock(couple_fix_sph_cost_func, ground_loss_function,
                                                                T_w_j.data());
                            } else {
                                problem_seg.AddResidualBlock(couple_fix_sph_cost_func, seg_loss_function, T_w_j.data());
                            }
                        } else {
                            if (point_cloud_info.is_ground_cloud_vec[i]) {
                                problem_ground.AddResidualBlock(couple_fix_sph_cost_func, nullptr, T_w_j.data());

                            } else {
                                problem_seg.AddResidualBlock(couple_fix_sph_cost_func, nullptr, T_w_j.data());
                            }
                        }
                    } else {
                        if (curl_tracking_config_ptr->is_use_score_as_weight) {
                            couple_fix_sph_cost_func = CoupleFixSphFactor<BasicType>::Create(
                                T_o_w, pts, patch_info_ptr->patch_procession_ptr.get(),
                                curl_tracking_config_ptr->max_valid_residual, weight);
                        } else {
                            couple_fix_sph_cost_func = CoupleFixSphFactor<BasicType>::Create(
                                T_o_w, pts, patch_info_ptr->patch_procession_ptr.get(),
                                curl_tracking_config_ptr->max_valid_residual);
                        }
                        if (curl_tracking_config_ptr->is_robust_kernel) {
                            problem.AddResidualBlock(couple_fix_sph_cost_func, loss_function, T_w_j.data());
                        } else {
                            problem.AddResidualBlock(couple_fix_sph_cost_func, nullptr, T_w_j.data());
                        }
                    }
                }
                // update data-association information for update sph coeff and visualization
                succeed_idx.push_back(i);
                // for visualization
                query_bounding_box_vec.emplace_back(initial_query_bounding_box_vec[i]);

                // for visualization
                aabb::AABB bounding_box = conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch_info_ptr->key);
                map_bounding_box_vec.emplace_back(bounding_box.lowerBound, bounding_box.upperBound);
                // for sph coeff updation
                if (!patch_info_ptr->is_ground) {
                    Eigen::Matrix<BasicType, 4, 4, Eigen::RowMajor> T_o_j =
                        ((patch_info_ptr->T_obj_lidar * patch_info_ptr->keyframe_ptr->T_w_lidar.inverse()).matrix() *
                         T_w_j)
                            .template cast<BasicType>();
                    patch_info_ptr_vec.push_back(patch_info_ptr);
                    point_cloud_obj_vec.push_back(
                        (T_o_j(Eigen::seq(0, 2), Eigen::seq(0, 2)) * point_cloud_vec[i]).colwise() +
                        T_o_j(Eigen::seq(0, 2), 3));
                }
            }
            // clear all the information
            for (auto &initial_scan_data_asso : initial_scan_data_asso_vec) {
                // this will also clear data_asso information stored in patch_info_ptr
                initial_scan_data_asso.clear();
            }

            if (debug_config_ptr->is_evaluate_time) {
                data_association_time_vec.push_back(data_asso_timer.elapsedMilliseconds());
                std::cout << "Data association time: " << data_association_time_vec.back() << " ms" << std::endl;
            }
            if (debug_config_ptr->is_evaluate_time) {
                opt_timer.start();
            }
            ceres::Solver::Options options;
            options.max_num_iterations = curl_tracking_config_ptr->max_num_iterations;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = false;
            options.update_state_every_iteration = curl_tracking_config_ptr->update_state_every_iteration;
            options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
            options.use_nonmonotonic_steps = false;
            options.num_threads = curl_tracking_config_ptr->opt_num_threads;
            if (curl_tracking_config_ptr->is_stop_after_cost_increased) {
                options.callbacks.push_back(&stop_callback);
            }
            if (curl_tracking_config_ptr->is_two_step_opt &&
                (true_ground_num > curl_tracking_config_ptr->min_num_ground_pairs) &&
                (true_seg_num > curl_tracking_config_ptr->min_num_seg_pairs)) {
                // use ground points to do optimization
                ceres::Solver::Summary summary_ground;
                problem_ground.SetManifold(T_w_j.data(), SE3_manifold_z_roll_pitch);
                ceres::Solve(options, &problem_ground, &summary_ground);
                //                double diff = abs(T_w_j(2, 3) - T_w_j_1(2, 3));
                // use non-ground points to do optimization
                ceres::Solver::Summary summary_seg;
                problem_seg.SetManifold(T_w_j.data(), SE3_manifold_x_y_yall);
                ceres::Solve(options, &problem_seg, &summary_seg);
                //                T_w_j = Sophus::SE3<double>::exp(T_w_j_tangent).matrix();
                //                diff = abs(T_w_j(2, 3) - T_w_j_1(2, 3));
            } else {
                ceres::Solver::Summary summary;
                problem.SetManifold(T_w_j.data(), SE3_manifold);
                ceres::Solve(options, &problem, &summary);
            }
            if (debug_config_ptr->is_evaluate_time) {
                opt_time_vec.push_back(opt_timer.elapsedMilliseconds());
                std::cout << "Optimization time: " << opt_time_vec.back() << " ms" << std::endl;
            }

            // after optimization its time to update the spherical harmonics coefficients
            //            tracking_timer.start();
            //            std::vector<std::shared_ptr<PatchInfo<BasicType>>> patch_info_ptr_vec;
            if (!succeed_idx.empty()) {
                update_sph_coeff_lock.lock();
                patch_info_vec_queue.push(patch_info_ptr_vec);
                point_cloud_obj_vec_queue.push(point_cloud_obj_vec);
                while (patch_info_vec_queue.size() > SH_table_config_ptr->update_sph_cpeff_queue_size &&
                       point_cloud_obj_vec_queue.size() > SH_table_config_ptr->update_sph_cpeff_queue_size &&
                       SH_table_config_ptr->update_sph_cpeff_queue_size != -1) {
                    patch_info_vec_queue.pop();
                    point_cloud_obj_vec_queue.pop();
                }
                update_sph_coeff_lock.unlock();
                update_sph_coeff_cv.notify_one();
            }
            // for visualization
            //            tracking_timer.start();
            ros::Time current_time = ros::Time::now();
            publish_lidar_tf(current_time);
            publish_trajectory(current_time);
            publish_point_cloud(current_time, point_cloud_vec, succeed_idx);
            publish_query_map_bounding_box(current_time, query_bounding_box_vec, map_bounding_box_vec);
            if (debug_config_ptr->is_save_middle_results) {
                save_middle_results(query_bounding_box_vec, map_bounding_box_vec, point_cloud_vec, point_cloud_info);
            }
            //            std::cout << "Publishing time: " << tracking_timer.elapsedMilliseconds() << " ms" <<
            //            std::endl;
            // get odometry
            T_j_1_j = (T_w_j_1.inverse() * T_w_j).matrix();
            // for debugging
            if (debug_config_ptr->is_save_final_results) {

                // save odometry and trajectory
                std::string odometry_folder =
                    debug_config_ptr->results_dir + "/" + debug_config_ptr->data_set + "_odom_" + debug_config_ptr->seq;
                curl::create_directory_if_not_exists(odometry_folder);
                FileReaderBase::write_txt_file(odometry_folder + "/" + std::to_string(frame_id) + "_T_j_1_j.txt",
                                               Eigen::MatrixXd(T_j_1_j));
                std::string trajectory_folder = debug_config_ptr->results_dir + "/" + debug_config_ptr->data_set +
                                                "_trajectory_" + debug_config_ptr->seq;
                curl::create_directory_if_not_exists(trajectory_folder);
                FileReaderBase::write_txt_file(trajectory_folder + "/" + std::to_string(frame_id) + "_T_w_j.txt",
                                               Eigen::MatrixXd(T_w_j));
                //                std::cout << "here" << std::endl;
            }
            // use constant speed model to update the next initial pose
            T_w_j_1 = T_w_j;
            // transform to local coordinate
            std::set<std::pair<int, double>, PairCompare> new_box_pair_set(new_box_map.begin(), new_box_map.end());
            if (!new_box_pair_set.empty()) {
                std::vector<Eigen::MatrixX<BasicType>> point_cloud_local_vec;
                point_cloud_local_vec.reserve(new_box_pair_set.size());
                PointCloudInfo new_point_cloud_info;
                new_point_cloud_info.time = point_cloud_info.time;
                tmp_counter = 0;
                for (auto &new_box_pair : new_box_pair_set) {
                    int i = new_box_pair.first;
                    if (point_cloud_vec[i].cols() < curl_tracking_config_ptr->minimum_points_to_add_new_patch) {
                        continue;
                    }
                    if (tmp_counter < curl_tracking_config_ptr->max_new_added_landmarks_num ||
                        curl_tracking_config_ptr->max_new_added_landmarks_num == -1) {
                        point_cloud_local_vec.push_back(
                            (point_cloud_info.T_obj_lidar_vec[i].rotation().matrix().cast<BasicType>() *
                             point_cloud_vec[i])
                                .colwise() +
                            point_cloud_info.T_obj_lidar_vec[i].translation().matrix().cast<BasicType>());
                        new_point_cloud_info.T_obj_lidar_vec.push_back(point_cloud_info.T_obj_lidar_vec[i]);
                        new_point_cloud_info.is_ground_cloud_vec.push_back(point_cloud_info.is_ground_cloud_vec[i]);
                        // Check to ensure we won't step past the end
                        ++tmp_counter;
                    } else {
                        break;
                    }
                }
                // do conformal mapping generate a keyframe
                Eigen::Isometry3d T_w_lidar;
                T_w_lidar.matrix() = T_w_j;
                add_landmark_lock.lock();
                point_cloud_local_vec_queue.push(point_cloud_local_vec);
                point_cloud_info_queue.push(new_point_cloud_info);
                T_w_lidar_queue.push(T_w_lidar);
                while (point_cloud_local_vec_queue.size() > curl_tracking_config_ptr->landmark_queue_size &&
                       point_cloud_info_queue.size() > curl_tracking_config_ptr->landmark_queue_size &&
                       curl_tracking_config_ptr->landmark_queue_size != -1) {
                    point_cloud_local_vec_queue.pop();
                    point_cloud_info_queue.pop();
                    T_w_lidar_queue.pop();
                }
                add_landmark_lock.unlock();
                if (curl_tracking_config_ptr->is_frame_to_frame) {
                    omp_set_lock(&conformal_mapping_ptr->writelock);
                    conformal_mapping_ptr->map_attributes_ptr->clear();
                    omp_unset_lock(&conformal_mapping_ptr->writelock);
                }
                add_landmark_cv.notify_one();
            }
            // TODO: 3. save corresponding points and its information into vectors
            if (!middle_map_new_box_set.empty() && debug_config_ptr->is_save_middle_map &&
                !curl_tracking_config_ptr->is_frame_to_frame) {
                std::vector<Eigen::MatrixX<BasicType>> middle_map_point_cloud_local_vec;
                middle_map_point_cloud_local_vec.reserve(middle_map_new_box_set.size());
                PointCloudInfo middle_map_new_point_cloud_info;
                middle_map_new_point_cloud_info.time = point_cloud_info.time;
                tmp_counter = 0;
                for (auto &new_box : middle_map_new_box_set) {
                    if (tmp_counter < curl_tracking_config_ptr->max_new_added_landmarks_num ||
                        curl_tracking_config_ptr->max_new_added_landmarks_num == -1) {
                        int i = new_box;
                        middle_map_point_cloud_local_vec.push_back(
                            (point_cloud_info.T_obj_lidar_vec[i].rotation().matrix().cast<BasicType>() *
                             point_cloud_vec[i])
                                .colwise() +
                            point_cloud_info.T_obj_lidar_vec[i].translation().matrix().cast<BasicType>());
                        middle_map_new_point_cloud_info.T_obj_lidar_vec.push_back(point_cloud_info.T_obj_lidar_vec[i]);
                        middle_map_new_point_cloud_info.is_ground_cloud_vec.push_back(
                            point_cloud_info.is_ground_cloud_vec[i]);
                        // Check to ensure we won't step past the end
                        ++tmp_counter;
                    } else {
                        break;
                    }
                }
                // do conformal mapping generate a keyframe
                Eigen::Isometry3d middle_map_T_w_lidar;
                middle_map_T_w_lidar.matrix() = T_w_j;
                middle_map_point_cloud_local_vec_vec.push_back(middle_map_point_cloud_local_vec);
                middle_map_point_cloud_info_vec.push_back(middle_map_new_point_cloud_info);
                middle_map_T_w_lidar_vec.push_back(middle_map_T_w_lidar);
            }
            // with until notified from the add landmark thread
            std::unique_lock<std::mutex> ul(tracking_lock);
            if (!curl_tracking_config_ptr->is_frame_to_frame) {
                tracking_cv.wait(ul, [&] { return point_cloud_info_queue.empty() && T_w_lidar_queue.empty(); });
                //                tracking_cv.wait(ul);
            } else {
                tracking_cv.wait(ul);
            }
            std::cout << "frame id: " << frame_id << std::endl;
            ++frame_id;
            if (debug_config_ptr->is_evaluate_time) {
                total_time_vec.push_back(total_timer.elapsedMilliseconds());
                std::cout << "Total time: " << total_time_vec.back() << " ms" << std::endl;
            }
        }
        rate.sleep();
    }
    std::cout << "Average patches inserted into map per scan: "
              << double(conformal_mapping_ptr->map_attributes_ptr->size()) / double(frame_id) << std::endl;
    if (debug_config_ptr->is_save_middle_map) {
        std::cout << "Saving Middle Map" << std::endl;
        save_middle_map();
        std::cout << "Done!" << std::endl;
    }
    if (debug_config_ptr->is_evaluate_time) {
        std::cout << "Saving times" << std::endl;
        std::string times_folder =
            debug_config_ptr->results_dir + "/" + debug_config_ptr->data_set + "_times_" + debug_config_ptr->seq;
        curl::create_directory_if_not_exists(times_folder);
        std::string data_association_time_file = times_folder + "/data_association_time.txt";
        FileReaderBase::write_vector_txt_file(data_association_time_file, data_association_time_vec);
        std::string landmark_initialization_time_file = times_folder + "/landmark_initialization_time.txt";
        FileReaderBase::write_vector_txt_file(landmark_initialization_time_file, landmark_initialization_time_vec);
        std::string opt_time_file = times_folder + "/opt_time.txt";
        FileReaderBase::write_vector_txt_file(opt_time_file, opt_time_vec);
        std::string update_sph_time_file = times_folder + "/update_sph_time.txt";
        FileReaderBase::write_vector_txt_file(update_sph_time_file, update_sph_time_vec);
        std::string total_time_file = times_folder + "/total_time.txt";
        FileReaderBase::write_vector_txt_file(total_time_file, total_time_vec);
        std::cout << "Done!" << std::endl;
    }
}
template <typename BasicType> void CurlTracking<BasicType>::update_sph_coeff_thread() {
    std::cout << "start update sph coeff thread" << std::endl;
    while (ros::ok()) {
        std::unique_lock<std::mutex> ul(update_sph_coeff_lock);
        update_sph_coeff_cv.wait(ul, [&] {
            return !patch_info_vec_queue.empty() && !point_cloud_obj_vec_queue.empty();
        }); // wait until queue is not empty
        ul.unlock();
        std::vector<std::shared_ptr<PatchInfo<BasicType>>> patch_info_ptr_vec = patch_info_vec_queue.front();
        std::vector<Eigen::MatrixX<BasicType>> point_cloud_obj_vec = point_cloud_obj_vec_queue.front();
        ul.lock();
        patch_info_vec_queue.pop();
        point_cloud_obj_vec_queue.pop();
        ul.unlock();
        if (debug_config_ptr->is_evaluate_time) {
            update_sph_coeff_timer.start();
        }
        for (int i = 0; i < patch_info_ptr_vec.size(); ++i) {
            bool is_recons_updated =
                patch_info_ptr_vec[i]->patch_procession_ptr->update_sph_residuals(point_cloud_obj_vec[i]);
            patch_info_ptr_vec[i]->data_asso_counter++;
            // for visualization
            if (is_recons_updated) {
                if (patch_info_ptr_vec[i]->color[0] + curl_tracking_config_ptr->color_change_step < 1) {
                    patch_info_ptr_vec[i]->color[0] += curl_tracking_config_ptr->color_change_step;
                }
                if (patch_info_ptr_vec[i]->color[1] + curl_tracking_config_ptr->color_change_step < 1) {
                    patch_info_ptr_vec[i]->color[1] += curl_tracking_config_ptr->color_change_step;
                }
                if (patch_info_ptr_vec[i]->color[2] + curl_tracking_config_ptr->color_change_step < 1) {
                    patch_info_ptr_vec[i]->color[2] += curl_tracking_config_ptr->color_change_step;
                }
                conformal_mapping_ptr->rt_publish_landmark_patch_recons(patch_info_ptr_vec[i]);
            }
        }
        if (debug_config_ptr->is_evaluate_time) {
            update_sph_time_vec.push_back(update_sph_coeff_timer.elapsedMilliseconds());
            std::cout << "Update sph coeff: " << update_sph_time_vec.back() << " ms" << std::endl;
        }
    }
}
template <typename BasicType> void CurlTracking<BasicType>::add_landmark_thread() {
    std::cout << "start add landmark thread" << std::endl;
    while (ros::ok()) {
        std::unique_lock<std::mutex> ul(add_landmark_lock);
        add_landmark_cv.wait(ul, [&] {
            return !point_cloud_local_vec_queue.empty() && !point_cloud_info_queue.empty() && !T_w_lidar_queue.empty();
        }); // wait until queue is not empty
        ul.unlock();
        std::vector<Eigen::MatrixX<BasicType>> point_cloud_local_vec = point_cloud_local_vec_queue.front();
        PointCloudInfo new_point_cloud_info = point_cloud_info_queue.front();
        Eigen::Isometry3d T_w_lidar = T_w_lidar_queue.front();
        ul.lock();
        point_cloud_local_vec_queue.pop();
        point_cloud_info_queue.pop();
        T_w_lidar_queue.pop();
        ul.unlock();
        if (debug_config_ptr->is_evaluate_time) {
            keyframe_timer.start();
        }
        conformal_mapping_ptr->rt_conformal_mapping(point_cloud_local_vec, new_point_cloud_info, T_w_lidar);
        if (debug_config_ptr->is_evaluate_time) {
            landmark_initialization_time_vec.push_back(keyframe_timer.elapsedMilliseconds());
            std::cout << "Keyframe initialization: " << landmark_initialization_time_vec.back() << " ms" << std::endl;
        }
        // notify the tracking thread
        tracking_cv.notify_one();
    }
    tracking_cv.notify_one();
};

// TODO: 4. make the return type as the pair, which can detect whether this can be added as middle map
template <typename BasicType>
std::pair<bool, bool> CurlTracking<BasicType>::new_landmark_add_judge(
    const unsigned int &key, const double &IoU, const std::vector<double> &query_lower_bound,
    const std::vector<double> &query_upper_bound, std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr,
    const Eigen::Isometry3d &T_obj_lidar, double max_new_landmark_distance) {
    // can make sure that the landmark is not too far from the lidar center
    if (T_obj_lidar.inverse().translation().norm() > max_new_landmark_distance &&
        curl_tracking_config_ptr->IQR_max_distance_of_landmarks != -100) {
        return std::make_pair(false, false);
    }
    if (IoU > curl_tracking_config_ptr->lowest_IoU_for_same_obj_different_patch) {
        // since the IoU is very high, it could be the same object in different view, so they should be different patch
        Eigen::Vector3d centroid_w = (T_w_j * T_obj_lidar.inverse()).translation();
        if ((centroid_w - patch_info_ptr->get_centroid_w()).norm() <
            curl_tracking_config_ptr->lowest_landmark_centroids_distance) {
            return std::make_pair(false, false);
        } else {
            return std::make_pair(true, true);
        }
    } else {
        std::vector<double> map_lower_bound;
        std::vector<double> map_upper_bound;
        if (!patch_info_ptr->is_ground) {
            // get the original box size
            map_lower_bound = conformal_mapping_ptr->map_attributes_ptr->get_AABB(key).lowerBound;
            map_lower_bound[0] += aabb_config_ptr->extra_skin_thickness;
            map_lower_bound[1] += aabb_config_ptr->extra_skin_thickness;
            map_lower_bound[2] += aabb_config_ptr->extra_skin_thickness;
            map_upper_bound = conformal_mapping_ptr->map_attributes_ptr->get_AABB(key).upperBound;
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
            map_lower_bound = conformal_mapping_ptr->map_attributes_ptr->get_AABB(key).lowerBound;
            map_upper_bound = conformal_mapping_ptr->map_attributes_ptr->get_AABB(key).upperBound;
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
            return std::make_pair(false, false); // if the newer box is inside of the map box
        } else {
            // calculate the IoU
            double IoU_value = curl::IoU_3d(map_lower_bound, map_upper_bound, query_lower_bound, query_upper_bound);
            // decide whether this should be initlize as a new key frame
            bool is_add_new_landmark, is_add_into_middle_map;
            if (IoU_value < curl_tracking_config_ptr->highest_IoU_new_landmark_thres) {
                is_add_new_landmark = true;
                is_add_into_middle_map = false;
            } else {
                is_add_new_landmark = false;
                if (IoU_value < debug_config_ptr->middle_map_highest_IoU_new_landmark_thres) {
                    is_add_into_middle_map = true;
                } else {
                    is_add_into_middle_map = false;
                }
            }
            return std::make_pair(is_add_new_landmark, is_add_into_middle_map);
        }
    }
}

template <typename BasicType> void CurlTracking<BasicType>::publish_trajectory(ros::Time current_time) {
    // publish the trajectory
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = T_w_j(0, 3);
    pose.pose.position.y = T_w_j(1, 3);
    pose.pose.position.z = T_w_j(2, 3);
    Eigen::Quaterniond quat = Eigen::Quaterniond(Eigen::Matrix3d(T_w_j(Eigen::seq(0, 2), Eigen::seq(0, 2))));
    pose.pose.orientation.x = quat.x(); // the x component of your quaternion
    pose.pose.orientation.y = quat.y(); // the y component of your quaternion
    pose.pose.orientation.z = quat.z(); // the z component of your quaternion
    pose.pose.orientation.w = quat.w(); // the w component of your quaternion
    trajectory.poses.push_back(pose);
    trajectory.header.stamp = current_time;
    pub_trajectory.publish(trajectory);
}

template <typename BasicType> void CurlTracking<BasicType>::publish_lidar_tf(ros::Time current_time) {
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "lidar";
    transformStamped.transform.translation.x = T_w_j(0, 3);
    transformStamped.transform.translation.y = T_w_j(1, 3);
    transformStamped.transform.translation.z = T_w_j(2, 3);
    Eigen::Quaterniond quat = Eigen::Quaterniond(Eigen::Matrix3d(T_w_j(Eigen::seq(0, 2), Eigen::seq(0, 2))));
    transformStamped.transform.rotation.x = quat.x(); // the x component of your quaternion
    transformStamped.transform.rotation.y = quat.y(); // the y component of your quaternion
    transformStamped.transform.rotation.z = quat.z(); // the z component of your quaternion
    transformStamped.transform.rotation.w = quat.w(); // the w component of your quaternion
    transformStamped.header.stamp = current_time;
    tfb.sendTransform(transformStamped);
}

template <typename BasicType>
void CurlTracking<BasicType>::publish_point_cloud(ros::Time current_time,
                                                  const std::vector<Eigen::MatrixX<BasicType>> &point_cloud_vec,
                                                  const std::vector<int> &succeed_idx) {
    pcl::PointXYZI point;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointCloud<pcl::PointXYZI> used_cloud;
    for (int i = 0; i < point_cloud_vec.size(); ++i) {
        point.intensity = static_cast<float>(i);
        if (std::find(succeed_idx.begin(), succeed_idx.end(), i) != succeed_idx.end()) {
            // i is in the vector.
            for (int j = 0; j < point_cloud_vec[i].cols(); ++j) {
                point.x = point_cloud_vec[i](0, j);
                point.y = point_cloud_vec[i](1, j);
                point.z = point_cloud_vec[i](2, j);
                cloud.push_back(point);
                used_cloud.push_back(point);
            }
        } else {
            // i is in the vector.
            for (int j = 0; j < point_cloud_vec[i].cols(); ++j) {
                point.x = point_cloud_vec[i](0, j);
                point.y = point_cloud_vec[i](1, j);
                point.z = point_cloud_vec[i](2, j);
                cloud.push_back(point);
            }
        }
    }
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "lidar";
    cloud_msg.header.stamp = current_time;
    pub_cloud.publish(cloud_msg);
    sensor_msgs::PointCloud2 used_cloud_msg;
    pcl::toROSMsg(used_cloud, used_cloud_msg);
    used_cloud_msg.header.frame_id = "lidar";
    used_cloud_msg.header.stamp = current_time;
    pub_used_cloud.publish(used_cloud_msg);
}

template <typename BasicType>
void CurlTracking<BasicType>::publish_query_map_bounding_box(
    ros::Time current_time,
    const std::vector<std::pair<std::vector<double>, std::vector<double>>> &query_bounding_box_vec,
    const std::vector<std::pair<std::vector<double>, std::vector<double>>> &map_bounding_box_vec) {
    float line_scale[3] = {0.2, 0, 0};
    std::array<float, 3> red_color{1, 0, 0};
    std::array<float, 3> green_color{0, 1, 0};
    visualization_msgs::Marker query_bounding_box = conformal_mapping_ptr->set_default_marker(
        "map", "query_bounding_box", 0, red_color, visualization_msgs::Marker::LINE_LIST, line_scale);
    visualization_msgs::Marker map_bounding_box = conformal_mapping_ptr->set_default_marker(
        "map", "map_bounding_box", 0, green_color, visualization_msgs::Marker::LINE_LIST, line_scale);
    for (auto &box_pair : query_bounding_box_vec) {
        Eigen::MatrixX<double> box_points = curl::generate_box_points<double>(box_pair.first, box_pair.second);
        for (int i = 0; i < box_points.cols(); ++i) {
            geometry_msgs::Point pt;
            pt.x = box_points(0, i);
            pt.y = box_points(1, i);
            pt.z = box_points(2, i);
            query_bounding_box.points.push_back(pt);
        }
    }
    for (auto &box_pair : map_bounding_box_vec) {
        Eigen::MatrixX<double> box_points = curl::generate_box_points<double>(box_pair.first, box_pair.second);
        for (int i = 0; i < box_points.cols(); ++i) {
            geometry_msgs::Point pt;
            pt.x = box_points(0, i);
            pt.y = box_points(1, i);
            pt.z = box_points(2, i);
            map_bounding_box.points.push_back(pt);
        }
    }
    pub_bounding_box.publish(query_bounding_box);
    pub_bounding_box.publish(map_bounding_box);
}

template <typename BasicType> void CurlTracking<BasicType>::save_middle_map() {
    // save middle maps
    std::string map_folder =
        debug_config_ptr->results_dir + "/" + debug_config_ptr->data_set + "_map_" + debug_config_ptr->seq;
    curl::create_directory_if_not_exists(map_folder);

    // TODO: 6. Save aabb tree map
    std::string aabb_map_file = map_folder + "/aabb_map.txt";
    std::ofstream aabb_map_outfile(aabb_map_file);
    aabb_map_outfile.precision(std::numeric_limits<BasicType>::max_digits10);
    int idx = 0;
    for (const auto &patch : *(conformal_mapping_ptr->map_attributes_ptr)) {
        if (patch->patch_procession_ptr->conformal_mapping.first.first.number_of_vertices() !=
            patch->patch_procession_ptr->inv_conformal_mapping.first.first.number_of_vertices()) {
            continue; // CGAL::Delaunay_triangulation_2 cloud reduce some points if they are too close
        }
        aabb_map_outfile << "# T_w_obj\n";
        Eigen::Isometry3d T_w_obj = patch->keyframe_ptr->T_w_lidar * patch->T_obj_lidar.inverse();
        for (int j = 0; j < T_w_obj.rows(); ++j) {
            aabb_map_outfile << T_w_obj(j, 0) << " " << T_w_obj(j, 1) << " " << T_w_obj(j, 2) << " " << T_w_obj(j, 3)
                             << "\n";
        }
        aabb_map_outfile << "# is ground cloud\n";
        aabb_map_outfile << (patch->is_ground ? "true" : "false") << "\n";
        if (!patch->is_ground) {
            aabb_map_outfile << "# data association times\n";
            aabb_map_outfile << patch->data_asso_counter << "\n";
        }
        aabb_map_outfile << "# lower bound and upper bound\n";
        aabb_map_outfile << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).lowerBound[0] << " "
                         << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).lowerBound[1] << " "
                         << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).lowerBound[2] << "\n";
        aabb_map_outfile << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).upperBound[0] << " "
                         << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).upperBound[1] << " "
                         << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).upperBound[2] << "\n";
        aabb_map_outfile << "# coefficients\n";
        for (const auto &sph_coeff_value : patch->patch_procession_ptr->sph_coeff) {
            aabb_map_outfile << sph_coeff_value << "\n";
        }
        aabb_map_outfile << "# XY\n";
        // initlize a matrix to store XY value, notice the order needs to be same with ->info()

        Eigen::MatrixX<BasicType> XY_matrix(
            patch->patch_procession_ptr->conformal_mapping.first.first.number_of_vertices(), 2);
        for (auto v_it = patch->patch_procession_ptr->conformal_mapping.first.first.finite_vertices_begin();
             v_it != patch->patch_procession_ptr->conformal_mapping.first.first.finite_vertices_end(); ++v_it) {
            if (v_it->info() >= patch->patch_procession_ptr->conformal_mapping.first.first.number_of_vertices()) {
                continue;
            }
            XY_matrix(static_cast<int>(v_it->info()), 0) = v_it->point().x();
            XY_matrix(static_cast<int>(v_it->info()), 1) = v_it->point().y();
            //            aabb_map_outfile << v_it->point().x() << " " << v_it->point().y() << " " << v_it->info() <<
            //            "\n";
        }
        for (int k = 0; k < XY_matrix.rows(); ++k) {
            aabb_map_outfile << XY_matrix(k, 0) << " " << XY_matrix(k, 1) << "\n";
        }
        aabb_map_outfile << "# xy\n";
        Eigen::MatrixX<BasicType> xy_matrix(
            patch->patch_procession_ptr->inv_conformal_mapping.first.first.number_of_vertices(), 2);
        for (auto v_it = patch->patch_procession_ptr->inv_conformal_mapping.first.first.finite_vertices_begin();
             v_it != patch->patch_procession_ptr->inv_conformal_mapping.first.first.finite_vertices_end(); ++v_it) {
            if (v_it->info() >= patch->patch_procession_ptr->inv_conformal_mapping.first.first.number_of_vertices()) {
                continue;
            }
            xy_matrix(static_cast<int>(v_it->info()), 0) = v_it->point().x();
            xy_matrix(static_cast<int>(v_it->info()), 1) = v_it->point().y();
            //            aabb_map_outfile << v_it->point().x() << " " << v_it->point().y() << " " << v_it->info() <<
            //            "\n";
        }
        for (int k = 0; k < xy_matrix.rows(); ++k) {
            aabb_map_outfile << xy_matrix(k, 0) << " " << xy_matrix(k, 1) << "\n";
        }
        std::cout << "Saving AABB Map Progress: "
                  << std::to_string(double(idx) / double(conformal_mapping_ptr->map_attributes_ptr->size() - 1) * 100)
                  << "%" << std::endl;
        ++idx;
    }
    aabb_map_outfile << "# Finish";

    // TODO: 5. Save middle map
    std::string middle_map_file = map_folder + "/middle_map.txt";
    std::ofstream middle_map_outfile(middle_map_file);
    middle_map_outfile.precision(std::numeric_limits<BasicType>::max_digits10);
    int middle_map_counter = 0;
    for (int i = 0; i < middle_map_T_w_lidar_vec.size(); ++i) {
        middle_map_outfile << "# T_w_lidar\n";
        for (int j = 0; j < middle_map_T_w_lidar_vec[i].rows(); ++j) {
            middle_map_outfile << middle_map_T_w_lidar_vec[i](j, 0) << " " << middle_map_T_w_lidar_vec[i](j, 1) << " "
                               << middle_map_T_w_lidar_vec[i](j, 2) << " " << middle_map_T_w_lidar_vec[i](j, 3) << "\n";
        }
        for (int j = 0; j < middle_map_point_cloud_local_vec_vec[i].size(); ++j) {
            middle_map_outfile << "# T_obj_lidar\n";
            for (int k = 0; k < middle_map_point_cloud_info_vec[i].T_obj_lidar_vec[j].rows(); ++k) {
                middle_map_outfile << middle_map_point_cloud_info_vec[i].T_obj_lidar_vec[j](k, 0) << " "
                                   << middle_map_point_cloud_info_vec[i].T_obj_lidar_vec[j](k, 1) << " "
                                   << middle_map_point_cloud_info_vec[i].T_obj_lidar_vec[j](k, 2) << " "
                                   << middle_map_point_cloud_info_vec[i].T_obj_lidar_vec[j](k, 3) << "\n";
            }
            middle_map_outfile << "# is ground cloud\n";
            middle_map_outfile << (middle_map_point_cloud_info_vec[i].is_ground_cloud_vec[j] ? "true" : "false")
                               << "\n";
            middle_map_outfile << "# local point cloud\n";
            for (int k = 0; k < middle_map_point_cloud_local_vec_vec[i][j].cols(); ++k) {
                middle_map_outfile << middle_map_point_cloud_local_vec_vec[i][j](0, k) << " "
                                   << middle_map_point_cloud_local_vec_vec[i][j](1, k) << " "
                                   << middle_map_point_cloud_local_vec_vec[i][j](2, k) << "\n";
            }
            ++middle_map_counter;
        }
        std::cout << "Saving Middle Map Progress: "
                  << std::to_string(double(i) / double(middle_map_T_w_lidar_vec.size() - 1) * 100) << "%" << std::endl;
    }
    middle_map_outfile << "# Finish";
    std::cout << "Middle map patch number: " << middle_map_counter << std::endl;
    std::cout << "AABB map patch number: " << conformal_mapping_ptr->map_attributes_ptr->size() << std::endl;
    std::cout << "Output directory: " << map_folder << std::endl;
}

template <typename BasicType>
void CurlTracking<BasicType>::save_middle_results(
    const std::vector<std::pair<std::vector<double>, std::vector<double>>> &query_bounding_box_vec,
    const std::vector<std::pair<std::vector<double>, std::vector<double>>> &map_bounding_box_vec,
    const std::vector<Eigen::MatrixX<BasicType>> &point_cloud_vec, PointCloudInfo &point_cloud_info) {
    // valid query bounding box
    std::string valid_query_bounding_box_filename =
        debug_config_ptr->middle_data_asso_dir + "/valid_query_bounding_box.txt";
    std::ofstream valid_query_bounding_box_outfile(valid_query_bounding_box_filename);
    valid_query_bounding_box_outfile.precision(std::numeric_limits<BasicType>::max_digits10);
    for (const auto &query_bounding_box : query_bounding_box_vec) {
        valid_query_bounding_box_outfile << query_bounding_box.first[0] << " " << query_bounding_box.first[1] << " "
                                         << query_bounding_box.first[2] << " " << query_bounding_box.second[0] << " "
                                         << query_bounding_box.second[1] << " " << query_bounding_box.second[2] << "\n";
    }
    // valid map bounding box
    std::string valid_map_bounding_box_filename =
        debug_config_ptr->middle_data_asso_dir + "/valid_map_bounding_box.txt";
    std::ofstream valid_map_bounding_box_outfile(valid_map_bounding_box_filename);
    for (const auto &map_bounding_box : map_bounding_box_vec) {
        valid_map_bounding_box_outfile << map_bounding_box.first[0] << " " << map_bounding_box.first[1] << " "
                                       << map_bounding_box.first[2] << " " << map_bounding_box.second[0] << " "
                                       << map_bounding_box.second[1] << " " << map_bounding_box.second[2] << "\n";
    }
    // all query patches points and bounding box
    std::string query_points_filename = debug_config_ptr->middle_data_asso_dir + "/query_points.txt";
    std::ofstream query_points_outfile(query_points_filename);
    std::string query_points_bounding_box_filename =
        debug_config_ptr->middle_data_asso_dir + "/query_points_bounding_box.txt";
    std::ofstream query_points_bounding_box_outfile(query_points_bounding_box_filename);
    std::string query_ground_points_filename = debug_config_ptr->middle_data_asso_dir + "/ground_points.txt";
    std::ofstream query_ground_points_outfile(query_ground_points_filename);
    std::string query_non_ground_points_filename = debug_config_ptr->middle_data_asso_dir + "/non_ground_points.txt";
    std::ofstream query_non_ground_points_outfile(query_non_ground_points_filename);
    for (int i = 0; i < point_cloud_vec.size(); ++i) {
        Eigen::MatrixX<BasicType> point_cloud_w =
            (T_w_j(Eigen::seq(0, 2), Eigen::seq(0, 2)).cast<BasicType>() * point_cloud_vec[i]).colwise() +
            T_w_j(Eigen::seq(0, 2), 3).cast<BasicType>();
        for (int col_idx = 0; col_idx < point_cloud_w.cols(); ++col_idx) {
            query_points_outfile << point_cloud_w(0, col_idx) << " " << point_cloud_w(1, col_idx) << " "
                                 << point_cloud_w(2, col_idx) << "\n";
        }
        query_points_bounding_box_outfile << point_cloud_w.row(0).minCoeff() << " " << point_cloud_w.row(1).minCoeff()
                                          << " " << point_cloud_w.row(2).minCoeff() << " "
                                          << point_cloud_w.row(0).maxCoeff() << " " << point_cloud_w.row(1).maxCoeff()
                                          << " " << point_cloud_w.row(2).maxCoeff() << "\n";
        if (point_cloud_info.is_ground_cloud_vec[i]) {
            for (int col_idx = 0; col_idx < point_cloud_vec[i].cols(); ++col_idx) {
                query_ground_points_outfile << point_cloud_vec[i](0, col_idx) << " " << point_cloud_vec[i](1, col_idx)
                                            << " " << point_cloud_vec[i](2, col_idx) << "\n";
            }
        } else {
            for (int col_idx = 0; col_idx < point_cloud_vec[i].cols(); ++col_idx) {
                query_non_ground_points_outfile << point_cloud_vec[i](0, col_idx) << " "
                                                << point_cloud_vec[i](1, col_idx) << " "
                                                << point_cloud_vec[i](2, col_idx) << "\n";
            }
        }
    }
    // all map patches points and bounding box
    std::set<unsigned int> key_set;
    // get all keys without repetitive first
    for (const auto &vector_0 : point_cloud_info.intersected_score_pairs_vec) {
        for (const auto &pairs : vector_0) {
            key_set.insert(pairs.first);
        }
    }
    // go through all pathes
    std::string intersected_map_points_filename =
        debug_config_ptr->middle_data_asso_dir + "/intersected_map_points.txt";
    std::string intersected_map_bounding_box_filename =
        debug_config_ptr->middle_data_asso_dir + "/intersected_map_bounding_box.txt";
    std::ofstream intersected_map_points_outfile(intersected_map_points_filename);
    std::ofstream intersected_map_bounding_box_outfile(intersected_map_bounding_box_filename);
    for (unsigned int key : key_set) {
        std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr = (*conformal_mapping_ptr->map_attributes_ptr)[key];
        Eigen::Isometry3d T_w_obj = patch_info_ptr->keyframe_ptr->T_w_lidar * patch_info_ptr->T_obj_lidar.inverse();
        patch_info_ptr->patch_procession_ptr->I_lock.lock();
        Eigen::MatrixX<BasicType> recons_v_world =
            (T_w_obj.rotation().cast<BasicType>() * patch_info_ptr->patch_procession_ptr->recons_v_local.transpose())
                .colwise() +
            T_w_obj.translation().cast<BasicType>();
        patch_info_ptr->patch_procession_ptr->I_lock.unlock();
        for (int col_idx = 0; col_idx < recons_v_world.cols(); ++col_idx) {
            intersected_map_points_outfile << recons_v_world(0, col_idx) << " " << recons_v_world(1, col_idx) << " "
                                           << recons_v_world(2, col_idx) << "\n";
        }
        intersected_map_bounding_box_outfile
            << conformal_mapping_ptr->map_attributes_ptr->get_AABB(key).lowerBound[0] << " "
            << conformal_mapping_ptr->map_attributes_ptr->get_AABB(key).lowerBound[1] << " "
            << conformal_mapping_ptr->map_attributes_ptr->get_AABB(key).lowerBound[2] << " "
            << conformal_mapping_ptr->map_attributes_ptr->get_AABB(key).upperBound[0] << " "
            << conformal_mapping_ptr->map_attributes_ptr->get_AABB(key).upperBound[1] << " "
            << conformal_mapping_ptr->map_attributes_ptr->get_AABB(key).upperBound[2] << "\n";
    }
}

template class CurlTracking<BT>; // this is very important