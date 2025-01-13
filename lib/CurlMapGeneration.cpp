//
// Created by zkc on 18/08/23.
//

#include "CurlMapGeneration.h"

// Constructors
template <typename BasicType>
CurlMapGeneration<BasicType>::CurlMapGeneration(ros::NodeHandle *nh,
                                                std::shared_ptr<MAP_GENERATION_CONFIG> _map_generation_config_ptr,
                                                std::shared_ptr<SH_TABLE_CONFIG<BT>> _SH_table_config_ptr,
                                                std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                                                std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                                                std::shared_ptr<ConformalMappingSPH<BasicType>> _conformal_mapping_ptr)
    : map_generation_config_ptr(_map_generation_config_ptr), SH_table_config_ptr(_SH_table_config_ptr),
      direct_method_config_ptr(_direct_method_config_ptr), debug_config_ptr(_debug_config_ptr),
      conformal_mapping_ptr(_conformal_mapping_ptr) {
    f = std::bind(&CurlMapGeneration<BasicType>::param_callback, this, std::placeholders::_1, std::placeholders::_2);
    server.setCallback(f);
    pub_seg_cloud = nh->advertise<sensor_msgs::PointCloud2>("/curl/seg_cloud_reconsed", 10);
    pub_ground_cloud = nh->advertise<sensor_msgs::PointCloud2>("/curl/ground_cloud_reconsed", 10);
    has_ground_info = false;
    has_bounding_box_info = false;
    has_data_asso_times = false;
    current_map_generation_config.position.x = current_map_generation_config.position.y =
        current_map_generation_config.position.y = current_map_generation_config.radius =
            std::numeric_limits<double>::max();
    current_map_generation_config.minimum_rso = std::numeric_limits<int>::max();
    open3d_display_pcd_ptr = std::make_shared<open3d::geometry::PointCloud>();
    omp_init_lock(&writelock);
    // for tracking in open3d display
    tracking_idx = 0;
    update_flag = false;
}

// member functions

template <typename BasicType> void CurlMapGeneration<BasicType>::run() {
    ros::Rate rate(10); // 10 hz
    bool is_initial_success = map_initialization();
    if (is_initial_success) {
        conformal_mapping_ptr->map_attributes_ptr->initlize_kd_tree();
        pcl::PointCloud<pcl::PointXYZI> seg_reconsed_cloud, ground_reconsed_cloud, total_local_cloud_w;
        open3d::geometry::PointCloud open3d_pcd;
        sensor_msgs::PointCloud2 seg_cloud_msg, ground_cloud_msg;
        while (ros::ok()) {
            ros::spinOnce(); // update map_generation_config_ptr first
            if (current_map_generation_config != (*map_generation_config_ptr)) {
                Eigen::Vector3<BasicType> center(static_cast<BasicType>(map_generation_config_ptr->position.x),
                                                 static_cast<BasicType>(map_generation_config_ptr->position.y),
                                                 static_cast<BasicType>(map_generation_config_ptr->position.z));
                std::vector<std::shared_ptr<PatchInfo<BasicType>>> patch_info_vec =
                    conformal_mapping_ptr->map_attributes_ptr->radius_search_patches(
                        center, static_cast<BasicType>(map_generation_config_ptr->radius));
                std::cout << "Searched patches size: " << patch_info_vec.size() << std::endl;
                std::vector<Eigen::MatrixX<BasicType>> points_recons_world_vec(patch_info_vec.size());
                double counter = 0;
#pragma omp parallel for num_threads(omp_get_num_procs()) default(none)                                                \
    shared(patch_info_vec, map_generation_config_ptr, outlier_patch_thres, points_recons_world_vec, std::cout,         \
               counter, writelock, Eigen::Dynamic, __gnu_cxx::__default_lock_policy)
                for (int i = 0; i < patch_info_vec.size(); ++i) {
                    ++counter;
                    if (map_generation_config_ptr->is_show_cleaned_map && !patch_info_vec[i]->is_ground &&
                        patch_info_vec[i]->data_asso_counter < outlier_patch_thres) {
                        continue;
                    }
                    Eigen::MatrixX<BasicType> reonsed_local_points =
                        patch_info_vec[i]->patch_procession_ptr->reconstruction(
                            map_generation_config_ptr->minimum_rso, map_generation_config_ptr->degree,
                            map_generation_config_ptr->IQR_thres, map_generation_config_ptr->method);
                    Eigen::MatrixX<BasicType> reconsed_world_points =
                        ((patch_info_vec[i]->map_gen_T_w_obj.rotation().template cast<BasicType>() *
                          reonsed_local_points.transpose())
                             .colwise() +
                         patch_info_vec[i]->map_gen_T_w_obj.translation().template cast<BasicType>())
                            .transpose();
                    omp_set_lock(&writelock);
                    points_recons_world_vec[i] = reconsed_world_points;
                    std::cout << "Reconstruction progress: " << counter / double(patch_info_vec.size()) * 100 << "%"
                              << std::endl;
                    omp_unset_lock(&writelock);
                }
                seg_reconsed_cloud.clear();
                ground_reconsed_cloud.clear();
                open3d_pcd.points_.clear();
                for (int i = 0; i < points_recons_world_vec.size(); ++i) {
                    if (points_recons_world_vec[i].rows() > 0) {
                        for (int j = 0; j < points_recons_world_vec[i].rows(); ++j) {
                            pcl::PointXYZI pt;
                            pt.x = points_recons_world_vec[i](j, 0);
                            pt.y = points_recons_world_vec[i](j, 1);
                            pt.z = points_recons_world_vec[i](j, 2);
                            pt.intensity = static_cast<float>(i);
                            if (patch_info_vec[i]->is_ground) {
                                ground_reconsed_cloud.push_back(pt);
                                open3d_pcd.points_.emplace_back((pt.getVector3fMap()).cast<double>());
                            } else {
                                seg_reconsed_cloud.push_back(pt);
                                open3d_pcd.points_.emplace_back((pt.getVector3fMap()).cast<double>());
                            }
                        }
                    }
                }
                if (map_generation_config_ptr->is_save_local_points) {
                    total_local_cloud_w = ground_reconsed_cloud + seg_reconsed_cloud;
                }
                if (map_generation_config_ptr->is_publish) {
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(seg_reconsed_cloud, centroid);
                    // Subtract the centroid from each point
                    for (auto &point : seg_reconsed_cloud) {
                        point.x -= centroid[0];
                        point.y -= centroid[1];
                        point.z -= centroid[2];
                        // point.intensity remains unchanged
                    }
                    for (auto &point : ground_reconsed_cloud) {
                        point.x -= centroid[0];
                        point.y -= centroid[1];
                        point.z -= centroid[2];
                        // point.intensity remains unchanged
                    }
                    pcl::toROSMsg(seg_reconsed_cloud, seg_cloud_msg);
                    seg_cloud_msg.header.frame_id = "map";
                    seg_cloud_msg.header.stamp = ros::Time::now();
                    pcl::toROSMsg(ground_reconsed_cloud, ground_cloud_msg);
                    ground_cloud_msg.header.frame_id = "map";
                    ground_cloud_msg.header.stamp = ros::Time::now();
                }
                if (map_generation_config_ptr->is_shown_in_open3d) {
                    if (map_generation_config_ptr->is_calculate_normals_open3d) {
                        std::cout << "Start calculating the normals for Phong lighting (Better visualisation)"
                                  << std::endl;
                        open3d_pcd.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(100), true);
                        std::cout << "Done calculating the normals for Phong lighting (Better visualisation)"
                                  << std::endl;
                    }
                    //                    update_open3d_points_lock.lock();
                    //                    std::unique_lock<std::mutex> ul(update_open3d_points_lock);
                    //                    while (!open3d_display_pcd_ptr->IsEmpty()) {
                    //                        std::cout << "Start Waiting" << std::endl;
                    //                        update_open3d_points_cv.wait(ul);
                    //                        std::cout << "End Waiting" << std::endl;
                    //                        break;
                    //                    }
                    update_open3d_points_lock.lock();
                    *open3d_display_pcd_ptr = open3d_pcd;
                    open3d_display_pcd_center = open3d_pcd.GetCenter();
                    update_flag = true;
                    current_map_generation_config = (*map_generation_config_ptr);
                    update_open3d_points_lock.unlock();
                    update_open3d_points_cv.notify_one();
                    std::cout << "Notification" << std::endl;
                } else {
                    current_map_generation_config = (*map_generation_config_ptr);
                }
                std::cout << "**************************" << std::endl;
                std::cout << "Position: " << map_generation_config_ptr->position.x << " "
                          << map_generation_config_ptr->position.y << " " << map_generation_config_ptr->position.z
                          << std::endl;
                std::cout << "w: " << map_generation_config_ptr->minimum_rso << std::endl;
                std::cout << "Degree: " << map_generation_config_ptr->degree << std::endl;
                std::cout << "Radius: " << map_generation_config_ptr->radius << std::endl;
                //                std::cout << "IQR_thres: " << map_generation_config_ptr->IQR_thres << std::endl;
                std::cout << "Method: " << map_generation_config_ptr->method << std::endl;
                std::cout << "Is start saving: " << map_generation_config_ptr->is_start_saving << std::endl;
                //                std::cout << "outlier_patch_thres: " << outlier_patch_thres << std::endl;
            }
            if (map_generation_config_ptr->is_publish) {
                pub_seg_cloud.publish(seg_cloud_msg);
                pub_ground_cloud.publish(ground_cloud_msg);
            }
            // start saving files and end the program
            if (map_generation_config_ptr->is_start_saving) {
                if (map_generation_config_ptr->is_save_local_points) {
                    std::string map_folder = debug_config_ptr->results_dir + "/" + debug_config_ptr->data_set +
                                             "_map_" + debug_config_ptr->seq;
                    curl::create_directory_if_not_exists(map_folder);
                    std::string local_points_file = map_folder + "/local_points.txt";
                    std::ofstream local_points_outfile(local_points_file);
                    local_points_outfile.precision(std::numeric_limits<BasicType>::max_digits10);
                    int idx = 0;
                    for (auto &pt : total_local_cloud_w) {
                        local_points_outfile << pt.x << " " << pt.y << " " << pt.z << "\n";
                        std::cout << "Saving local points: " << double(++idx) / double(total_local_cloud_w.size()) * 100
                                  << "%" << std::endl;
                    }
                }
                if (map_generation_config_ptr->is_save_final_map) {
                    save_final_coefficients_map();
                }
                if (map_generation_config_ptr->is_save_whole_points) {
                    save_whole_points();
                }
                current_map_generation_config.is_start_saving = false;
                ros::spinOnce();
                std::cout << "Skipped invalid patches: " << invalid_counter << std::endl;
                std::cout << "Valid patches number: "
                          << conformal_mapping_ptr->map_attributes_ptr->size() - invalid_counter << std::endl;
                invalid_counter = 0;
                if (map_generation_config_ptr->is_start_saving) {
                    return;
                }
            }
            rate.sleep();
        }
    }
}

template <typename BasicType> void CurlMapGeneration<BasicType>::open3d_display_thread() {
    std::cout << "Start open3d display thread" << std::endl;
    ros::Rate rate(10); // 10 hz
    while (ros::ok()) {
        if (map_generation_config_ptr->is_shown_in_open3d) {
            std::unique_lock<std::mutex> ul(update_open3d_points_lock);
            std::cout << "Start Waiting" << std::endl;
            while (open3d_display_pcd_ptr->IsEmpty()) {
                update_open3d_points_cv.wait(ul);
                break;
            }
            std::cout << "End Waiting" << std::endl;
            open3d_vis.CreateVisualizerWindow("Reconstruction", 1920, 1080);
            open3d_vis.AddGeometry(open3d_display_pcd_ptr);
            ul.unlock();
            while (map_generation_config_ptr->is_shown_in_open3d) {
                if (map_generation_config_ptr->open3d_display_method == "rotate") {
                    ul.lock();
                    Eigen::Matrix3d R = open3d_display_pcd_ptr->GetRotationMatrixFromXYZ(
                        Eigen::Vector3d(0, 0, M_1_PI * 2 / 1000 * map_generation_config_ptr->open3d_speed));
                    open3d_display_pcd_ptr->Rotate(R, open3d_display_pcd_center);
                    open3d_vis.UpdateGeometry();
                    open3d_vis.PollEvents();
                    open3d_vis.UpdateRender();
                    ul.unlock();
                    ros::spinOnce();
                    if (current_map_generation_config != (*map_generation_config_ptr)) {
                        update_open3d_points_cv.notify_one();
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    }
                } else if (map_generation_config_ptr->open3d_display_method == "track") {
                    ul.lock();
                    if (update_flag && !T_w_j_vec.empty()) {
                        open3d_display_pcd_ptr->Transform(T_w_j_vec[tracking_idx].inverse());
                        update_flag = false;
                    }
                    if (T_w_j_vec.empty()) { // only enter once
                        std::string trajectories_filename =
                            debug_config_ptr->results_dir + "/" + map_generation_config_ptr->trajectory_dir;
                        trajectories_num = curl::is_directory_non_empty(trajectories_filename);
                        if (trajectories_num > 0) {
                            T_w_j_vec.reserve(trajectories_num);
                            for (int idx = 0; idx < trajectories_num; ++idx) {
                                std::string trjectory_filename =
                                    trajectories_filename + "/" + std::to_string(idx) + "_T_w_j.txt";
                                T_w_j_vec.emplace_back(FileReaderBase::read_txt_file<double>(trjectory_filename));
                            }
                        } else {
                            T_w_j_vec.emplace_back(Eigen::Matrix4d::Identity());
                            trajectories_num = 1;
                        }
                        tracking_idx = 0;
                        open3d_display_pcd_ptr->Transform(T_w_j_vec.back().inverse());
                        update_flag = false;
                    }
                    // ToDo: add transformation for the point cloud
                    if (tracking_idx == 0) {
                        T_w_j_1 = T_w_j_vec.back();
                    } else {
                        T_w_j_1 = T_w_j_vec[tracking_idx - 1];
                    }
                    open3d_display_pcd_ptr->Transform(T_w_j_vec[tracking_idx].inverse() * T_w_j_1);
                    //                    tracking_idx += map_generation_config_ptr->open3d_speed;
                    ++tracking_idx;
                    if (tracking_idx >= T_w_j_vec.size()) {
                        tracking_idx = 0;
                    }
                    auto ctr = open3d_vis.GetViewControl();
                    ctr.SetLookat(Eigen::Vector3d(0, 0, 0));

                    open3d_vis.UpdateGeometry();
                    open3d_vis.PollEvents();
                    open3d_vis.UpdateRender();
                    //                    if (update_flag) {
                    //                        open3d_display_pcd_ptr->Transform(T_w_j_vec[tracking_idx].inverse());
                    //                        update_flag = false;
                    //                    }
                    ul.unlock();
                    ros::spinOnce();
                    //                    if (current_map_generation_config != (*map_generation_config_ptr)) {
                    //                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    //                    }
                }
            }
            open3d_vis.DestroyVisualizerWindow();
        }
        rate.sleep();
    }
}

template <typename BasicType>
void CurlMapGeneration<BasicType>::param_callback(curl_slam::MapGenerationConfig &config, uint32_t level) {
    map_generation_config_ptr->position.x = config.position_x;
    map_generation_config_ptr->position.y = config.position_y;
    map_generation_config_ptr->position.z = config.position_z;
    map_generation_config_ptr->minimum_rso = config.w;
    map_generation_config_ptr->degree = config.degree;
    map_generation_config_ptr->radius = config.radius;
    //    map_generation_config_ptr->IQR_thres = config.IQR_thres;
    map_generation_config_ptr->is_show_cleaned_map = config.is_show_cleaned_map;
    map_generation_config_ptr->method = config.method;
    map_generation_config_ptr->is_start_saving = config.is_start_saving;
    map_generation_config_ptr->is_save_final_map = config.is_save_final_map;
    map_generation_config_ptr->is_save_ground_info = config.is_save_ground_info;
    map_generation_config_ptr->is_save_bounding_box_info = config.is_save_bounding_box_info;
    map_generation_config_ptr->is_save_local_points = config.is_save_local_points;
    map_generation_config_ptr->is_save_whole_points = config.is_save_whole_points;
    map_generation_config_ptr->is_save_cleaned_map = config.is_save_cleaned_map;
    map_generation_config_ptr->is_publish = config.is_publish;
    map_generation_config_ptr->is_shown_in_open3d = config.is_shown_in_open3d;
    map_generation_config_ptr->open3d_display_method = config.open3d_display_method;
    map_generation_config_ptr->open3d_speed = config.open3d_speed;
    map_generation_config_ptr->is_calculate_normals_open3d = config.is_calculate_normals_open3d;
}

template <typename BasicType> bool CurlMapGeneration<BasicType>::map_initialization() {
    // load aabb map
    std::string map_folder =
        debug_config_ptr->results_dir + "/" + debug_config_ptr->data_set + "_map_" + debug_config_ptr->seq;
    if ((debug_config_ptr->is_load_aabb_map && debug_config_ptr->is_load_final_map) ||
        (debug_config_ptr->is_load_middle_map && debug_config_ptr->is_load_final_map)) {
        std::cerr << "Please correctly seting configuration file, if load final map, both aabb map and middle map can "
                     "not be loaded"
                  << std::endl;
        return false;
    }
    std::string map_file = map_folder + "/map.txt";
    if (debug_config_ptr->is_load_final_map) {
        if (std::filesystem::exists(map_file)) {
            return map_loader(map_file, false);
        } else {
            std::cout << "aabb_map.txt doesn't exist" << std::endl;
        }
    }
    std::string aabb_map_file = map_folder + "/aabb_map.txt";
    if (debug_config_ptr->is_load_aabb_map) {
        if (std::filesystem::exists(aabb_map_file)) {
            bool is_success = map_loader(aabb_map_file, true);
            if (!is_success) {
                return false;
            }
        } else {
            std::cout << "aabb_map.txt doesn't exist" << std::endl;
        }
    }
    // load middle map
    std::string middle_map_file = map_folder + "/middle_map.txt";
    if (debug_config_ptr->is_load_middle_map) {
        if (std::filesystem::exists(middle_map_file)) {
            std::string line;
            std::string current_type = " ";
            std::ifstream middle_map_in_file(middle_map_file);
            std::vector<Eigen::Isometry3d> T_w_lidar_vec;
            std::vector<std::vector<Eigen::Isometry3d>> T_obj_lidar_vec_vec;
            std::vector<Eigen::Isometry3d> T_obj_lidar_vec_tmp;
            std::vector<std::vector<bool>> is_ground_cloud_vec_vec;
            std::vector<bool> is_ground_cloud_vec_tmp;
            std::vector<std::vector<Eigen::MatrixX<BasicType>>> local_point_cloud_vec_vec;
            std::vector<Eigen::MatrixX<BasicType>> local_point_cloud_vec_tmp;
            std::vector<double> data_stream;
            int frame_id = 0;
            while (std::getline(middle_map_in_file, line)) {
                if (line[0] == '#' && current_type == " ") {
                    current_type = line.substr(2);
                } else if ((line[0] == '#' && current_type != " ")) {
                    if (current_type == "T_w_lidar") {
                        if (data_stream.size() == 12) {
                            Eigen::Isometry3d T_w_lidar = Eigen::Isometry3d::Identity();
                            int idx = 0;
                            for (int i = 0; i < 3; ++i) {
                                for (int j = 0; j < 4; ++j) {
                                    T_w_lidar(i, j) = data_stream[idx++];
                                }
                            }
                            T_w_lidar_vec.push_back(T_w_lidar);
                            if (!T_obj_lidar_vec_tmp.empty() && !is_ground_cloud_vec_tmp.empty() &&
                                !local_point_cloud_vec_tmp.empty()) {
                                T_obj_lidar_vec_vec.push_back(T_obj_lidar_vec_tmp);
                                is_ground_cloud_vec_vec.push_back(is_ground_cloud_vec_tmp);
                                local_point_cloud_vec_vec.push_back(local_point_cloud_vec_tmp);
                                T_obj_lidar_vec_tmp.clear();
                                is_ground_cloud_vec_tmp.clear();
                                local_point_cloud_vec_tmp.clear();
                            }
                            ++frame_id;
                        } else {
                            std::cerr << "T_w_lidar size is wrong!" << std::endl;
                            return false;
                        }
                    } else if (current_type == "T_obj_lidar") {
                        if (data_stream.size() == 12) {
                            Eigen::Isometry3d T_obj_lidar = Eigen::Isometry3d::Identity();
                            int idx = 0;
                            for (int i = 0; i < 3; ++i) {
                                for (int j = 0; j < 4; ++j) {
                                    T_obj_lidar(i, j) = data_stream[idx++];
                                }
                            }
                            T_obj_lidar_vec_tmp.push_back(T_obj_lidar);
                        } else {
                            std::cerr << "T_obj_lidar size is wrong!" << std::endl;
                            return false;
                        }
                    } else if (current_type == "local point cloud") {
                        if (data_stream.size() % 3 == 0) {
                            Eigen::MatrixX<BasicType> local_point_cloud =
                                (Eigen::Map<Eigen::MatrixXd>(data_stream.data(), 3,
                                                             static_cast<int>(data_stream.size()) / 3))
                                    .cast<BasicType>();
                            local_point_cloud_vec_tmp.push_back(local_point_cloud);
                        } else {
                            std::cerr << "local_point_cloud points are not three column!" << std::endl;
                            return false;
                        }
                    } else {
                        std::cerr << "No corresponding type!" << std::endl;
                        return false;
                    }
                    data_stream.clear();
                    current_type = line.substr(2);
                    if (current_type == "is ground cloud") {
                        if (std::getline(middle_map_in_file, line)) {
                            is_ground_cloud_vec_tmp.push_back(line == "true");
                            if (std::getline(middle_map_in_file, line) && line[0] == '#') {
                                current_type = line.substr(2);
                            } else {
                                std::cerr << "middle map data incomplete!" << std::endl;
                                return false;
                            }

                        } else {
                            std::cerr << "middle map data incomplete!" << std::endl;
                            return false;
                        }
                    } else if (current_type == "Finish") {
                        if (!T_obj_lidar_vec_tmp.empty() && !is_ground_cloud_vec_tmp.empty() &&
                            !local_point_cloud_vec_tmp.empty()) {
                            T_obj_lidar_vec_vec.push_back(T_obj_lidar_vec_tmp);
                            is_ground_cloud_vec_vec.push_back(is_ground_cloud_vec_tmp);
                            local_point_cloud_vec_vec.push_back(local_point_cloud_vec_tmp);
                            T_obj_lidar_vec_tmp.clear();
                            is_ground_cloud_vec_tmp.clear();
                            local_point_cloud_vec_tmp.clear();
                        }
                        break;
                    }
                } else if (current_type != " ") {
                    std::istringstream iss(line);
                    double value;
                    while (iss >> value) {
                        data_stream.push_back(value);
                    }
                } else {
                    std::cerr << "aabb_map.txt file format is wrong!" << std::endl;
                    return false;
                }
            }
            // add these patches into the map
            conformal_mapping_ptr->map_gen_conformal_mapping(T_w_lidar_vec, T_obj_lidar_vec_vec,
                                                             is_ground_cloud_vec_vec, local_point_cloud_vec_vec,
                                                             outlier_patch_thres);
        } else {
            std::cout << "middle_map.txt doesn't exist" << std::endl;
        }
    }
    if (!std::filesystem::exists(aabb_map_file) && !std::filesystem::exists(middle_map_file)) {
        std::cerr << "Both aabb_map.txt and middle_map.txt are not existed!" << std::endl;
        return false;
    }
    return true;
}

template <typename BasicType> void CurlMapGeneration<BasicType>::save_final_coefficients_map() {
    std::string map_folder =
        debug_config_ptr->results_dir + "/" + debug_config_ptr->data_set + "_map_" + debug_config_ptr->seq;
    curl::create_directory_if_not_exists(map_folder);
    std::string map_file = map_folder + "/map.txt";
    std::ofstream map_outfile(map_file);
    map_outfile.precision(std::numeric_limits<BasicType>::max_digits10);
    for (int idx = 0; idx < conformal_mapping_ptr->map_attributes_ptr->size(); ++idx) {
        auto patch = conformal_mapping_ptr->map_attributes_ptr->at(idx);
        if (map_generation_config_ptr->is_save_cleaned_map && !patch->is_ground &&
            patch->data_asso_counter < outlier_patch_thres) {
            ++invalid_counter;
            continue;
        }
        if (patch->patch_procession_ptr->conformal_mapping.first.first.number_of_vertices() !=
            patch->patch_procession_ptr->inv_conformal_mapping.first.first.number_of_vertices()) {
            ++invalid_counter;
            continue; // CGAL::Delaunay_triangulation_2 cloud reduce some points if they are too close
        }
        map_outfile << "# T_w_obj\n";
        Eigen::Isometry3d T_w_obj = patch->map_gen_T_w_obj;
        for (int j = 0; j < T_w_obj.rows(); ++j) {
            map_outfile << T_w_obj(j, 0) << " " << T_w_obj(j, 1) << " " << T_w_obj(j, 2) << " " << T_w_obj(j, 3)
                        << "\n";
        }
        if (map_generation_config_ptr->is_save_ground_info && has_ground_info) {
            map_outfile << "# is ground cloud\n";
            map_outfile << (patch->is_ground ? "true" : "false") << "\n";
        }
        if (map_generation_config_ptr->is_save_bounding_box_info && has_bounding_box_info) {
            map_outfile << "# lower bound and upper bound\n";
            map_outfile << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).lowerBound[0] << " "
                        << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).lowerBound[1] << " "
                        << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).lowerBound[2] << "\n";
            map_outfile << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).upperBound[0] << " "
                        << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).upperBound[1] << " "
                        << conformal_mapping_ptr->map_attributes_ptr->get_AABB(patch->key).upperBound[2] << "\n";
        }
        map_outfile << "# coefficients\n";
        // final coefficients number is decieded by dynamic parameter
        int num_of_sph_coeff;
        if (map_generation_config_ptr->degree <= patch->patch_procession_ptr->SH_degree) {
            num_of_sph_coeff = std::pow(map_generation_config_ptr->degree + 1, 2);
        } else {
            num_of_sph_coeff = std::pow(patch->patch_procession_ptr->SH_degree + 1, 2);
        }

        for (int i = 0; i < num_of_sph_coeff; ++i) {
            map_outfile << patch->patch_procession_ptr->sph_coeff(i) << "\n";
        }
        map_outfile << "# XY\n";
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
        }
        for (int k = 0; k < XY_matrix.rows(); ++k) {
            map_outfile << XY_matrix(k, 0) << " " << XY_matrix(k, 1) << "\n";
        }
        map_outfile << "# xy\n";
        Eigen::MatrixX<BasicType> xy_matrix(
            patch->patch_procession_ptr->inv_conformal_mapping.first.first.number_of_vertices(), 2);
        for (auto v_it = patch->patch_procession_ptr->inv_conformal_mapping.first.first.finite_vertices_begin();
             v_it != patch->patch_procession_ptr->inv_conformal_mapping.first.first.finite_vertices_end(); ++v_it) {
            if (v_it->info() >= patch->patch_procession_ptr->inv_conformal_mapping.first.first.number_of_vertices()) {
                continue;
            }
            xy_matrix(static_cast<int>(v_it->info()), 0) = v_it->point().x();
            xy_matrix(static_cast<int>(v_it->info()), 1) = v_it->point().y();
        }
        for (int k = 0; k < xy_matrix.rows(); ++k) {
            map_outfile << xy_matrix(k, 0) << " " << xy_matrix(k, 1) << "\n";
        }
        std::cout << "Saving final coefficients map: "
                  << std::to_string(double(idx) / double(conformal_mapping_ptr->map_attributes_ptr->size() - 1) * 100)
                  << "%" << std::endl;
    }
    map_outfile << "# Finish";
}

template <typename BasicType>
bool CurlMapGeneration<BasicType>::map_loader(const std::string &filename, bool is_aabb_map) {
    std::string line;
    std::string current_type = " ";
    std::ifstream map_in_file(filename);
    std::vector<Eigen::Isometry3d> T_w_obj_vec;
    std::vector<bool> is_ground_cloud_vec;
    std::vector<BasicType> data_asso_times_vec;
    std::vector<aabb::AABB> box_boundary_vec;
    std::vector<Eigen::VectorX<BasicType>> sph_coeff_vec;
    std::vector<Eigen::MatrixX<BasicType>> XY_vec;
    std::vector<Eigen::MatrixX<BasicType>> xy_vec;
    std::vector<double> data_stream;
    while (std::getline(map_in_file, line)) {
        if (line[0] == '#' && current_type == " ") {
            current_type = line.substr(2);
        } else if ((line[0] == '#' && current_type != " ")) {
            // Based on the currentType, store the data in the appropriate vector
            if (current_type == "T_w_obj") {
                if (data_stream.size() == 12) {
                    Eigen::Isometry3d T_w_obj = Eigen::Isometry3d::Identity();
                    int idx = 0;
                    for (int i = 0; i < 3; ++i) {
                        for (int j = 0; j < 4; ++j) {
                            T_w_obj(i, j) = data_stream[idx++];
                        }
                    }
                    T_w_obj_vec.push_back(T_w_obj);
                } else {
                    std::cerr << "T_w_obj size is wrong!" << std::endl;
                    return false;
                }
            } else if (current_type == "lower bound and upper bound") {
                has_bounding_box_info = true;
                if (data_stream.size() == 6) {
                    aabb::AABB box_boundary(3);
                    for (int i = 0; i < 6; ++i) {
                        if (i < 3) {
                            box_boundary.lowerBound[i] = data_stream[i];
                        } else {
                            box_boundary.upperBound[i - 3] = data_stream[i];
                        }
                    }
                    box_boundary_vec.push_back(box_boundary);
                } else {
                    std::cerr << "lower and upper bound number is wrong!" << std::endl;
                    return false;
                }
            } else if (current_type == "coefficients") {
                Eigen::VectorX<BasicType> sph_coeff =
                    Eigen::Map<Eigen::VectorXd>(data_stream.data(), static_cast<int>(data_stream.size()))
                        .cast<BasicType>();
                sph_coeff_vec.push_back(sph_coeff);
            } else if (current_type == "XY") {
                if (data_stream.size() % 2 == 0) {
                    Eigen::MatrixX<BasicType> XY =
                        (Eigen::Map<Eigen::MatrixXd>(data_stream.data(), 2, static_cast<int>(data_stream.size()) / 2))
                            .cast<BasicType>();
                    XY_vec.push_back(XY);
                } else {
                    std::cerr << "XY points number is not even!" << std::endl;
                    return false;
                }
            } else if (current_type == "xy") {
                if (data_stream.size() % 2 == 0) {
                    Eigen::MatrixX<BasicType> xy =
                        (Eigen::Map<Eigen::MatrixXd>(data_stream.data(), 2, static_cast<int>(data_stream.size()) / 2))
                            .cast<BasicType>();
                    xy_vec.push_back(xy);
                } else {
                    std::cerr << "XY points number is not even!" << std::endl;
                    return false;
                }
            } else if (current_type == "data association times") {
                has_data_asso_times = true;
                if (data_stream.size() == 1) {
                    data_asso_times_vec.push_back(static_cast<BasicType>(data_stream[0]));
                } else {
                    std::cerr << "data association times is not one number!" << std::endl;
                    return false;
                }
            } else {
                std::cerr << "No corresponding type!" << std::endl;
                return false;
            }
            data_stream.clear();
            current_type = line.substr(2);
            if (current_type == "is ground cloud") {
                has_ground_info = true;
                if (std::getline(map_in_file, line)) {
                    is_ground_cloud_vec.push_back(line == "true");
                    if (std::getline(map_in_file, line) && line[0] == '#') {
                        current_type = line.substr(2);
                    } else {
                        std::cerr << "coefficient map data incomplete!" << std::endl;
                        return false;
                    }
                } else {
                    std::cerr << "coefficient map data incomplete!" << std::endl;
                    return false;
                }
            } else if (current_type == "Finish") {
                break;
            }
        } else if (current_type != " ") {
            std::istringstream iss(line);
            double value;
            while (iss >> value) {
                data_stream.push_back(value);
            }
        } else {
            std::cerr << "aabb_map.txt or map.txt file format is wrong!" << std::endl;
            return false;
        }
    }
    if (!has_bounding_box_info && is_aabb_map) {
        std::cerr << "aabb map has to have bounding box info" << std::endl;
        return false;
    }
    // add above patches into map
    std::vector<std::array<float, 3>> colors = {
        //            {1.0f, 0.0f, 0.0f}, // Red
        //            {0.0f, 1.0f, 0.0f}, // Green
        {0.0f, 0.0f, 1.0f} // Blue
                           //            {1.0f, 1.0f, 0.0f}, // Yellow
                           //            {0.0f, 1.0f, 1.0f}  // Cyan
    };
    if (has_data_asso_times && map_generation_config_ptr->outlier_patch_IQR_thres != -100) {
        //        outlier_patch_thres = curl::IQR_thres<BasicType>(
        //            data_asso_times_vec, static_cast<BasicType>(map_generation_config_ptr->outlier_patch_IQR_thres));
        outlier_patch_thres = map_generation_config_ptr->outlier_patch_IQR_thres;
    } else {
        outlier_patch_thres = 0;
    }

    if (!T_w_obj_vec.empty() && !sph_coeff_vec.empty() && !XY_vec.empty() && !xy_vec.empty()) {
        int seg_idx = 0;
        for (int i = 0; i < T_w_obj_vec.size(); ++i) {
            interp_func_pair F0 = linear_interpolation_2_func(XY_vec[i].transpose(), xy_vec[i].row(0).transpose());
            interp_func_pair F1 = linear_interpolation_2_func(XY_vec[i].transpose(), xy_vec[i].row(1).transpose());
            // inverse conformal mapping
            interp_func_pair B0 = linear_interpolation_2_func(xy_vec[i].transpose(), XY_vec[i].row(0).transpose());
            interp_func_pair B1 = linear_interpolation_2_func(xy_vec[i].transpose(), XY_vec[i].row(1).transpose());
            int current_degree = sqrt(double(sph_coeff_vec[i].size())) - 1;
            std::shared_ptr<PatchProcession<BasicType>> patch_procession_ptr =
                std::make_shared<PatchProcession<BasicType>>(direct_method_config_ptr, SH_table_config_ptr,
                                                             debug_config_ptr, std::make_pair(F0, F1),
                                                             std::make_pair(B0, B1), sph_coeff_vec[i], current_degree);
            if (has_bounding_box_info && has_ground_info) {
                std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr = std::make_shared<PatchInfo<BasicType>>(
                    T_w_obj_vec[i], patch_procession_ptr, is_ground_cloud_vec[i], colors[i % colors.size()]);
                if (!is_ground_cloud_vec[i]) {
                    if (has_data_asso_times) {
                        patch_info_ptr->data_asso_counter = static_cast<unsigned int>(data_asso_times_vec[seg_idx++]);
                    }
                    conformal_mapping_ptr->map_attributes_ptr->map_gen_insert(
                        patch_info_ptr, box_boundary_vec[i].lowerBound, box_boundary_vec[i].upperBound);
                } else {
                    conformal_mapping_ptr->map_attributes_ptr->map_gen_insert_ground(
                        patch_info_ptr, box_boundary_vec[i].lowerBound, box_boundary_vec[i].upperBound);
                }
            }
            if (has_bounding_box_info && !has_ground_info) {
                std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr = std::make_shared<PatchInfo<BasicType>>(
                    T_w_obj_vec[i], patch_procession_ptr, colors[i % colors.size()]);
                if (has_data_asso_times) {
                    patch_info_ptr->data_asso_counter = static_cast<unsigned int>(data_asso_times_vec[seg_idx++]);
                }
                conformal_mapping_ptr->map_attributes_ptr->map_gen_insert(
                    patch_info_ptr, box_boundary_vec[i].lowerBound, box_boundary_vec[i].upperBound);
            }
            if (!has_bounding_box_info && has_ground_info) {
                std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr = std::make_shared<PatchInfo<BasicType>>(
                    T_w_obj_vec[i], patch_procession_ptr, is_ground_cloud_vec[i], colors[i % colors.size()]);
                if (!is_ground_cloud_vec[i]) {
                    if (has_data_asso_times) {
                        patch_info_ptr->data_asso_counter = static_cast<unsigned int>(data_asso_times_vec[seg_idx++]);
                    }
                    conformal_mapping_ptr->map_attributes_ptr->map_gen_insert(patch_info_ptr);
                } else {
                    conformal_mapping_ptr->map_attributes_ptr->map_gen_insert_ground(patch_info_ptr);
                }
            }
            if (!has_bounding_box_info && !has_ground_info) {
                std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr = std::make_shared<PatchInfo<BasicType>>(
                    T_w_obj_vec[i], patch_procession_ptr, colors[i % colors.size()]);
                if (has_data_asso_times) {
                    patch_info_ptr->data_asso_counter = static_cast<unsigned int>(data_asso_times_vec[seg_idx++]);
                }
                conformal_mapping_ptr->map_attributes_ptr->map_gen_insert(patch_info_ptr);
            }
            std::cout << "Loading coefficients map: "
                      << std::to_string(double(i) / double(T_w_obj_vec.size() - 1) * 100) << "%" << std::endl;
        }
    } else {
        std::cerr << "Data corruption!!!" << std::endl;
        return false;
    }
    return true;
}

template <typename BasicType> void CurlMapGeneration<BasicType>::save_whole_points() {
    std::vector<Eigen::MatrixX<BasicType>> points_recons_world_vec;
    int idx = 0;
    for (const auto &patch_info_ptr : (*conformal_mapping_ptr->map_attributes_ptr)) {
        if (map_generation_config_ptr->is_save_cleaned_map && !patch_info_ptr->is_ground &&
            patch_info_ptr->data_asso_counter < outlier_patch_thres) {
            continue;
        }
        Eigen::MatrixX<BasicType> reonsed_local_points = patch_info_ptr->patch_procession_ptr->reconstruction(
            map_generation_config_ptr->minimum_rso, map_generation_config_ptr->degree,
            map_generation_config_ptr->IQR_thres, map_generation_config_ptr->method);
        Eigen::MatrixX<BasicType> reconsed_world_points =
            ((patch_info_ptr->map_gen_T_w_obj.rotation().template cast<BasicType>() * reonsed_local_points.transpose())
                 .colwise() +
             patch_info_ptr->map_gen_T_w_obj.translation().template cast<BasicType>())
                .transpose();
        points_recons_world_vec.push_back(reconsed_world_points);
        std::cout << "Whole map reconstruction progress: "
                  << double(++idx) / double(conformal_mapping_ptr->map_attributes_ptr->size()) * 100 << "%"
                  << std::endl;
    }

    std::string map_folder =
        debug_config_ptr->results_dir + "/" + debug_config_ptr->data_set + "_map_" + debug_config_ptr->seq;
    curl::create_directory_if_not_exists(map_folder);
    std::string map_points_file = map_folder + "/whole_map_points.txt";
    std::ofstream map_points_outfile(map_points_file);
    map_points_outfile.precision(std::numeric_limits<BasicType>::max_digits10);
    std::cout << "Start saving map points file" << std::endl;
    for (const auto &matrix : points_recons_world_vec) {
        for (int i = 0; i < matrix.rows(); ++i) {
            map_points_outfile << matrix(i, 0) << " " << matrix(i, 1) << " " << matrix(i, 2) << "\n";
        }
    }
    std::cout << "Done" << std::endl;
}

template class CurlMapGeneration<BT>; // this is very important