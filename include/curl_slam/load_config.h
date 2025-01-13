/**
 * @file load_config.h
 * @author zkc (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-11
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef EXAMPLE_LOAD_CONFIG
#define EXAMPLE_LOAD_CONFIG
#include "Eigen/Core"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <string>

struct FILE_READER_CONFIG {
    std::string data_set;
    std::string root_dir;
    std::string sequence;
    std::string scans_dir;
    double scan_scale_factor;
    std::string poses_dir;
    std::string semantics_dir;
    int start_frame;
    int end_frame;
    std::string scan_file_type;
    std::string semantic_file_type;
    std::string poses_file_type;
    bool is_debug = false;
    std::string transformation_obj_LiDAR_dir;
    std::string conformal_map_dir;
    std::string gamma_gradient_dir;
    std::string valid_idx_dir;
    std::string sph_coeff_dir;
    std::string odometry_file;
    std::string odometry_noised_file;
    std::string trajectory_file;
    std::string trajectory_noised_file;
    int obj_num = 0;
    double gamma_rso;
};

struct PRE_PROCESSION_CONFIG {
    double length_percentage;
    double min_length;
    double filter_mean;
    double filter_std;
    double max_length;
};

struct VSA_CONFIG {
    bool is_selected;
    std::string seeding_method;
    int max_number_of_proxies = 0;
    int number_of_iterations = 0;
    double min_error_drop = 0;
};

struct PCL_NORMAL_CONFIG {
    int search_region;
};

struct REGION_GROWING_CONFIG {
    bool is_selected;
    bool is_igl;
    int min_cluster_size;
    int max_cluster_size;
    int number_of_neighbours;
    double smoothness_threshold;
    double curvature_threshold;
};

struct CURVATURE_CONFIG {
    bool gaussian_curvature_is_selected;
    bool mean_curvature_is_selected;
    std::string area_method;
};

struct PROBABILITY_CLUSTERING_CONFIG {
    bool is_merge;
    std::string is_average_face_normal_and_curvature;
    int average_window_size;
    int initial_window_size;
    double th_normal_angle;
    double th_diff_normal_norm_mean;
    double th_diff_normal_variance;
    double th_K_L_J_S_threshold;
    double th_diff_G_curvature_mean;
    double th_diff_G_curvature_variance;
    double th_diff_M_curvature_mean;
    double th_diff_M_curvature_variance;
    bool is_normal_selected;
    double diff_normal_norm_mean;
    double diff_normal_variance;
    bool is_length_selected;
    double diff_length_mean;
    double diff_length_variance;
    bool is_G_curvature_selected;
    std::string comparison_metric;
    double K_L_J_S_threshold;
    double diff_G_curvature_mean;
    double diff_G_curvature_variance;
    bool is_M_curvature_selected;
    double diff_M_curvature_mean;
    double diff_M_curvature_variance;
    int region_num;
};

struct LOCAL_BA_CONFIG {
    bool is_set_constant;
    bool is_couple;
    bool is_couple_sph;
    bool is_couple_sph_mod;
    bool is_couple_switch;
    bool is_odometry;
    bool is_object_to_map;
    bool is_object_fix_to_map;
    bool is_object_to_object;
    bool is_object_point_to_object_point;
    bool is_sph_conformal_fix_point_to_point;
    bool is_sph_conformal_fix_point_to_map;
    bool is_sph_conformal_point_to_map;
    bool is_sph_fix_point_to_map;
    bool is_nn_data_association;
    bool is_pure_ICP;
    bool is_direct_method;
    bool is_obj_fix;
    double weight;
    int max_num_iterations;
    bool update_state_every_iteration;
    int iter_num;
    double threshold;
    bool is_debug;
    bool is_ground_truth_initial;
    std::string debug_2D_projection_dir;
};

struct DIRECT_METHOD_CONFIG {
    std::string method;
    std::string valid_region_method;
    int minimum_img_rso;
    double gamma_sampling_interval;
    int pyramid_minimum_img_rso;
    bool is_pyramid_track;
    bool is_display;
    bool is_display_block;
    int max_iters;
    std::string update_points_method;
    bool is_update_sph_coeff;
    double update_sph_coeff_threshold;
    bool is_gaussian_blur;
    double sigma;
    std::string loss_function;
    double IQR_delta;
    double linear_loss_param;
    double trapeziform_lower_bound;
    double trapeziform_higher_bound;
};

template <typename T> struct SH_TABLE_CONFIG {
    bool is_speed_up;
    int max_SH_degree;
    int warping_SH_degree;
    int ground_SH_degree;
    T SH_scale;
    int azi_rso;
    int elev_rso;
    int init_SPH_pass;
    std::string inv_method;
    T bad_thres;
    T medium_thres;
    int update_sph_counter_thres;
    int update_sph_cpeff_queue_size;

    int update_azi_rso;
    int update_elev_rso;
    T sampling_factor;

    Eigen::MatrixX<T> SH_table;
    Eigen::MatrixX<T> SH_G_theta_table;
    Eigen::MatrixX<T> SH_G_phi_table;
    Eigen::VectorXi update_idx;

  public:
    T get_azi_low() const { return (M_PI * (1 - SH_scale)); }
    T get_azi_high() const { return (2 * M_PI * SH_scale + get_azi_low()); }
    T get_elev_low() const { return (M_PI / 2 * (1 - SH_scale)); }
    T get_elev_high() const { return (M_PI * SH_scale + get_elev_low()); }
};

struct CONFORMAL_MAPPING_CONFIG {
    bool is_rt;
    bool is_icp;
    bool is_points_distance;
    double cut_threshold;
    double ground_cut_threshold;
    bool is_voxel_grid_filter;
    double leaf_size;
    bool is_StatisticalOutlierRemoval;
    int outlier_removing_neighbour_num;
    double outlier_removing_std;
    bool is_RadiusOutlierRemoval;
    float outlier_removing_radius;
    int outlier_removing_min_nei;
    int patch_minimum_pts_num;
    int patch_minimun_pts_num_after_filter;
    std::string defect_triangulation_type;
    double tri_angle_min;
    double tri_angle_max;
    double tri_area_max;
    double tri_length_max;
    bool is_IQR;
    double tri_area_max_scale_factor;
    double tri_length_max_scale_factor;
    bool is_hist;
    bool is_hist_fall;
    double tri_area_percentage;
    double tri_length_percentage;
    double rectangular_corner_weight;
    int minimum_triangles_after_filter;
    int height_opt_iters;
    int thread_num;
    bool is_even_segmentation;
    int map_window_size;
};

struct DEBUG_CONFIG {
    std::string results_dir;
    std::string data_set;
    std::string seq;
    std::string middle_results_dir;
    std::string middle_patch_dir;
    std::string middle_data_asso_dir;
    bool is_save_middle_results;
    unsigned int patch_id;
    bool is_save_final_results;
    bool is_evaluate_time;
    bool is_save_middle_map;
    double middle_map_highest_IoU_new_landmark_thres;
    bool is_load_middle_map;
    bool is_load_aabb_map;
    bool is_load_final_map;
};

struct AABB_CONFIG {
    double skin_thickness;
    double extra_skin_thickness;
    double ground_fatten_scale;
    bool touch_is_overlap;
};

struct CURL_TRACKING_CONFIG {
    double frequency;
    bool is_frame_to_frame;
    double IoU_first_level;
    double IoU_second_level;
    double centroids_distance;
    double ground_IoU_first_level;
    double valid_ground_radius;
    double weight_of_IoU;
    double largest_centroid_distance;
    int largest_tree_key_difference;
    double IQR_scale_factor;
    bool is_select_the_best_pair_only;
    bool is_use_score_as_weight;
    int max_seg_pairs;
    int max_ground_pairs;
    int max_query_paired_patches;
    int minimum_points_to_add_new_patch;
    double highest_IoU_new_landmark_thres;
    double lowest_IoU_for_same_obj_different_patch;
    double lowest_landmark_centroids_distance;
    int landmark_queue_size;
    int max_new_added_landmarks_num;
    double IQR_max_distance_of_landmarks;
    bool is_two_step_opt;
    int min_num_ground_pairs;
    int min_num_seg_pairs;
    int max_num_iterations;
    bool update_state_every_iteration;
    bool is_robust_kernel;
    double kernel_threshold;
    double max_valid_residual;
    int opt_num_threads;
    int ground_opt_step;
    bool is_stop_after_cost_increased;
    float color_change_step;
};

struct MAP_GENERATION_CONFIG {
    struct POSITION {
        double x;
        double y;
        double z;

        bool operator==(const POSITION &other) const {
            // Use some threshold for floating point comparison if needed
            constexpr double EPSILON = 1e-9;
            return (std::abs(x - other.x) < EPSILON) && (std::abs(y - other.y) < EPSILON) &&
                   (std::abs(z - other.z) < EPSILON);
        }
    };
    POSITION position;
    int minimum_rso;
    int degree;
    double radius;
    double IQR_thres = 0;
    double outlier_patch_IQR_thres;
    std::string method;
    bool is_shown_in_open3d; // won't cause reconstruction (shouldn't add this into == operator)
    std::string open3d_display_method;
    bool is_calculate_normals_open3d;
    int open3d_speed; // won't cause reconstruction (shouldn't add this into == operator)
    bool is_show_cleaned_map;
    bool is_publish;
    bool is_start_saving;
    // saving options; these don't need in equality comparsion
    bool is_save_final_map;
    bool is_save_cleaned_map;
    bool is_save_ground_info;
    bool is_save_bounding_box_info;
    bool is_save_local_points;
    bool is_save_whole_points;
    std::string trajectory_dir;

    bool operator==(const MAP_GENERATION_CONFIG &other) const {
        // Use some threshold for floating point comparison if needed
        constexpr double EPSILON = 1e-9;
        return position == other.position && minimum_rso == other.minimum_rso && degree == other.degree &&
               IQR_thres == other.IQR_thres && method == other.method && (std::abs(radius - other.radius) < EPSILON) &&
               is_start_saving == other.is_start_saving && is_show_cleaned_map == other.is_show_cleaned_map &&
               is_publish == other.is_publish && is_calculate_normals_open3d == other.is_calculate_normals_open3d &&
               open3d_display_method == other.open3d_display_method;
    }

    bool operator!=(const MAP_GENERATION_CONFIG &other) const { return !((*this) == other); }
};

void load_config_file(const std::string &dir, FILE_READER_CONFIG &file_reader_config) {
    YAML::Node config = YAML::LoadFile(dir);
    file_reader_config.data_set = config["DATA_SET"].as<decltype(file_reader_config.data_set)>();
    file_reader_config.root_dir =
        config[file_reader_config.data_set]["root_dir"].as<decltype(file_reader_config.root_dir)>();
    file_reader_config.sequence =
        config[file_reader_config.data_set]["sequence"].as<decltype(file_reader_config.sequence)>();
    file_reader_config.scans_dir =
        config[file_reader_config.data_set]["scans_dir"].as<decltype(file_reader_config.scans_dir)>();
    file_reader_config.scan_scale_factor =
        config[file_reader_config.data_set]["scan_scale_factor"].as<decltype(file_reader_config.scan_scale_factor)>();
    if (config[file_reader_config.data_set]["poses_dir"]) {
        file_reader_config.poses_dir =
            config[file_reader_config.data_set]["poses_dir"].as<decltype(file_reader_config.poses_dir)>();
    }

    file_reader_config.semantics_dir =
        config[file_reader_config.data_set]["semantics_dir"].as<decltype(file_reader_config.semantics_dir)>();
    file_reader_config.start_frame =
        config[file_reader_config.data_set]["start_frame"].as<decltype(file_reader_config.start_frame)>();
    file_reader_config.end_frame =
        config[file_reader_config.data_set]["end_frame"].as<decltype(file_reader_config.end_frame)>();
    file_reader_config.scan_file_type =
        config[file_reader_config.data_set]["scan_file_type"].as<decltype(file_reader_config.scan_file_type)>();
    file_reader_config.semantic_file_type =
        config[file_reader_config.data_set]["semantic_file_type"].as<decltype(file_reader_config.semantic_file_type)>();
    if (config[file_reader_config.data_set]["poses_file_type"]) {
        file_reader_config.poses_file_type =
            config[file_reader_config.data_set]["poses_file_type"].as<decltype(file_reader_config.poses_file_type)>();
    }
    if (config[file_reader_config.data_set]["is_debug"]) {
        file_reader_config.is_debug =
            config[file_reader_config.data_set]["is_debug"].as<decltype(file_reader_config.is_debug)>();
    }
    if (config[file_reader_config.data_set]["transformation_obj_LiDAR_dir"]) {
        file_reader_config.transformation_obj_LiDAR_dir =
            config[file_reader_config.data_set]["transformation_obj_LiDAR_dir"]
                .as<decltype(file_reader_config.transformation_obj_LiDAR_dir)>();
    }
    if (config[file_reader_config.data_set]["conformal_map_dir"]) {
        file_reader_config.conformal_map_dir = config[file_reader_config.data_set]["conformal_map_dir"]
                                                   .as<decltype(file_reader_config.conformal_map_dir)>();
    }
    if (config[file_reader_config.data_set]["gamma_gradient_dir"]) {
        file_reader_config.gamma_gradient_dir = config[file_reader_config.data_set]["gamma_gradient_dir"]
                                                    .as<decltype(file_reader_config.gamma_gradient_dir)>();
    }
    if (config[file_reader_config.data_set]["valid_idx_dir"]) { // valid_idx_dir
        file_reader_config.valid_idx_dir =
            config[file_reader_config.data_set]["valid_idx_dir"].as<decltype(file_reader_config.valid_idx_dir)>();
    }
    if (config[file_reader_config.data_set]["sph_coeff_dir"]) {
        file_reader_config.sph_coeff_dir =
            config[file_reader_config.data_set]["sph_coeff_dir"].as<decltype(file_reader_config.sph_coeff_dir)>();
    }
    if (config[file_reader_config.data_set]["odometry_file"]) {
        file_reader_config.odometry_file =
            config[file_reader_config.data_set]["odometry_file"].as<decltype(file_reader_config.odometry_file)>();
    }
    if (config[file_reader_config.data_set]["odometry_noised_file"]) {
        file_reader_config.odometry_noised_file = config[file_reader_config.data_set]["odometry_noised_file"]
                                                      .as<decltype(file_reader_config.odometry_noised_file)>();
    }
    if (config[file_reader_config.data_set]["trajectory_file"]) {
        file_reader_config.trajectory_file =
            config[file_reader_config.data_set]["trajectory_file"].as<decltype(file_reader_config.trajectory_file)>();
    }
    if (config[file_reader_config.data_set]["trajectory_noised_file"]) {
        file_reader_config.trajectory_noised_file = config[file_reader_config.data_set]["trajectory_noised_file"]
                                                        .as<decltype(file_reader_config.trajectory_noised_file)>();
    }
    if (config[file_reader_config.data_set]["obj_num"]) {
        file_reader_config.obj_num =
            config[file_reader_config.data_set]["obj_num"].as<decltype(file_reader_config.obj_num)>();
    }
    if (config[file_reader_config.data_set]["gamma_rso"]) {
        file_reader_config.gamma_rso =
            config[file_reader_config.data_set]["gamma_rso"].as<decltype(file_reader_config.gamma_rso)>();
    }
}

bool load_preprocessing_config(const std::string &dir, PRE_PROCESSION_CONFIG &pre_precossing_config) {
    YAML::Node config = YAML::LoadFile(dir);
    if (config["DATA_SET"].as<std::string>() == "KITTI") {
        pre_precossing_config.length_percentage =
            config["KITTI_PREPROCESSING"]["length_percentage"].as<decltype(pre_precossing_config.length_percentage)>();
        pre_precossing_config.min_length =
            config["KITTI_PREPROCESSING"]["min_length"].as<decltype(pre_precossing_config.min_length)>();
        pre_precossing_config.filter_mean =
            config["KITTI_PREPROCESSING"]["filter_mean"].as<decltype(pre_precossing_config.filter_mean)>();
        pre_precossing_config.filter_std =
            config["KITTI_PREPROCESSING"]["filter_std"].as<decltype(pre_precossing_config.filter_std)>();
        return true;
    } else {
        std::cout << "No corresponding preprocessing configure file exist!" << std::endl;
        return false;
    }
}

void load_vsa_config(const std::string &dir, VSA_CONFIG &vsa_config) {
    YAML::Node config = YAML::LoadFile(dir);
    vsa_config.is_selected = config["VSA"]["is_selected"].as<decltype(vsa_config.is_selected)>();
    vsa_config.seeding_method = config["VSA"]["seeding_method"].as<decltype(vsa_config.seeding_method)>();
    vsa_config.max_number_of_proxies =
        config["VSA"]["max_number_of_proxies"].as<decltype(vsa_config.max_number_of_proxies)>();
    vsa_config.number_of_iterations =
        config["VSA"]["number_of_iterations"].as<decltype(vsa_config.number_of_iterations)>();
    vsa_config.min_error_drop = config["VSA"]["min_error_drop"].as<decltype(vsa_config.min_error_drop)>();
}

void load_pcl_normal_config(const std::string &dir, PCL_NORMAL_CONFIG &pcl_normal_config) {
    YAML::Node config = YAML::LoadFile(dir);
    pcl_normal_config.search_region =
        config["PCL_NORMAL"]["search_region"].as<decltype(pcl_normal_config.search_region)>();
}

void load_region_growing_config(const std::string &dir, REGION_GROWING_CONFIG &region_growing_config) {
    YAML::Node config = YAML::LoadFile(dir);
    region_growing_config.is_selected =
        config["REGION_GROWING"]["is_selected"].as<decltype(region_growing_config.is_selected)>();
    region_growing_config.is_igl = config["REGION_GROWING"]["is_igl"].as<decltype(region_growing_config.is_igl)>();
    region_growing_config.min_cluster_size =
        config["REGION_GROWING"]["min_cluster_size"].as<decltype(region_growing_config.min_cluster_size)>();
    region_growing_config.max_cluster_size =
        config["REGION_GROWING"]["max_cluster_size"].as<decltype(region_growing_config.max_cluster_size)>();
    region_growing_config.number_of_neighbours =
        config["REGION_GROWING"]["number_of_neighbours"].as<decltype(region_growing_config.number_of_neighbours)>();
    region_growing_config.smoothness_threshold =
        config["REGION_GROWING"]["smoothness_threshold"].as<decltype(region_growing_config.smoothness_threshold)>();
    region_growing_config.curvature_threshold =
        config["REGION_GROWING"]["curvature_threshold"].as<decltype(region_growing_config.curvature_threshold)>();
}

void load_curvature_config(const std::string &dir, CURVATURE_CONFIG &curvature_config) {
    YAML::Node config = YAML::LoadFile(dir);
    curvature_config.gaussian_curvature_is_selected =
        config["CURVATURE"]["gaussian_curvature_is_selected"]
            .as<decltype(curvature_config.gaussian_curvature_is_selected)>();
    curvature_config.mean_curvature_is_selected =
        config["CURVATURE"]["mean_curvature_is_selected"].as<decltype(curvature_config.mean_curvature_is_selected)>();
    curvature_config.area_method = config["CURVATURE"]["area_method"].as<decltype(curvature_config.area_method)>();
};

void load_probability_clustering_config(const std::string &dir,
                                        PROBABILITY_CLUSTERING_CONFIG &probability_clustering_config) {
    YAML::Node config = YAML::LoadFile(dir);
    probability_clustering_config.is_merge =
        config["PROBABILITY_CLUSTERING"]["is_merge"].as<decltype(probability_clustering_config.is_merge)>();
    probability_clustering_config.is_average_face_normal_and_curvature =
        config["PROBABILITY_CLUSTERING"]["is_average_face_normal_and_curvature"]
            .as<decltype(probability_clustering_config.is_average_face_normal_and_curvature)>();
    probability_clustering_config.average_window_size =
        config["PROBABILITY_CLUSTERING"]["average_window_size"]
            .as<decltype(probability_clustering_config.average_window_size)>();
    probability_clustering_config.initial_window_size =
        config["PROBABILITY_CLUSTERING"]["initial_window_size"]
            .as<decltype(probability_clustering_config.initial_window_size)>();
    probability_clustering_config.th_normal_angle = config["PROBABILITY_CLUSTERING"]["th_normal_angle"]
                                                        .as<decltype(probability_clustering_config.th_normal_angle)>();
    probability_clustering_config.th_diff_normal_norm_mean =
        config["PROBABILITY_CLUSTERING"]["th_diff_normal_norm_mean"]
            .as<decltype(probability_clustering_config.th_diff_normal_norm_mean)>();
    probability_clustering_config.th_diff_normal_variance =
        config["PROBABILITY_CLUSTERING"]["th_diff_normal_variance"]
            .as<decltype(probability_clustering_config.th_diff_normal_variance)>();
    probability_clustering_config.th_K_L_J_S_threshold =
        config["PROBABILITY_CLUSTERING"]["th_K_L_J_S_threshold"]
            .as<decltype(probability_clustering_config.th_K_L_J_S_threshold)>();
    probability_clustering_config.th_diff_G_curvature_mean =
        config["PROBABILITY_CLUSTERING"]["th_diff_G_curvature_mean"]
            .as<decltype(probability_clustering_config.th_diff_G_curvature_mean)>();
    probability_clustering_config.th_diff_G_curvature_variance =
        config["PROBABILITY_CLUSTERING"]["th_diff_G_curvature_variance"]
            .as<decltype(probability_clustering_config.th_diff_G_curvature_variance)>();
    probability_clustering_config.th_diff_M_curvature_mean =
        config["PROBABILITY_CLUSTERING"]["th_diff_M_curvature_mean"]
            .as<decltype(probability_clustering_config.th_diff_M_curvature_mean)>();
    probability_clustering_config.th_diff_M_curvature_variance =
        config["PROBABILITY_CLUSTERING"]["th_diff_M_curvature_variance"]
            .as<decltype(probability_clustering_config.th_diff_M_curvature_variance)>();
    probability_clustering_config.is_normal_selected =
        config["PROBABILITY_CLUSTERING"]["is_normal_selected"]
            .as<decltype(probability_clustering_config.is_normal_selected)>();
    probability_clustering_config.diff_normal_norm_mean =
        config["PROBABILITY_CLUSTERING"]["diff_normal_norm_mean"]
            .as<decltype(probability_clustering_config.diff_normal_norm_mean)>();
    probability_clustering_config.diff_normal_variance =
        config["PROBABILITY_CLUSTERING"]["diff_normal_variance"]
            .as<decltype(probability_clustering_config.diff_normal_variance)>();
    probability_clustering_config.is_length_selected =
        config["PROBABILITY_CLUSTERING"]["is_length_selected"]
            .as<decltype(probability_clustering_config.is_length_selected)>();
    probability_clustering_config.diff_length_mean =
        config["PROBABILITY_CLUSTERING"]["diff_length_mean"]
            .as<decltype(probability_clustering_config.diff_length_mean)>();
    probability_clustering_config.diff_length_variance =
        config["PROBABILITY_CLUSTERING"]["diff_length_variance"]
            .as<decltype(probability_clustering_config.diff_length_variance)>();
    probability_clustering_config.is_G_curvature_selected =
        config["PROBABILITY_CLUSTERING"]["is_G_curvature_selected"]
            .as<decltype(probability_clustering_config.is_G_curvature_selected)>();
    probability_clustering_config.comparison_metric =
        config["PROBABILITY_CLUSTERING"]["comparison_metric"]
            .as<decltype(probability_clustering_config.comparison_metric)>();
    probability_clustering_config.K_L_J_S_threshold =
        config["PROBABILITY_CLUSTERING"]["K_L_J_S_threshold"]
            .as<decltype(probability_clustering_config.K_L_J_S_threshold)>();
    probability_clustering_config.diff_G_curvature_mean =
        config["PROBABILITY_CLUSTERING"]["diff_G_curvature_mean"]
            .as<decltype(probability_clustering_config.diff_G_curvature_mean)>();
    probability_clustering_config.diff_G_curvature_variance =
        config["PROBABILITY_CLUSTERING"]["diff_G_curvature_variance"]
            .as<decltype(probability_clustering_config.diff_G_curvature_variance)>();
    probability_clustering_config.is_M_curvature_selected =
        config["PROBABILITY_CLUSTERING"]["is_M_curvature_selected"]
            .as<decltype(probability_clustering_config.is_M_curvature_selected)>();
    probability_clustering_config.diff_M_curvature_mean =
        config["PROBABILITY_CLUSTERING"]["diff_M_curvature_mean"]
            .as<decltype(probability_clustering_config.diff_M_curvature_mean)>();
    probability_clustering_config.diff_M_curvature_variance =
        config["PROBABILITY_CLUSTERING"]["diff_M_curvature_variance"]
            .as<decltype(probability_clustering_config.diff_M_curvature_variance)>();
    probability_clustering_config.region_num =
        config["PROBABILITY_CLUSTERING"]["region_num"].as<decltype(probability_clustering_config.region_num)>();
}

void load_local_BA_config(const std::string &dir, LOCAL_BA_CONFIG &local_BA_config) {
    YAML::Node config = YAML::LoadFile(dir);
    if (config["LOCAL_BA"]["is_set_constant"]) {
        local_BA_config.is_set_constant =
            config["LOCAL_BA"]["is_set_constant"].as<decltype(local_BA_config.is_set_constant)>();
    }
    if (config["LOCAL_BA"]["is_couple"]) { // is_couple
        local_BA_config.is_couple = config["LOCAL_BA"]["is_couple"].as<decltype(local_BA_config.is_couple)>();
    }
    if (config["LOCAL_BA"]["is_couple_sph"]) { // is_couple_sph
        local_BA_config.is_couple_sph =
            config["LOCAL_BA"]["is_couple_sph"].as<decltype(local_BA_config.is_couple_sph)>();
    }
    if (config["LOCAL_BA"]["is_couple_sph_mod"]) { // is_couple_sph_mod
        local_BA_config.is_couple_sph_mod =
            config["LOCAL_BA"]["is_couple_sph_mod"].as<decltype(local_BA_config.is_couple_sph_mod)>();
    }
    if (config["LOCAL_BA"]["is_couple_switch"]) {
        local_BA_config.is_couple_switch =
            config["LOCAL_BA"]["is_couple_switch"].as<decltype(local_BA_config.is_couple_switch)>();
    }
    if (config["LOCAL_BA"]["is_odometry"]) {
        local_BA_config.is_odometry = config["LOCAL_BA"]["is_odometry"].as<decltype(local_BA_config.is_odometry)>();
    }
    if (config["LOCAL_BA"]["is_object_to_map"]) {
        local_BA_config.is_object_to_map =
            config["LOCAL_BA"]["is_object_to_map"].as<decltype(local_BA_config.is_object_to_map)>();
    }
    if (config["LOCAL_BA"]["is_object_fix_to_map"]) {
        local_BA_config.is_object_fix_to_map =
            config["LOCAL_BA"]["is_object_fix_to_map"].as<decltype(local_BA_config.is_object_fix_to_map)>();
    }
    if (config["LOCAL_BA"]["is_object_to_object"]) {
        local_BA_config.is_object_to_object =
            config["LOCAL_BA"]["is_object_to_object"].as<decltype(local_BA_config.is_object_to_object)>();
    }
    if (config["LOCAL_BA"]["is_object_point_to_object_point"]) {
        local_BA_config.is_object_point_to_object_point =
            config["LOCAL_BA"]["is_object_point_to_object_point"]
                .as<decltype(local_BA_config.is_object_point_to_object_point)>();
    }
    if (config["LOCAL_BA"]["is_sph_conformal_fix_point_to_point"]) {
        local_BA_config.is_sph_conformal_fix_point_to_point =
            config["LOCAL_BA"]["is_sph_conformal_fix_point_to_point"]
                .as<decltype(local_BA_config.is_sph_conformal_fix_point_to_point)>();
    }
    if (config["LOCAL_BA"]["is_sph_conformal_fix_point_to_map"]) {
        local_BA_config.is_sph_conformal_fix_point_to_map =
            config["LOCAL_BA"]["is_sph_conformal_fix_point_to_map"]
                .as<decltype(local_BA_config.is_sph_conformal_fix_point_to_map)>();
    }
    if (config["LOCAL_BA"]["is_sph_conformal_point_to_map"]) {
        local_BA_config.is_sph_conformal_point_to_map =
            config["LOCAL_BA"]["is_sph_conformal_point_to_map"]
                .as<decltype(local_BA_config.is_sph_conformal_point_to_map)>();
    }
    if (config["LOCAL_BA"]["is_sph_fix_point_to_map"]) {
        local_BA_config.is_sph_fix_point_to_map =
            config["LOCAL_BA"]["is_sph_fix_point_to_map"].as<decltype(local_BA_config.is_sph_fix_point_to_map)>();
    }
    if (config["LOCAL_BA"]["is_nn_data_association"]) {
        local_BA_config.is_nn_data_association =
            config["LOCAL_BA"]["is_nn_data_association"].as<decltype(local_BA_config.is_nn_data_association)>();
    }
    if (config["LOCAL_BA"]["is_pure_ICP"]) {
        local_BA_config.is_pure_ICP = config["LOCAL_BA"]["is_pure_ICP"].as<decltype(local_BA_config.is_pure_ICP)>();
    }
    if (config["LOCAL_BA"]["is_direct_method"]) {
        local_BA_config.is_direct_method =
            config["LOCAL_BA"]["is_direct_method"].as<decltype(local_BA_config.is_direct_method)>();
    }
    if (config["LOCAL_BA"]["is_obj_fix"]) {
        local_BA_config.is_obj_fix = config["LOCAL_BA"]["is_obj_fix"].as<decltype(local_BA_config.is_obj_fix)>();
    }
    if (config["LOCAL_BA"]["weight"]) {
        local_BA_config.weight = config["LOCAL_BA"]["weight"].as<decltype(local_BA_config.weight)>();
    }
    if (config["LOCAL_BA"]["max_num_iterations"]) {
        local_BA_config.max_num_iterations =
            config["LOCAL_BA"]["max_num_iterations"].as<decltype(local_BA_config.max_num_iterations)>();
    }
    if (config["LOCAL_BA"]["update_state_every_iteration"]) {
        local_BA_config.update_state_every_iteration =
            config["LOCAL_BA"]["update_state_every_iteration"]
                .as<decltype(local_BA_config.update_state_every_iteration)>();
    }
    if (config["LOCAL_BA"]["iter_num"]) {
        local_BA_config.iter_num = config["LOCAL_BA"]["iter_num"].as<decltype(local_BA_config.iter_num)>();
    }
    if (config["LOCAL_BA"]["threshold"]) {
        local_BA_config.threshold = config["LOCAL_BA"]["threshold"].as<decltype(local_BA_config.threshold)>();
    }
    if (config["LOCAL_BA"]["is_debug"]) {
        local_BA_config.is_debug = config["LOCAL_BA"]["is_debug"].as<decltype(local_BA_config.is_debug)>();
    }
    if (config["LOCAL_BA"]["is_ground_truth_initial"]) {
        local_BA_config.is_ground_truth_initial =
            config["LOCAL_BA"]["is_ground_truth_initial"].as<decltype(local_BA_config.is_ground_truth_initial)>();
    }
    if (config["LOCAL_BA"]["debug_2D_projection_dir"]) {
        local_BA_config.debug_2D_projection_dir =
            config["LOCAL_BA"]["debug_2D_projection_dir"].as<decltype(local_BA_config.debug_2D_projection_dir)>();
    }
};

void load_direct_method_config(const std::string &dir, DIRECT_METHOD_CONFIG &direct_method_config) {
    YAML::Node config = YAML::LoadFile(dir);
    direct_method_config.method = config["DIRECT_METHOD"]["method"].as<decltype(direct_method_config.method)>();
    direct_method_config.valid_region_method =
        config["DIRECT_METHOD"]["valid_region_method"].as<decltype(direct_method_config.valid_region_method)>();
    if (config["DIRECT_METHOD"]["CURL_ODOMETRY"]["minimum_img_rso"]) {
        direct_method_config.minimum_img_rso = config["DIRECT_METHOD"]["CURL_ODOMETRY"]["minimum_img_rso"]
                                                   .as<decltype(direct_method_config.minimum_img_rso)>();
    }
    if (config["DIRECT_METHOD"]["CURL_ODOMETRY"]["gamma_sampling_interval"]) {
        direct_method_config.gamma_sampling_interval =
            config["DIRECT_METHOD"]["CURL_ODOMETRY"]["gamma_sampling_interval"]
                .as<decltype(direct_method_config.gamma_sampling_interval)>();
    }
    direct_method_config.pyramid_minimum_img_rso =
        config["DIRECT_METHOD"]["pyramid_minimum_img_rso"].as<decltype(direct_method_config.pyramid_minimum_img_rso)>();
    direct_method_config.is_pyramid_track =
        config["DIRECT_METHOD"]["is_pyramid_track"].as<decltype(direct_method_config.is_pyramid_track)>();
    direct_method_config.is_display =
        config["DIRECT_METHOD"]["is_display"].as<decltype(direct_method_config.is_display)>();
    direct_method_config.is_display_block =
        config["DIRECT_METHOD"]["is_display_block"].as<decltype(direct_method_config.is_display_block)>();
    direct_method_config.max_iters =
        config["DIRECT_METHOD"]["max_iters"].as<decltype(direct_method_config.max_iters)>();
    direct_method_config.update_points_method =
        config["DIRECT_METHOD"]["update_points_method"].as<decltype(direct_method_config.update_points_method)>();
    direct_method_config.is_update_sph_coeff =
        config["DIRECT_METHOD"]["is_update_sph_coeff"].as<decltype(direct_method_config.is_update_sph_coeff)>();
    direct_method_config.update_sph_coeff_threshold =
        config["DIRECT_METHOD"]["update_sph_coeff_threshold"]
            .as<decltype(direct_method_config.update_sph_coeff_threshold)>();
    direct_method_config.is_gaussian_blur =
        config["DIRECT_METHOD"]["is_gaussian_blur"].as<decltype(direct_method_config.is_gaussian_blur)>();
    direct_method_config.sigma = config["DIRECT_METHOD"]["sigma"].as<decltype(direct_method_config.sigma)>();
    direct_method_config.loss_function =
        config["DIRECT_METHOD"]["loss_function"].as<decltype(direct_method_config.loss_function)>();
    direct_method_config.IQR_delta =
        config["DIRECT_METHOD"]["IQR_delta"].as<decltype(direct_method_config.IQR_delta)>();
    direct_method_config.linear_loss_param =
        config["DIRECT_METHOD"]["linear_loss_param"].as<decltype(direct_method_config.linear_loss_param)>();
    direct_method_config.trapeziform_lower_bound =
        config["DIRECT_METHOD"]["trapeziform_lower_bound"].as<decltype(direct_method_config.trapeziform_lower_bound)>();
    direct_method_config.trapeziform_higher_bound = config["DIRECT_METHOD"]["trapeziform_higher_bound"]
                                                        .as<decltype(direct_method_config.trapeziform_higher_bound)>();
}
template <typename T> void load_SH_table_config(const std::string &dir, SH_TABLE_CONFIG<T> &SH_table_config) {
    YAML::Node config = YAML::LoadFile(dir);
    SH_table_config.is_speed_up = config["SH_TABLE"]["is_speed_up"].as<decltype(SH_table_config.is_speed_up)>();
    SH_table_config.max_SH_degree = config["SH_TABLE"]["max_SH_degree"].as<decltype(SH_table_config.max_SH_degree)>();
    SH_table_config.warping_SH_degree =
        config["SH_TABLE"]["warping_SH_degree"].as<decltype(SH_table_config.warping_SH_degree)>();
    if (config["SH_TABLE"]["ground_SH_degree"]) {
        SH_table_config.ground_SH_degree =
            config["SH_TABLE"]["ground_SH_degree"].as<decltype(SH_table_config.ground_SH_degree)>();
    }
    SH_table_config.SH_scale = config["SH_TABLE"]["SH_scale"].as<decltype(SH_table_config.SH_scale)>();
    SH_table_config.azi_rso = config["SH_TABLE"]["azi_rso"].as<decltype(SH_table_config.azi_rso)>();
    SH_table_config.elev_rso = config["SH_TABLE"]["elev_rso"].as<decltype(SH_table_config.elev_rso)>();
    SH_table_config.init_SPH_pass = config["SH_TABLE"]["init_SPH_pass"].as<decltype(SH_table_config.init_SPH_pass)>();
    SH_table_config.inv_method = config["SH_TABLE"]["inv_method"].as<decltype(SH_table_config.inv_method)>();
    SH_table_config.bad_thres = config["SH_TABLE"]["bad_thres"].as<decltype(SH_table_config.bad_thres)>();
    if (config["SH_TABLE"]["medium_thres"]) {
        SH_table_config.medium_thres = config["SH_TABLE"]["medium_thres"].as<decltype(SH_table_config.medium_thres)>();
    }
    if (config["SH_TABLE"]["update_sph_counter_thres"]) {
        SH_table_config.update_sph_counter_thres =
            config["SH_TABLE"]["update_sph_counter_thres"].as<decltype(SH_table_config.update_sph_counter_thres)>();
    }
    if (config["SH_TABLE"]["update_sph_cpeff_queue_size"]) {
        SH_table_config.update_sph_cpeff_queue_size = config["SH_TABLE"]["update_sph_cpeff_queue_size"]
                                                          .as<decltype(SH_table_config.update_sph_cpeff_queue_size)>();
    }
    if (config["SH_TABLE"]["sampling_factor"]) {
        SH_table_config.sampling_factor =
            config["SH_TABLE"]["sampling_factor"].as<decltype(SH_table_config.sampling_factor)>();
    }
}

void load_conformal_mapping_config(const std::string &dir, CONFORMAL_MAPPING_CONFIG &conformal_mapping_config) {
    YAML::Node config = YAML::LoadFile(dir);
    if (config["CONFORMAL_MAPPING"]["is_rt"]) {
        conformal_mapping_config.is_rt =
            config["CONFORMAL_MAPPING"]["is_rt"].as<decltype(conformal_mapping_config.is_rt)>();
    }
    if (config["CONFORMAL_MAPPING"]["ICP"]["is_icp"]) {
        conformal_mapping_config.is_icp =
            config["CONFORMAL_MAPPING"]["ICP"]["is_icp"].as<decltype(conformal_mapping_config.is_icp)>();
    }
    if (config["CONFORMAL_MAPPING"]["ICP"]["is_points_distance"]) {
        conformal_mapping_config.is_points_distance = config["CONFORMAL_MAPPING"]["ICP"]["is_points_distance"]
                                                          .as<decltype(conformal_mapping_config.is_points_distance)>();
    }

    if (config["CONFORMAL_MAPPING"]["thread_num"]) {
        conformal_mapping_config.thread_num =
            config["CONFORMAL_MAPPING"]["thread_num"].as<decltype(conformal_mapping_config.thread_num)>();
    }
    if (config["CONFORMAL_MAPPING"]["MAPPING"]["map_window_size"]) {
        conformal_mapping_config.map_window_size = config["CONFORMAL_MAPPING"]["MAPPING"]["map_window_size"]
                                                       .as<decltype(conformal_mapping_config.map_window_size)>();
    }
    if (config["CONFORMAL_MAPPING"]["SEGMENTATION"]["is_even_segmentation"]) {
        conformal_mapping_config.is_even_segmentation =
            config["CONFORMAL_MAPPING"]["SEGMENTATION"]["is_even_segmentation"]
                .as<decltype(conformal_mapping_config.is_even_segmentation)>();
    }
    if (config["CONFORMAL_MAPPING"]["SEGMENTATION"]["cut_threshold"]) {
        conformal_mapping_config.cut_threshold = config["CONFORMAL_MAPPING"]["SEGMENTATION"]["cut_threshold"]
                                                     .as<decltype(conformal_mapping_config.cut_threshold)>();
    }
    if (config["CONFORMAL_MAPPING"]["SEGMENTATION"]["ground_cut_threshold"]) { // for ground
        conformal_mapping_config.ground_cut_threshold =
            config["CONFORMAL_MAPPING"]["SEGMENTATION"]["ground_cut_threshold"]
                .as<decltype(conformal_mapping_config.ground_cut_threshold)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["patch_minimum_pts_num"]) {
        conformal_mapping_config.patch_minimum_pts_num =
            config["CONFORMAL_MAPPING"]["FILTER"]["patch_minimum_pts_num"]
                .as<decltype(conformal_mapping_config.patch_minimum_pts_num)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["patch_minimun_pts_num_after_filter"]) {
        conformal_mapping_config.patch_minimun_pts_num_after_filter =
            config["CONFORMAL_MAPPING"]["FILTER"]["patch_minimun_pts_num_after_filter"]
                .as<decltype(conformal_mapping_config.patch_minimun_pts_num_after_filter)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["minimum_triangles_after_filter"]) {
        conformal_mapping_config.minimum_triangles_after_filter =
            config["CONFORMAL_MAPPING"]["FILTER"]["minimum_triangles_after_filter"]
                .as<decltype(conformal_mapping_config.minimum_triangles_after_filter)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["is_voxel_grid_filter"]) {
        conformal_mapping_config.is_voxel_grid_filter =
            config["CONFORMAL_MAPPING"]["FILTER"]["is_voxel_grid_filter"]
                .as<decltype(conformal_mapping_config.is_voxel_grid_filter)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["leaf_size"]) {
        conformal_mapping_config.leaf_size =
            config["CONFORMAL_MAPPING"]["FILTER"]["leaf_size"].as<decltype(conformal_mapping_config.leaf_size)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["is_StatisticalOutlierRemoval"]) {
        config["CONFORMAL_MAPPING"]["FILTER"]["is_StatisticalOutlierRemoval"]
            .as<decltype(conformal_mapping_config.is_StatisticalOutlierRemoval)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["outlier_removing_neighbour_num"]) {
        conformal_mapping_config.outlier_removing_neighbour_num =
            config["CONFORMAL_MAPPING"]["FILTER"]["outlier_removing_neighbour_num"]
                .as<decltype(conformal_mapping_config.outlier_removing_neighbour_num)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["outlier_removing_std"]) {
        conformal_mapping_config.outlier_removing_std =
            config["CONFORMAL_MAPPING"]["FILTER"]["outlier_removing_std"]
                .as<decltype(conformal_mapping_config.outlier_removing_std)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["is_RadiusOutlierRemoval"]) {
        conformal_mapping_config.is_RadiusOutlierRemoval =
            config["CONFORMAL_MAPPING"]["FILTER"]["is_RadiusOutlierRemoval"]
                .as<decltype(conformal_mapping_config.is_RadiusOutlierRemoval)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["outlier_removing_radius"]) {
        conformal_mapping_config.outlier_removing_radius =
            config["CONFORMAL_MAPPING"]["FILTER"]["outlier_removing_radius"]
                .as<decltype(conformal_mapping_config.outlier_removing_radius)>();
    }
    if (config["CONFORMAL_MAPPING"]["FILTER"]["outlier_removing_min_nei"]) {
        conformal_mapping_config.outlier_removing_min_nei =
            config["CONFORMAL_MAPPING"]["FILTER"]["outlier_removing_min_nei"]
                .as<decltype(conformal_mapping_config.outlier_removing_min_nei)>();
    }

    if (config["CONFORMAL_MAPPING"]["TRIANGULATION"]["defect_triangulation_type"]) {
        conformal_mapping_config.defect_triangulation_type =
            config["CONFORMAL_MAPPING"]["TRIANGULATION"]["defect_triangulation_type"]
                .as<decltype(conformal_mapping_config.defect_triangulation_type)>();
    }
    if (config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_angle_min"]) {
        conformal_mapping_config.tri_angle_min = config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_angle_min"]
                                                     .as<decltype(conformal_mapping_config.tri_angle_min)>() *
                                                 M_PI / 180;
    }
    if (config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_angle_max"]) {
        conformal_mapping_config.tri_angle_max = config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_angle_max"]
                                                     .as<decltype(conformal_mapping_config.tri_angle_max)>() *
                                                 M_PI / 180;
    }
    if (config["CONFORMAL_MAPPING"]["TRIANGULATION"]["is_IQR"]) {
        conformal_mapping_config.is_IQR =
            config["CONFORMAL_MAPPING"]["TRIANGULATION"]["is_IQR"].as<decltype(conformal_mapping_config.is_IQR)>();
    }
    if (config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_area_max_scale_factor"]) {
        conformal_mapping_config.tri_area_max_scale_factor =
            config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_area_max_scale_factor"]
                .as<decltype(conformal_mapping_config.tri_area_max_scale_factor)>();
    }
    if (config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_length_max_scale_factor"]) {
        conformal_mapping_config.tri_length_max_scale_factor =
            config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_length_max_scale_factor"]
                .as<decltype(conformal_mapping_config.tri_length_max_scale_factor)>();
    }
    if (config["CONFORMAL_MAPPING"]["TRIANGULATION"]["is_hist"]) {
        conformal_mapping_config.is_hist =
            config["CONFORMAL_MAPPING"]["TRIANGULATION"]["is_hist"].as<decltype(conformal_mapping_config.is_hist)>();
    }
    if (config["CONFORMAL_MAPPING"]["TRIANGULATION"]["is_hist_fall"]) {
        conformal_mapping_config.is_hist_fall = config["CONFORMAL_MAPPING"]["TRIANGULATION"]["is_hist_fall"]
                                                    .as<decltype(conformal_mapping_config.is_hist_fall)>();
    }
    if (config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_area_percentage"]) {
        conformal_mapping_config.tri_area_percentage =
            config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_area_percentage"]
                .as<decltype(conformal_mapping_config.tri_area_percentage)>();
    }
    if (config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_length_percentage"]) {
        conformal_mapping_config.tri_length_percentage =
            config["CONFORMAL_MAPPING"]["TRIANGULATION"]["tri_length_percentage"]
                .as<decltype(conformal_mapping_config.tri_length_percentage)>();
    }
    if (config["CONFORMAL_MAPPING"]["CONFORMALMAPPING"]["rectangular_corner_weight"]) { // 0.0~1.0
        conformal_mapping_config.rectangular_corner_weight =
            config["CONFORMAL_MAPPING"]["CONFORMALMAPPING"]["rectangular_corner_weight"]
                .as<decltype(conformal_mapping_config.rectangular_corner_weight)>();
    }

    if (config["CONFORMAL_MAPPING"]["CONFORMALMAPPING"]["height_opt_iters"]) {
        conformal_mapping_config.height_opt_iters = config["CONFORMAL_MAPPING"]["CONFORMALMAPPING"]["height_opt_iters"]
                                                        .as<decltype(conformal_mapping_config.height_opt_iters)>();
    }
}

void load_debug_config(const std::string &dir, DEBUG_CONFIG &debug_config) {
    YAML::Node config = YAML::LoadFile(dir);
    if (config["DEBUG"]["results_dir"]) {
        debug_config.results_dir = config["DEBUG"]["results_dir"].as<decltype(debug_config.results_dir)>();
    }
    if (config["DEBUG"]["data_set"]) {
        debug_config.data_set = config["DEBUG"]["data_set"].as<decltype(debug_config.data_set)>();
    }
    if (config["DEBUG"]["seq"]) { // 0~10
        debug_config.seq = config["DEBUG"]["seq"].as<decltype(debug_config.seq)>();
    }
    if (config["DEBUG"]["is_save_middle_results"]) {
        debug_config.is_save_middle_results =
            config["DEBUG"]["is_save_middle_results"].as<decltype(debug_config.is_save_middle_results)>();
    }
    if (config["DEBUG"]["is_save_final_results"]) {
        debug_config.is_save_final_results =
            config["DEBUG"]["is_save_final_results"].as<decltype(debug_config.is_save_final_results)>();
    }
    if (config["DEBUG"]["is_evaluate_time"]) {
        debug_config.is_evaluate_time =
            config["DEBUG"]["is_evaluate_time"].as<decltype(debug_config.is_evaluate_time)>();
    }
    if (config["DEBUG"]["MIDDLE_MAP"]["is_save_middle_map"]) {
        debug_config.is_save_middle_map =
            config["DEBUG"]["MIDDLE_MAP"]["is_save_middle_map"].as<decltype(debug_config.is_save_middle_map)>();
    }
    if (config["DEBUG"]["MIDDLE_MAP"]["middle_map_highest_IoU_new_landmark_thres"]) {
        debug_config.middle_map_highest_IoU_new_landmark_thres =
            config["DEBUG"]["MIDDLE_MAP"]["middle_map_highest_IoU_new_landmark_thres"]
                .as<decltype(debug_config.middle_map_highest_IoU_new_landmark_thres)>();
    }
    if (config["DEBUG"]["FINAL_MAP"]["LOAD_MIDDLE_MAP"]["is_load_middle_map"]) {
        debug_config.is_load_middle_map = config["DEBUG"]["FINAL_MAP"]["LOAD_MIDDLE_MAP"]["is_load_middle_map"]
                                              .as<decltype(debug_config.is_load_middle_map)>();
    }
    if (config["DEBUG"]["FINAL_MAP"]["LOAD_MIDDLE_MAP"]["is_load_aabb_map"]) {
        debug_config.is_load_aabb_map = config["DEBUG"]["FINAL_MAP"]["LOAD_MIDDLE_MAP"]["is_load_aabb_map"]
                                            .as<decltype(debug_config.is_load_aabb_map)>();
    }
    if (config["DEBUG"]["FINAL_MAP"]["LOAD_FINAL_MAP"]["is_load_final_map"]) {
        debug_config.is_load_final_map = config["DEBUG"]["FINAL_MAP"]["LOAD_FINAL_MAP"]["is_load_final_map"]
                                             .as<decltype(debug_config.is_load_final_map)>();
    }
}

void load_aabb_config(const std::string &dir, AABB_CONFIG &aabb_config) {
    YAML::Node config = YAML::LoadFile(dir);
    if (config["AABB"]["skin_thickness"]) {
        aabb_config.skin_thickness = config["AABB"]["skin_thickness"].as<decltype(aabb_config.skin_thickness)>();
    }
    if (config["AABB"]["extra_skin_thickness"]) {
        aabb_config.extra_skin_thickness =
            config["AABB"]["extra_skin_thickness"].as<decltype(aabb_config.extra_skin_thickness)>();
    }
    if (config["AABB"]["ground_fatten_scale"]) {
        aabb_config.ground_fatten_scale =
            config["AABB"]["ground_fatten_scale"].as<decltype(aabb_config.ground_fatten_scale)>();
    }
    if (config["AABB"]["touch_is_overlap"]) {
        aabb_config.touch_is_overlap = config["AABB"]["touch_is_overlap"].as<decltype(aabb_config.touch_is_overlap)>();
    }
}

void load_curl_tracking(const std::string &dir, CURL_TRACKING_CONFIG &curl_tracking_config) {
    YAML::Node config = YAML::LoadFile(dir);
    if (config["CURL_TRACKING"]["frequency"]) {
        curl_tracking_config.frequency =
            config["CURL_TRACKING"]["frequency"].as<decltype(curl_tracking_config.frequency)>();
    }
    if (config["CURL_TRACKING"]["is_frame_to_frame"]) {
        curl_tracking_config.is_frame_to_frame =
            config["CURL_TRACKING"]["is_frame_to_frame"].as<decltype(curl_tracking_config.is_frame_to_frame)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["IoU_first_level"]) {
        curl_tracking_config.IoU_first_level = config["CURL_TRACKING"]["DATA_ASSOCIATION"]["IoU_first_level"]
                                                   .as<decltype(curl_tracking_config.IoU_first_level)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["IoU_second_level"]) {
        curl_tracking_config.IoU_second_level = config["CURL_TRACKING"]["DATA_ASSOCIATION"]["IoU_second_level"]
                                                    .as<decltype(curl_tracking_config.IoU_second_level)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["centroids_distance"]) {
        curl_tracking_config.centroids_distance = config["CURL_TRACKING"]["DATA_ASSOCIATION"]["centroids_distance"]
                                                      .as<decltype(curl_tracking_config.centroids_distance)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["ground_IoU_first_level"]) {
        curl_tracking_config.ground_IoU_first_level =
            config["CURL_TRACKING"]["DATA_ASSOCIATION"]["ground_IoU_first_level"]
                .as<decltype(curl_tracking_config.ground_IoU_first_level)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["valid_ground_radius"]) {
        curl_tracking_config.valid_ground_radius = config["CURL_TRACKING"]["DATA_ASSOCIATION"]["valid_ground_radius"]
                                                       .as<decltype(curl_tracking_config.valid_ground_radius)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["weight_of_IoU"]) { // 0.0~1.0
        curl_tracking_config.weight_of_IoU = config["CURL_TRACKING"]["DATA_ASSOCIATION"]["weight_of_IoU"]
                                                 .as<decltype(curl_tracking_config.weight_of_IoU)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["largest_centroid_distance"]) {
        curl_tracking_config.largest_centroid_distance =
            config["CURL_TRACKING"]["DATA_ASSOCIATION"]["largest_centroid_distance"]
                .as<decltype(curl_tracking_config.largest_centroid_distance)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["largest_tree_key_difference"]) {
        curl_tracking_config.largest_tree_key_difference =
            config["CURL_TRACKING"]["DATA_ASSOCIATION"]["largest_tree_key_difference"]
                .as<decltype(curl_tracking_config.largest_tree_key_difference)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["IQR_scale_factor"]) {
        curl_tracking_config.IQR_scale_factor = config["CURL_TRACKING"]["DATA_ASSOCIATION"]["IQR_scale_factor"]
                                                    .as<decltype(curl_tracking_config.IQR_scale_factor)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["is_select_the_best_pair_only"]) {
        curl_tracking_config.is_select_the_best_pair_only =
            config["CURL_TRACKING"]["DATA_ASSOCIATION"]["is_select_the_best_pair_only"]
                .as<decltype(curl_tracking_config.is_select_the_best_pair_only)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["is_use_score_as_weight"]) {
        curl_tracking_config.is_use_score_as_weight =
            config["CURL_TRACKING"]["DATA_ASSOCIATION"]["is_use_score_as_weight"]
                .as<decltype(curl_tracking_config.is_use_score_as_weight)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["max_seg_pairs"]) {
        curl_tracking_config.max_seg_pairs = config["CURL_TRACKING"]["DATA_ASSOCIATION"]["max_seg_pairs"]
                                                 .as<decltype(curl_tracking_config.max_seg_pairs)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["max_ground_pairs"]) {
        curl_tracking_config.max_ground_pairs = config["CURL_TRACKING"]["DATA_ASSOCIATION"]["max_ground_pairs"]
                                                    .as<decltype(curl_tracking_config.max_ground_pairs)>();
    }
    if (config["CURL_TRACKING"]["DATA_ASSOCIATION"]["max_query_paired_patches"]) {
        curl_tracking_config.max_query_paired_patches =
            config["CURL_TRACKING"]["DATA_ASSOCIATION"]["max_query_paired_patches"]
                .as<decltype(curl_tracking_config.max_query_paired_patches)>();
    }
    if (config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["minimum_points_to_add_new_patch"]) {
        curl_tracking_config.minimum_points_to_add_new_patch =
            config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["minimum_points_to_add_new_patch"]
                .as<decltype(curl_tracking_config.minimum_points_to_add_new_patch)>();
    }

    if (config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["highest_IoU_new_landmark_thres"]) {
        curl_tracking_config.highest_IoU_new_landmark_thres =
            config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["highest_IoU_new_landmark_thres"]
                .as<decltype(curl_tracking_config.highest_IoU_new_landmark_thres)>();
    }
    if (config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["lowest_IoU_for_same_obj_different_patch"]) {
        curl_tracking_config.lowest_IoU_for_same_obj_different_patch =
            config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["lowest_IoU_for_same_obj_different_patch"]
                .as<decltype(curl_tracking_config.lowest_IoU_for_same_obj_different_patch)>();
    }
    if (config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["lowest_landmark_centroids_distance"]) {
        curl_tracking_config.lowest_landmark_centroids_distance =
            config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["lowest_landmark_centroids_distance"]
                .as<decltype(curl_tracking_config.lowest_landmark_centroids_distance)>();
    }
    if (config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["landmark_queue_size"]) {
        curl_tracking_config.landmark_queue_size =
            config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["landmark_queue_size"]
                .as<decltype(curl_tracking_config.landmark_queue_size)>();
    }
    if (config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["max_new_added_landmarks_num"]) {
        curl_tracking_config.max_new_added_landmarks_num =
            config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["max_new_added_landmarks_num"]
                .as<decltype(curl_tracking_config.max_new_added_landmarks_num)>();
    }
    if (config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["IQR_max_distance_of_landmarks"]) {
        curl_tracking_config.IQR_max_distance_of_landmarks =
            config["CURL_TRACKING"]["NEW_LANDMARK_GENERATION"]["IQR_max_distance_of_landmarks"]
                .as<decltype(curl_tracking_config.IQR_max_distance_of_landmarks)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["is_two_step_opt"]) {
        curl_tracking_config.is_two_step_opt = config["CURL_TRACKING"]["OPTIMIZATION"]["is_two_step_opt"]
                                                   .as<decltype(curl_tracking_config.is_two_step_opt)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["min_num_ground_pairs"]) {
        curl_tracking_config.min_num_ground_pairs = config["CURL_TRACKING"]["OPTIMIZATION"]["min_num_ground_pairs"]
                                                        .as<decltype(curl_tracking_config.min_num_ground_pairs)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["min_num_seg_pairs"]) {
        curl_tracking_config.min_num_seg_pairs = config["CURL_TRACKING"]["OPTIMIZATION"]["min_num_seg_pairs"]
                                                     .as<decltype(curl_tracking_config.min_num_seg_pairs)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["max_num_iterations"]) {
        curl_tracking_config.max_num_iterations = config["CURL_TRACKING"]["OPTIMIZATION"]["max_num_iterations"]
                                                      .as<decltype(curl_tracking_config.max_num_iterations)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["update_state_every_iteration"]) {
        curl_tracking_config.update_state_every_iteration =
            config["CURL_TRACKING"]["OPTIMIZATION"]["update_state_every_iteration"]
                .as<decltype(curl_tracking_config.update_state_every_iteration)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["is_robust_kernel"]) {
        curl_tracking_config.is_robust_kernel = config["CURL_TRACKING"]["OPTIMIZATION"]["is_robust_kernel"]
                                                    .as<decltype(curl_tracking_config.is_robust_kernel)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["kernel_threshold"]) {
        curl_tracking_config.kernel_threshold = config["CURL_TRACKING"]["OPTIMIZATION"]["kernel_threshold"]
                                                    .as<decltype(curl_tracking_config.kernel_threshold)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["max_valid_residual"]) {
        curl_tracking_config.max_valid_residual = config["CURL_TRACKING"]["OPTIMIZATION"]["max_valid_residual"]
                                                      .as<decltype(curl_tracking_config.max_valid_residual)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["opt_num_threads"]) {
        curl_tracking_config.opt_num_threads = config["CURL_TRACKING"]["OPTIMIZATION"]["opt_num_threads"]
                                                   .as<decltype(curl_tracking_config.opt_num_threads)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["ground_opt_step"]) {
        curl_tracking_config.ground_opt_step = config["CURL_TRACKING"]["OPTIMIZATION"]["ground_opt_step"]
                                                   .as<decltype(curl_tracking_config.ground_opt_step)>();
    }
    if (config["CURL_TRACKING"]["OPTIMIZATION"]["is_stop_after_cost_increased"]) {
        curl_tracking_config.is_stop_after_cost_increased =
            config["CURL_TRACKING"]["OPTIMIZATION"]["is_stop_after_cost_increased"]
                .as<decltype(curl_tracking_config.is_stop_after_cost_increased)>();
    }
    if (config["CURL_TRACKING"]["VISUALIZATION"]["color_change_step"]) {
        curl_tracking_config.color_change_step = config["CURL_TRACKING"]["VISUALIZATION"]["color_change_step"]
                                                     .as<decltype(curl_tracking_config.color_change_step)>();
    }
}

void load_map_generation(const std::string &dir, MAP_GENERATION_CONFIG &map_generation_config) {
    YAML::Node config = YAML::LoadFile(dir);
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["position"] &&
        config["MAP_GENERATION"]["MAP_PROPERTIES"]["position"].IsSequence()) {
        auto position_vec = config["MAP_GENERATION"]["MAP_PROPERTIES"]["position"].as<std::vector<double>>();
        map_generation_config.position.x = position_vec[0];
        map_generation_config.position.y = position_vec[1];
        map_generation_config.position.z = position_vec[2];
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["minimum_rso"]) {
        map_generation_config.minimum_rso =
            config["MAP_GENERATION"]["MAP_PROPERTIES"]["minimum_rso"].as<decltype(map_generation_config.minimum_rso)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["degree"]) {
        map_generation_config.degree =
            config["MAP_GENERATION"]["MAP_PROPERTIES"]["degree"].as<decltype(map_generation_config.degree)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["radius"]) {
        map_generation_config.radius =
            config["MAP_GENERATION"]["MAP_PROPERTIES"]["radius"].as<decltype(map_generation_config.radius)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["IQR_thres"]) {
        map_generation_config.IQR_thres =
            config["MAP_GENERATION"]["MAP_PROPERTIES"]["IQR_thres"].as<decltype(map_generation_config.IQR_thres)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["method"]) {
        map_generation_config.method =
            config["MAP_GENERATION"]["MAP_PROPERTIES"]["method"].as<decltype(map_generation_config.method)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_start_saving"]) {
        map_generation_config.is_start_saving = config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_start_saving"]
                                                    .as<decltype(map_generation_config.is_start_saving)>();
    }
    if (config["DEBUG"]["MAP_PROPERTIES"]["is_save_final_map"]) {
        map_generation_config.is_save_final_map = config["DEBUG"]["MAP_PROPERTIES"]["is_save_final_map"]
                                                      .as<decltype(map_generation_config.is_save_final_map)>();
    }

    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_save_ground_info"]) {
        map_generation_config.is_save_ground_info = config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_save_ground_info"]
                                                        .as<decltype(map_generation_config.is_save_ground_info)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_save_bounding_box_info"]) {
        map_generation_config.is_save_bounding_box_info =
            config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_save_bounding_box_info"]
                .as<decltype(map_generation_config.is_save_bounding_box_info)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_save_local_points"]) {
        map_generation_config.is_save_local_points = config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_save_local_points"]
                                                         .as<decltype(map_generation_config.is_save_local_points)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_save_whole_points"]) {
        map_generation_config.is_save_whole_points = config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_save_whole_points"]
                                                         .as<decltype(map_generation_config.is_save_whole_points)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["outlier_patch_IQR_thres"]) {
        map_generation_config.outlier_patch_IQR_thres =
            config["MAP_GENERATION"]["MAP_PROPERTIES"]["outlier_patch_IQR_thres"]
                .as<decltype(map_generation_config.outlier_patch_IQR_thres)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_show_cleaned_map"]) {
        map_generation_config.is_show_cleaned_map = config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_show_cleaned_map"]
                                                        .as<decltype(map_generation_config.is_show_cleaned_map)>();
    }
    if (config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_publish"]) {
        map_generation_config.is_publish =
            config["MAP_GENERATION"]["MAP_PROPERTIES"]["is_publish"].as<decltype(map_generation_config.is_publish)>();
    }
    if (config["MAP_GENERATION"]["DIRECTORIES"]["trajectory_dir"]) {
        map_generation_config.trajectory_dir = config["MAP_GENERATION"]["DIRECTORIES"]["trajectory_dir"]
                                                   .as<decltype(map_generation_config.trajectory_dir)>();
    }
}

#endif // EXAMPLE_LOAD_CONFIG