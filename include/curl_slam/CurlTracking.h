//
// Created by zkc on 06/06/23.
//

#ifndef SRC_CURLTRACKING_H
#define SRC_CURLTRACKING_H
#include "ConformalMappingSPH.h"
#include "CurlLocalBA.h"
#include "Timer.h"
#include "light_structure.h"
#include "lock_manager.h"
#include "optimization_types.h"
#include "types.h"
#include <condition_variable>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <limits>
#include <memory>
#include <mutex>
#include <nav_msgs/Path.h>
#include <omp.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <tuple>

template <typename BasicType> class CurlTracking {
  private:
    bool is_initial;
    std::shared_ptr<CURL_TRACKING_CONFIG> curl_tracking_config_ptr;
    std::shared_ptr<AABB_CONFIG> aabb_config_ptr;
    std::shared_ptr<SH_TABLE_CONFIG<BasicType>> SH_table_config_ptr;
    std::shared_ptr<ConformalMappingSPH<BasicType>> conformal_mapping_ptr;
    std::shared_ptr<DEBUG_CONFIG> debug_config_ptr;
    //    Eigen::Matrix<double, 6, 1> T_w_j_tangent;
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_w_j;
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_j_1_j;
    Eigen::Isometry3d T_w_j_1;
    int frame_id;
    // optimizatin
    ceres::Manifold *SE3_manifold;
    ceres::Manifold *SE3_manifold_x_y_yall;
    ceres::Manifold *SE3_manifold_z_roll_pitch;
    ceres::LossFunctionWrapper *loss_function;
    ceres::LossFunctionWrapper *ground_loss_function;
    ceres::LossFunctionWrapper *seg_loss_function;
    StopCallback stop_callback;
    // ROS settings
    ros::Publisher pub_trajectory;
    nav_msgs::Path trajectory;
    ros::Publisher pub_cloud;
    ros::Publisher pub_used_cloud;
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
    ros::Publisher pub_bounding_box;
    // update sph_coeff
    std::queue<std::vector<std::shared_ptr<PatchInfo<BasicType>>>> patch_info_vec_queue;
    std::queue<std::vector<Eigen::MatrixX<BasicType>>> point_cloud_obj_vec_queue;
    std::mutex update_sph_coeff_lock;
    std::condition_variable update_sph_coeff_cv;
    // add new landmark
    std::queue<std::vector<Eigen::MatrixX<BasicType>>> point_cloud_local_vec_queue;
    std::queue<PointCloudInfo> point_cloud_info_queue;
    std::queue<Eigen::Isometry3d> T_w_lidar_queue;
    std::vector<std::vector<Eigen::MatrixX<BasicType>>> middle_map_point_cloud_local_vec_vec;
    std::vector<PointCloudInfo> middle_map_point_cloud_info_vec;
    std::vector<Eigen::Isometry3d> middle_map_T_w_lidar_vec;

    std::mutex add_landmark_lock;
    std::condition_variable add_landmark_cv;
    // after add new landmark, optimization thread should be notified
    std::mutex tracking_lock;
    std::condition_variable tracking_cv;
    // time evaluation
    Timer data_asso_timer, opt_timer, keyframe_timer, update_sph_coeff_timer, total_timer;
    std::vector<double> data_association_time_vec, opt_time_vec, landmark_initialization_time_vec, update_sph_time_vec,
        total_time_vec;
    int number_patches;

  public:
    CurlTracking() = delete;
    CurlTracking(ros::NodeHandle *nh, std::shared_ptr<CURL_TRACKING_CONFIG> _curl_tracking_config_ptr,
                 std::shared_ptr<AABB_CONFIG> _aabb_config_ptr,
                 std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                 std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                 std::shared_ptr<ConformalMappingSPH<BasicType>> _conformal_mapping_ptr);
    void run();
    std::pair<bool, bool> new_landmark_add_judge(const unsigned int &key, const double &IoU,
                                                 const std::vector<double> &query_lower_bound,
                                                 const std::vector<double> &query_upper_bound,
                                                 std::shared_ptr<PatchInfo<BasicType>> patch_info_ptr,
                                                 const Eigen::Isometry3d &T_obj_lidar,
                                                 double max_new_landmark_distance);

    void publish_trajectory(ros::Time current_time);
    void publish_lidar_tf(ros::Time current_time);
    void publish_point_cloud(ros::Time current_time, const std::vector<Eigen::MatrixX<BasicType>> &point_cloud_vec,
                             const std::vector<int> &succeed_idx);
    void publish_query_map_bounding_box(
        ros::Time current_time,
        const std::vector<std::pair<std::vector<double>, std::vector<double>>> &query_bounding_box_vec,
        const std::vector<std::pair<std::vector<double>, std::vector<double>>> &map_bounding_box_vec);

    void update_sph_coeff_thread();
    void add_landmark_thread();
    void save_middle_map();

    void
    save_middle_results(const std::vector<std::pair<std::vector<double>, std::vector<double>>> &query_bounding_box_vec,
                        const std::vector<std::pair<std::vector<double>, std::vector<double>>> &map_bounding_box_vec,
                        const std::vector<Eigen::MatrixX<BasicType>> &point_cloud_vec,
                        PointCloudInfo &point_cloud_info);
};

#endif // SRC_CURLTRACKING_H
