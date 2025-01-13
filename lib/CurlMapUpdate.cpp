//
// Created by zkc on 19/07/23.
//

#include "CurlMapUpdate.h"
template <typename BasicType>
CurlMapUpdate<BasicType>::CurlMapUpdate(ros::NodeHandle *nh,
                                        std::shared_ptr<CURL_TRACKING_CONFIG> _curl_tracking_config_ptr,
                                        std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr,
                                        std::shared_ptr<ConformalMappingSPH<BasicType>> _conformal_mapping_ptr)
    : curl_tracking_config_ptr(_curl_tracking_config_ptr), debug_config_ptr(_debug_config_ptr),
      conformal_mapping_ptr(_conformal_mapping_ptr), landmark_cloud_sub(*nh, "/curl/add_landmark_cloud", 10),
      landmark_T_obj_lidar_set_sub(*nh, "/curl/landmark_T_obj_lidar", 10), landmark_T_w_j_sub(*nh, "/curl/T_w_j", 10),
      sync(MySyncPolicy(10), landmark_cloud_sub, landmark_T_obj_lidar_set_sub, landmark_T_w_j_sub) {
    v_local_sub = nh->subscribe<sensor_msgs::PointCloud2>("/curl/update_sph_cloud", 10,
                                                          &CurlMapUpdate::update_sph_coeff_callback, this);

    sync.registerCallback(boost::bind(&CurlMapUpdate<BasicType>::add_landmark_call_back, this, _1, _2, _3));
}

template <typename BasicType>
void CurlMapUpdate<BasicType>::update_sph_coeff_callback(const sensor_msgs::PointCloud2::ConstPtr &v_obj_msg) {
    std::cout << "callback sph coeff" << std::endl;
    pcl::PointCloud<PointT> pcl_v_obj;
    pcl::fromROSMsg(*v_obj_msg, pcl_v_obj);
    Eigen::Map<Eigen::MatrixXf, 0, Eigen::Stride<8, 0>> eigen_v_obj(reinterpret_cast<float *>(pcl_v_obj.points.data()),
                                                                    3, static_cast<int>(pcl_v_obj.size()),
                                                                    Eigen::Stride<8, 0>());
    Eigen::MatrixX<BasicType> v_obj = eigen_v_obj.cast<BasicType>();
    std::shared_ptr<PatchInfo<BasicType>> patch_info =
        (*conformal_mapping_ptr->map_attributes_ptr)[pcl_v_obj.points[0].id];
    Timer update_sph_coeff_timer;
    update_sph_coeff_timer.start();
    bool is_recons_updated = patch_info->patch_procession_ptr->update_sph_residuals(v_obj);
    // for visualization
    if (is_recons_updated) {
        if (patch_info->color[0] + curl_tracking_config_ptr->color_change_step < 1) {
            patch_info->color[0] += curl_tracking_config_ptr->color_change_step;
        }
        if (patch_info->color[1] + curl_tracking_config_ptr->color_change_step < 1) {
            patch_info->color[1] += curl_tracking_config_ptr->color_change_step;
        }
        if (patch_info->color[2] + curl_tracking_config_ptr->color_change_step < 1) {
            patch_info->color[2] += curl_tracking_config_ptr->color_change_step;
        }
        conformal_mapping_ptr->rt_publish_landmark_patch_recons(patch_info);
    }
    std::cout << "Residual Updation: " << update_sph_coeff_timer.elapsedMilliseconds() << " ms" << std::endl;
}

template <typename BasicType>
void CurlMapUpdate<BasicType>::add_landmark_call_back(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                                                      const nav_msgs::Path::ConstPtr &path_msg,
                                                      const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    std::cout << "callback add landmark" << std::endl;
    Timer keyfrmae_timer;
    keyfrmae_timer.start();
    pcl::PointCloud<PointT> pcl_total_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_total_cloud);
    std::vector<std::vector<int>> sub_cloud_idx;
    std::vector<int> sub_cloud_idx_tmp;
    int last_id = pcl_total_cloud.points[0].id;
    for (int i = 0; i < pcl_total_cloud.size(); ++i) {
        if (last_id == pcl_total_cloud.points[i].id) {
            sub_cloud_idx_tmp.push_back(i);
        } else if (last_id != pcl_total_cloud.points[i].id || i == (pcl_total_cloud.size() - 1)) {
            sub_cloud_idx.push_back(sub_cloud_idx_tmp);
            sub_cloud_idx_tmp.clear();
            last_id = pcl_total_cloud.points[i].id;
        }
    }
    assert(sub_cloud_idx.size() == path_msg->poses.size());
    Eigen::Map<Eigen::MatrixXf, 0, Eigen::Stride<8, 0>> eigen_total_cloud(
        reinterpret_cast<float *>(pcl_total_cloud.points.data()), 3, static_cast<int>(pcl_total_cloud.size()),
        Eigen::Stride<8, 0>());
    std::vector<Eigen::MatrixX<BasicType>> point_cloud_local_vec;
    point_cloud_local_vec.reserve((sub_cloud_idx.size()));
    PointCloudInfo new_point_cloud_info;
    new_point_cloud_info.is_ground_cloud_vec.reserve(sub_cloud_idx.size());
    new_point_cloud_info.T_obj_lidar_vec.reserve(sub_cloud_idx.size());
    new_point_cloud_info.time = cloud_msg->header.stamp.toSec();
    for (int i = 0; i < sub_cloud_idx.size(); ++i) {
        point_cloud_local_vec.emplace_back(eigen_total_cloud(Eigen::all, sub_cloud_idx[i]).cast<BasicType>());
        new_point_cloud_info.is_ground_cloud_vec.push_back(pcl_total_cloud.points[sub_cloud_idx[i][0]].intensity == 0);
        Eigen::Isometry3d T_obj_lidar;
        T_obj_lidar.matrix()(0, 3) = path_msg->poses[i].pose.position.x;
        T_obj_lidar.matrix()(1, 3) = path_msg->poses[i].pose.position.y;
        T_obj_lidar.matrix()(2, 3) = path_msg->poses[i].pose.position.z;
        Eigen::Quaterniond quat;
        quat.x() = path_msg->poses[i].pose.orientation.x;
        quat.y() = path_msg->poses[i].pose.orientation.y;
        quat.z() = path_msg->poses[i].pose.orientation.z;
        quat.w() = path_msg->poses[i].pose.orientation.w;
        T_obj_lidar.matrix()(Eigen::seq(0, 2), Eigen::seq(0, 2)) = quat.normalized().toRotationMatrix();
        new_point_cloud_info.T_obj_lidar_vec.emplace_back(T_obj_lidar);
    }
    Eigen::Isometry3d T_w_lidar;
    T_w_lidar.matrix()(0, 3) = pose_msg->pose.position.x;
    T_w_lidar.matrix()(1, 3) = pose_msg->pose.position.y;
    T_w_lidar.matrix()(2, 3) = pose_msg->pose.position.z;
    Eigen::Quaterniond quat_w_lidar;
    quat_w_lidar.x() = pose_msg->pose.orientation.x;
    quat_w_lidar.y() = pose_msg->pose.orientation.y;
    quat_w_lidar.z() = pose_msg->pose.orientation.z;
    quat_w_lidar.w() = pose_msg->pose.orientation.w;
    T_w_lidar.matrix()(Eigen::seq(0, 2), Eigen::seq(0, 2)) = quat_w_lidar.normalized().toRotationMatrix();
    conformal_mapping_ptr->rt_conformal_mapping(point_cloud_local_vec, new_point_cloud_info, T_w_lidar);
    std::cout << "add landmark: " << keyfrmae_timer.elapsedMilliseconds() << " ms" << std::endl;
}

template class CurlMapUpdate<BT>; // this is very important