//
// Created by zkc on 24/03/23.
//

#ifndef SRC_ROSHANDLER_H
#define SRC_ROSHANDLER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class RosHandler {
  public:
    RosHandler();
    void set_pub_markers(const std::string &topic_name, const visualization_msgs::Marker &_msgs);
    void set_pub_trajectories(const std::string &topic_name, const geometry_msgs::PoseStamped &_msgs);
    static visualization_msgs::Marker set_default_marker(const std::string &frame_id, const std::string &ns,
                                                         const int &id, const float color[3],
                                                         int32_t type = visualization_msgs::Marker::POINTS,
                                                         const float scale[3] = (float[3]){0.01, 0.01, 0.01},
                                                         const float position[3] = (float[3]){0, 0, 0},
                                                         const std::string &text = "");
    static void plot_rviz(const std::string &ns_suffix, int start_frame, int end_frame, int obj_num,
                          RosHandler *ros_handler_ptr, std::vector<Eigen::Matrix3d> *R_w_j_vec_ptr,
                          std::vector<Eigen::Vector3d> *t_w_j_vec_ptr, std::vector<Eigen::Matrix3d> *R_o_w_vec_ptr,
                          std::vector<Eigen::Vector3d> *t_o_w_vec_ptr,
                          std::vector<std::vector<Eigen::MatrixXd>> *points_j_valid_vec_ptr,
                          std::vector<std::vector<std::vector<int>>> *overlap_idx_ptr,
                          std::vector<std::vector<Eigen::MatrixXd>> *points_j_transformed_recons_vec_ptr);
    static void plot_rviz_a_pair_of_a_patch(const std::string &ns_suffix, int start_frame, int end_frame, int obj_num,
                                            RosHandler *ros_handler_ptr,
                                            std::vector<Eigen::MatrixXd> *points_obj_recons_reference_ptr,
                                            std::vector<std::vector<Eigen::MatrixXd>> *points_obj_ptr);

  private:
    ros::NodeHandle n;
    std::unordered_map<std::string, ros::Publisher> pub_marker_map;
    std::unordered_map<std::string, ros::Publisher> pub_trajectories_map;
};

#endif // SRC_ROSHANDLER_H
