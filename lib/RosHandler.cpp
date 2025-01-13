//
// Created by zkc on 24/03/23.
//

#include "RosHandler.h"
RosHandler::RosHandler() = default;

void RosHandler::set_pub_markers(const std::string &topic_name, const visualization_msgs::Marker &_msgs) {
    if (pub_marker_map.find(topic_name) == pub_marker_map.end()) {
        pub_marker_map[topic_name] = n.advertise<visualization_msgs::Marker>(topic_name, 10);
    }
    pub_marker_map[topic_name].publish(_msgs);
}

void RosHandler::set_pub_trajectories(const std::string &topic_name, const geometry_msgs::PoseStamped &_msgs) {
    if (pub_trajectories_map.find(topic_name) == pub_trajectories_map.end()) {
        pub_trajectories_map[topic_name] = n.advertise<geometry_msgs::PoseStamped>(topic_name, 10);
    }
    pub_trajectories_map[topic_name].publish(_msgs);
}

visualization_msgs::Marker RosHandler::set_default_marker(const std::string &frame_id, const std::string &ns,
                                                          const int &id, const float color[3], int32_t type,
                                                          const float scale[3], const float position[3],
                                                          const std::string &text) {
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

void RosHandler::plot_rviz(const std::string &ns_suffix, int start_frame, int end_frame, int obj_num,
                           RosHandler *ros_handler_ptr, std::vector<Eigen::Matrix3d> *R_w_j_vec_ptr,
                           std::vector<Eigen::Vector3d> *t_w_j_vec_ptr, std::vector<Eigen::Matrix3d> *R_o_w_vec_ptr,
                           std::vector<Eigen::Vector3d> *t_o_w_vec_ptr,
                           std::vector<std::vector<Eigen::MatrixXd>> *points_j_valid_vec_ptr,
                           std::vector<std::vector<std::vector<int>>> *overlap_idx_ptr,
                           std::vector<std::vector<Eigen::MatrixXd>> *points_j_transformed_recons_vec_ptr) {
    std::vector<Eigen::Vector3d> obj_text_position(obj_num, Eigen::Vector3d::Zero());
    float color_recons[3] = {0, 1, 0};
    float color_valid[3] = {1, 0, 0};
    float color_line[3] = {0, 0, 1};
    float line_scale[3] = {0.0025, 0.0045, 0.006};
    for (int scans_id = 0; scans_id < points_j_valid_vec_ptr->size(); ++scans_id) {
        visualization_msgs::Marker points_recons =
            set_default_marker("map", "points_recons_" + ns_suffix, scans_id, color_recons);
        visualization_msgs::Marker points_valid =
            set_default_marker("map", "points_valid_" + ns_suffix, scans_id, color_valid);
        visualization_msgs::Marker line_list = set_default_marker("map", "line_list_" + ns_suffix, scans_id, color_line,
                                                                  visualization_msgs::Marker::LINE_LIST, line_scale);
        for (int obj_idx = 0; obj_idx < obj_num; ++obj_idx) {
            Eigen::MatrixXd points_j_valid_tmp = R_w_j_vec_ptr->at(start_frame - 1 + scans_id) *
                                                 points_j_valid_vec_ptr->at(scans_id)[obj_idx].transpose();
            Eigen::MatrixXd points_j_valid =
                (points_j_valid_tmp.colwise() + t_w_j_vec_ptr->at(start_frame - 1 + scans_id)).transpose();
            Eigen::MatrixXd points_j_transformed_recons_tmp =
                R_o_w_vec_ptr->at(obj_idx).transpose() *
                points_j_transformed_recons_vec_ptr->at(scans_id)[obj_idx].transpose();
            Eigen::MatrixXd points_j_transformed_recons =
                (points_j_transformed_recons_tmp.colwise() -
                 R_o_w_vec_ptr->at(obj_idx).transpose() * t_o_w_vec_ptr->at(obj_idx))
                    .transpose();
            int counter = 0;
            for (int i = 0; i < points_j_valid.rows(); ++i) {
                geometry_msgs::Point p_valid;
                p_valid.x = points_j_valid(i, 0);
                p_valid.y = points_j_valid(i, 1);
                p_valid.z = points_j_valid(i, 2);
                points_valid.points.push_back(p_valid);
                if (counter < overlap_idx_ptr->at(scans_id)[obj_idx].size()) {
                    if (overlap_idx_ptr->at(scans_id)[obj_idx][counter] == i) {
                        geometry_msgs::Point p_recons;
                        p_recons.x = points_j_transformed_recons(counter, 0);
                        p_recons.y = points_j_transformed_recons(counter, 1);
                        p_recons.z = points_j_transformed_recons(counter, 2);
                        points_recons.points.push_back(p_recons);
                        line_list.points.push_back(p_valid);
                        line_list.points.push_back(p_recons);
                        ++counter;
                    }
                }
            }
            if (obj_text_position[obj_idx].sum() == 0) {
                obj_text_position[obj_idx] = points_j_transformed_recons.colwise().mean().transpose();
            }
            float color[3] = {1, 1, 1};
            float scale[3] = {0, 0, 0.2};
            float position[3] = {float(obj_text_position[obj_idx](0)), float(obj_text_position[obj_idx](1)),
                                 float(obj_text_position[obj_idx](2) + 1)};
            visualization_msgs::Marker obj_text = set_default_marker(
                "map", "object_" + std::to_string(obj_idx), 0, color, visualization_msgs::Marker::TEXT_VIEW_FACING,
                scale, position, "obj: " + std::to_string(obj_idx + 1));
            ros_handler_ptr->set_pub_markers("/obj_text", obj_text);
        }
        // get trajectories value
        //        geometry_msgs::PoseStamped trajectory;
        //        trajectory.header.frame_id = "map";
        //        trajectory.header.stamp = ros::Time::now();
        //        trajectory.pose.position.x = t_w_j_vec_ptr->at(start_frame - 1 + scans_id)(0);
        //        trajectory.pose.position.y = t_w_j_vec_ptr->at(start_frame - 1 + scans_id)(1);
        //        trajectory.pose.position.z = t_w_j_vec_ptr->at(start_frame - 1 + scans_id)(2);
        //        trajectory.pose.orientation.x =
        //            Eigen::Quaterniond(R_w_j_vec_ptr->at(start_frame - 1 + scans_id)).x();
        //        trajectory.pose.orientation.y =
        //            Eigen::Quaterniond(R_w_j_vec_ptr->at(start_frame - 1 + scans_id)).y();
        //        trajectory.pose.orientation.z =
        //            Eigen::Quaterniond(R_w_j_vec_ptr->at(start_frame - 1 + scans_id)).z();
        //        trajectory.pose.orientation.w =
        //            Eigen::Quaterniond(R_w_j_vec_ptr->at(start_frame - 1 + scans_id)).w();
        for (int i = 0; i < 10; ++i) {
            ros_handler_ptr->set_pub_markers("/points_line_list/" + ns_suffix + "/" + std::to_string(scans_id),
                                             points_valid);
            ros_handler_ptr->set_pub_markers("/points_line_list/" + ns_suffix + "/" + std::to_string(scans_id),
                                             points_recons);
            ros_handler_ptr->set_pub_markers("/points_line_list/" + ns_suffix + "/" + std::to_string(scans_id),
                                             line_list);
            //            ros_handler_ptr->set_pub_trajectories(
            //                "/trajectory/" + ns_suffix + "/" + std::to_string(scans_id),
            //                trajectory);
        }
    }
}
void RosHandler::plot_rviz_a_pair_of_a_patch(const std::string &ns_suffix, int start_frame, int end_frame, int obj_num,
                                             RosHandler *ros_handler_ptr,
                                             std::vector<Eigen::MatrixXd> *points_obj_recons_reference_ptr,
                                             std::vector<std::vector<Eigen::MatrixXd>> *points_obj_ptr) {
    std::vector<visualization_msgs::Marker> points_obj_recons_vec(obj_num);
    float color[3] = {0, 1, 0};
    for (int j = 0; j < obj_num; ++j) {
        points_obj_recons_vec[j] = set_default_marker("map", "points_compare_" + ns_suffix, j, color);
        for (int k = 0; k < points_obj_recons_reference_ptr->at(j).rows(); ++k) {
            geometry_msgs::Point p_recons_map;
            p_recons_map.x = points_obj_recons_reference_ptr->at(j)(k, 0);
            p_recons_map.y = points_obj_recons_reference_ptr->at(j)(k, 1);
            p_recons_map.z = points_obj_recons_reference_ptr->at(j)(k, 2);
            points_obj_recons_vec[j].points.push_back(p_recons_map);
        }
    }
    color[0] = 1;
    color[1] = 0;
    color[2] = 0;
    for (int i = 0; i < points_obj_ptr->size(); ++i) {
        for (int j = 0; j < obj_num; ++j) {
            visualization_msgs::Marker points_valid =
                set_default_marker("map", "points_compare_valid_" + ns_suffix, j, color);
            for (int k = 0; k < points_obj_ptr->at(i)[j].rows(); ++k) {
                geometry_msgs::Point p_valid;
                p_valid.x = points_obj_ptr->at(i)[j](k, 0);
                p_valid.y = points_obj_ptr->at(i)[j](k, 1);
                p_valid.z = points_obj_ptr->at(i)[j](k, 2);
                points_valid.points.push_back(p_valid);
            }
            ros_handler_ptr->set_pub_markers("/patch_compare/" + ns_suffix + "/" + std::to_string(i) + "/" +
                                                 std::to_string(j + 1),
                                             points_obj_recons_vec[j]);
            ros_handler_ptr->set_pub_markers(
                "/patch_compare/" + ns_suffix + "/" + std::to_string(i) + "/" + std::to_string(j + 1), points_valid);
        }
    }
}
