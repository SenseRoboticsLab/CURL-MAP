//
// Created by zkc on 13/03/23.
//

#include "CURL_local_BA.h"

template <typename BasicType, typename SemanticType>
CURL_local_BA<BasicType, SemanticType>::CURL_local_BA(
    std::shared_ptr<FileReader<BasicType, SemanticType>> _file_loader_ptr, const std::string &filename)
    : file_loader_ptr(_file_loader_ptr) {}

template <typename BasicType, typename SemanticType>
CURL_local_BA<BasicType, SemanticType>::CURL_local_BA(FileReader<BasicType, SemanticType> &_file_loader,
                                                      const std::string &_config_dir)
    : file_loader(_file_loader), config_dir(_config_dir) {
    load_SH_table_config(config_dir, SH_table_config);
    curl::getSH_table(SH_table_config);
    load_local_BA_config(config_dir, local_BA_config);
    load_direct_method_config(config_dir, direct_method_config);
    debug_config_ptr = std::make_shared<DEBUG_CONFIG>();
    load_debug_config(config_dir, *debug_config_ptr);
    obj_num = file_loader.obj_num;
    // interp_valid_mat on grid
    double azi_step = (SH_table_config.azi_rso - 1) / ((SH_table_config.max_SH_degree + 1) * 2 -
                                                       1); // this decide the azi resolution used for updating sph_coeff
    azi_step = round(azi_step);
    double elev_step =
        (SH_table_config.elev_rso - 1) /
        ((SH_table_config.max_SH_degree + 1) * 2 - 1); // this decide the elev resolution used for updating sph_coeff
    elev_step = round(elev_step);
    Eigen::VectorXd std_azi_idx = curl::range<double>(0, SH_table_config.azi_rso, azi_step);
    Eigen::VectorXd std_elev_idx = curl::range<double>(0, SH_table_config.elev_rso, elev_step);
    std_idx.resize(std_azi_idx.size() * std_elev_idx.size());
    for (int i = 0; i < std_azi_idx.size(); ++i) {
        std_idx(Eigen::seq(i * std_elev_idx.size(), (i + 1) * std_elev_idx.size() - 1)) =
            (std_azi_idx(i) * SH_table_config.elev_rso + std_elev_idx.array()).cast<int>();
    }
    // get data
    odometry.resize(file_loader.odometry.rows());
    odometry_noised.resize(file_loader.odometry_noised.rows());
    trajectory.resize(file_loader.trajectory.rows());
    trajectory_noised.resize(file_loader.trajectory_noised.rows());
    R_w_j_vec.resize(file_loader.trajectory_noised.rows());
    t_w_j_vec.resize(file_loader.trajectory_noised.rows());
    T_w_j_vec.resize(file_loader.trajectory_noised.rows());
    std::vector<int> obj_idx;
    std::shared_ptr<SH_TABLE_CONFIG<double>> SH_table_config_ptr =
        std::make_shared<SH_TABLE_CONFIG<double>>(SH_table_config);
    std::shared_ptr<DIRECT_METHOD_CONFIG> direct_method_config_ptr =
        std::make_shared<DIRECT_METHOD_CONFIG>(direct_method_config);
    for (int i = 0; i < file_loader.trajectory.rows(); ++i) {
        Eigen::MatrixXd matrix_tmp;
        if (i < file_loader.odometry.rows()) {
            // odometry
            matrix_tmp = file_loader.odometry.row(i);
            odometry[i].matrix() = Eigen::Map<Eigen::Matrix<double, 4, 4>>(matrix_tmp.data(), 4, 4);
            //            std::cout << odometry[i].matrix() << '\n' << std::endl;
            // odometry noised
            matrix_tmp = file_loader.odometry_noised.row(i);
            odometry_noised[i].matrix() = Eigen::Map<Eigen::Matrix<double, 4, 4>>(matrix_tmp.data(), 4, 4);
            //            std::cout << odometry_noised[i].matrix() << '\n' << std::endl;
        }
        // trajectory
        matrix_tmp = file_loader.trajectory.row(i);
        trajectory[i].matrix() = Eigen::Map<Eigen::Matrix<double, 4, 4>>(matrix_tmp.data(), 4, 4);
        //        std::cout << trajectory[i].matrix() << '\n' << std::endl;
        // trajectory noised
        if (local_BA_config.is_ground_truth_initial) {
            matrix_tmp = file_loader.trajectory.row(i);
        } else {
            matrix_tmp = file_loader.trajectory_noised.row(i);
        }
        trajectory_noised[i].matrix() = Eigen::Map<Eigen::Matrix<double, 4, 4>>(matrix_tmp.data(), 4, 4);
        // Give noised trajectory / ground truth trajectory
        Eigen::Matrix3d R_tmp = trajectory_noised[i].matrix()(Eigen::seq(0, 2), Eigen::seq(0, 2));
        trajectory_noised[i].matrix()(Eigen::seq(0, 2), Eigen::seq(0, 2)) =
            Eigen::Quaterniond(R_tmp).normalized().toRotationMatrix();
        R_w_j_vec[i] = (trajectory_noised[i].rotation());
        t_w_j_vec[i] = (trajectory_noised[i].translation());
        T_w_j_vec[i] = trajectory_noised[i].matrix();
        //        std::cout << trajectory_noised[i].matrix() << '\n' << std::endl;
        R_w_j_init_vec = R_w_j_vec;
        t_w_j_init_vec = t_w_j_vec;
    }
    int counter = 0;
    while (!file_loader.scans_queue.empty()) {
        // save points to vector
        DynamicMatrix scan = file_loader.scans_queue.front();
        scans_vec.push_back(scan);
        file_loader.scans_queue.pop();
        // save semantics to vector
        DynamicMatrixSem semantic = file_loader.semantics_queue.front();
        semantics_vec.push_back(semantic);
        file_loader.semantics_queue.pop();
        // save conformal map value to vector
        std::vector<DynamicMatrix> conformal_map_tmp = file_loader.conformal_map_queue.front();
        conformal_map_vec.push_back(conformal_map_tmp);
        file_loader.conformal_map_queue.pop();
        // save gamma gradients to vector
        m_x_Gx.push_back(file_loader.m_x_Gx_queue.front());
        file_loader.m_x_Gx_queue.pop();
        m_x_Gy.push_back(file_loader.m_x_Gy_queue.front());
        file_loader.m_x_Gy_queue.pop();
        m_y_Gx.push_back(file_loader.m_y_Gx_queue.front());
        file_loader.m_y_Gx_queue.pop();
        m_y_Gy.push_back(file_loader.m_y_Gy_queue.front());
        file_loader.m_y_Gy_queue.pop();
        // save SPH coefficients to a vector
        std::vector<DynamicMatrix> sph_coeff_tmp = file_loader.sph_coeff_queue.front();
        sph_coeff_vec.push_back(sph_coeff_tmp);
        sph_coeff_vec_init.push_back(sph_coeff_tmp);
        file_loader.sph_coeff_queue.pop();
        // save scans_in_lidar T_obj_lidar and interpolation function to vectors
        std::vector<Eigen::Isometry3d> T_obj_LiDAR_Iso(obj_num);
        std::vector<DynamicMatrix> T_obj_LiDAR_tmp = file_loader.T_obj_LiDAR_queue.front();
        file_loader.T_obj_LiDAR_queue.pop();
        std::vector<DynamicMatrix> scans_obj_lidar_tmp(obj_num);
        std::vector<std::pair<interp_func_pair, interp_func_pair>> interp_conformal_tmp(obj_num);
        std::vector<std::pair<interp_func_pair, interp_func_pair>> interp_inv_conformal_tmp(obj_num);
        std::vector<PatchProcession<double>> patch_procession_tmp;
        //        patch_procession_tmp.reserve(obj_num);
        scan.conservativeResize(scan.rows(), scan.cols() + 1);
        scan.col(scan.cols() - 1) = Eigen::MatrixXd::Ones(scan.rows(), 1);
        // valid_idx tmp vector
        std::vector<DynamicMatrixSem> valid_idx_tmp = file_loader.valid_idx_queue.front();
        file_loader.valid_idx_queue.pop();
        for (int i = 0; i < obj_num; ++i) {
            // save T_obj_lidar to vector
            T_obj_LiDAR_Iso[i].matrix() = Eigen::Map<Eigen::Matrix<double, 4, 4>>(T_obj_LiDAR_tmp[i].data(), 4, 4);
            // save interpolation function for each object here
            obj_idx.clear();
            for (int j = 0; j < semantic.rows(); ++j) {
                if (semantic(j, 0) == i + 1) {
                    obj_idx.push_back(j);
                }
            }
            if (valid_idx_tmp[i].size() > 0) {
                Eigen::Map<Eigen::VectorX<SemanticType>> valid_idx(valid_idx_tmp[i].data(), valid_idx_tmp[i].size());
                scans_obj_lidar_tmp[i] = scan(obj_idx, Eigen::seq(0, 2))(valid_idx.template cast<int>(), Eigen::all);
            } else {
                scans_obj_lidar_tmp[i] = scan(obj_idx, Eigen::seq(0, 2));
            }
            Eigen::MatrixXd scan_obj_homo = (T_obj_LiDAR_Iso[i].matrix() * scan.transpose()).transpose();
            Eigen::MatrixXd scan_in_obj;
            if (valid_idx_tmp[i].size() > 0) {
                Eigen::Map<Eigen::VectorX<SemanticType>> valid_idx(valid_idx_tmp[i].data(), valid_idx_tmp[i].size());
                scan_in_obj = scan_obj_homo(obj_idx, Eigen::seq(0, 2))(valid_idx.template cast<int>(), Eigen::all);
            } else {
                scan_in_obj = scan_obj_homo(obj_idx, Eigen::seq(0, 2));
            }
            Eigen::VectorXd map_0 = conformal_map_tmp[i](Eigen::all, 0);
            Eigen::VectorXd map_1 = conformal_map_tmp[i](Eigen::all, 1);
            interp_func_pair F0 = linear_interpolation_2_func(scan_in_obj, map_0);
            interp_func_pair F1 = linear_interpolation_2_func(scan_in_obj, map_1);
            interp_conformal_tmp[i] = std::make_pair(F0, F1);
            // interp 2d_plane, from conforal to object
            interp_func_pair F2 = linear_interpolation_2_func(conformal_map_tmp[i], scan_in_obj.col(0));
            interp_func_pair F3 = linear_interpolation_2_func(conformal_map_tmp[i], scan_in_obj.col(1));
            interp_inv_conformal_tmp[i] = std::make_pair(F2, F3);
            interp_func_pair Fz = linear_interpolation_2_func(conformal_map_tmp[i], scan_in_obj.col(2));
            if (patch_procession.empty()) {
                patch_procession_tmp.emplace_back(
                    direct_method_config_ptr, SH_table_config_ptr, debug_config_ptr, interp_conformal_tmp[i],
                    interp_inv_conformal_tmp[i], Fz, scan_in_obj, map_0, map_1,
                    Eigen::Map<Eigen::VectorXd>(sph_coeff_tmp[i].data(), sph_coeff_tmp[i].size()));
                double x = patch_procession_tmp.back().fetch_Gamma(patch_procession_tmp.back().Gamma_x,
                                                                   scan_in_obj(100, 0), scan_in_obj(100, 1));
                double y = patch_procession_tmp.back().fetch_Gamma(patch_procession_tmp.back().Gamma_y,
                                                                   scan_in_obj(100, 0), scan_in_obj(100, 1));
                double intensity = patch_procession_tmp.back().fetch_I(patch_procession_tmp.back().I, x, y);
                //                double x =
                //                patch_procession_tmp.back().fetch_Gamma_x_interp(scan_in_obj(100,
                //                0),
                //                                                                            scan_in_obj(100,
                //                                                                            1));
                //                double y =
                //                patch_procession_tmp.back().fetch_Gamma_y_interp(scan_in_obj(100,
                //                0),
                //                                                                            scan_in_obj(100,
                //                                                                            1));
                //                double intensity = patch_procession_tmp.back().fetch_I_interp(x,
                //                y);
                std::cout << "map0: " << map_0(100) << " Gamma_x: " << x << std::endl;
                std::cout << "map1: " << map_1(100) << " Gamma_y: " << y << std::endl;
                std::cout << "Intensity: " << intensity << std::endl;
                std::cout << "Goal: " << scan_in_obj(100, 2) << std::endl;
            }

            if (local_BA_config.is_debug) {
                if (counter == 0) {
                    // save interpolation seeds
                    FileReaderBase::write_txt_file("/media/zkc/Extreme/omniverse_issac/lidar_simulator/"
                                                   "patch_00/experiment/projection_2d/seeds_obj_" +
                                                       std::to_string(i + 1) + ".txt",
                                                   scan_in_obj);
                }
            }
        }
        T_obj_lidar_vec.push_back(T_obj_LiDAR_Iso);
        interp_conformal_vec.push_back(interp_conformal_tmp);
        interp_inv_conformal_vec.push_back(interp_inv_conformal_tmp);
        if (patch_procession.empty()) {
            patch_procession.push_back(patch_procession_tmp);
        }
        scans_in_lidar.push_back(scans_obj_lidar_tmp);
        ++counter;
    }
    start_frame = file_loader.start_frame;
    end_frame = file_loader.end_frame;
    for (int i = 0; i < obj_num; ++i) {
        R_o_w_vec.emplace_back(T_obj_lidar_vec[0][i].rotation() *
                               trajectory_noised[start_frame - 1].rotation().transpose());
        t_o_w_vec.emplace_back(T_obj_lidar_vec[0][i].translation() -
                               T_obj_lidar_vec[0][i].rotation() *
                                   trajectory_noised[start_frame - 1].rotation().transpose() *
                                   trajectory_noised[start_frame - 1].translation());
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_o_w_tmp;
        T_o_w_tmp.setIdentity();
        T_o_w_tmp(Eigen::seq(0, 2), Eigen::seq(0, 2)) =
            Eigen::Quaterniond(R_o_w_vec.back()).normalized().toRotationMatrix();
        T_o_w_tmp(Eigen::seq(0, 2), 3) = t_o_w_vec.back();
        T_o_w_vec.push_back(T_o_w_tmp);
    }
    residual_sum_pair.resize(obj_num,
                             std::make_pair<Eigen::VectorXd, int>(
                                 Eigen::VectorXd::Zero(SH_table_config.azi_rso * SH_table_config.elev_rso), 0));
    processing();
}

template <typename BasicType, typename SemanticType>
void CURL_local_BA<BasicType, SemanticType>::CURL_local_BA::save_optimized_pose(const std::string &filename,
                                                                                std::vector<Eigen::Matrix3d> &R_vec,
                                                                                std::vector<Eigen::Vector3d> &t_vec) {
    Eigen::MatrixXd SE3_table(R_vec.size(), 16);
    SE3_table.setZero();
    for (int i = 0; i < R_vec.size(); ++i) {
        int count = 0;
        for (int j = 0; j < 12; ++j) {
            if ((j + 1) % 4 != 0) {
                SE3_table(i, j) = *(double *)(R_vec[i].data() + count);
                ++count;
            }
        }
        SE3_table(i, Eigen::seq(12, 14)) = t_vec[i];
        SE3_table(i, 15) = 1;
    }
    FileReaderBase::write_txt_file(filename, SE3_table);
}

template <typename BasicType, typename SemanticType>
void CURL_local_BA<BasicType, SemanticType>::CURL_local_BA::save_optimized_sph_coeff(
    const std::string &dir, std::vector<DynamicMatrix> sph_coeff) {
    for (int i = 0; i < sph_coeff.size(); ++i) {
        std::string filename = dir + "/sph_coeff/obj_" + std::to_string(i + 1) + ".txt";
        FileReaderBase::write_txt_file(filename, sph_coeff[i]);
    }
}

template <typename BasicType, typename SemanticType> void CURL_local_BA<BasicType, SemanticType>::BA() {
    Timer timer_direct_method;
    double direct_time = 0;
    Timer timer;
    timer.start();
    for (int num = 0; num < local_BA_config.iter_num; ++num) {
        ceres::Problem problem;
        ceres::Manifold *SO3_Manifold = SO3Manifold::Create();
        ceres::Manifold *SE3_manifold = SE3Manifold::Create();
        //        ceres::LossFunctionWrapper *loss_function =
        //            new ceres::LossFunctionWrapper(new ceres::HuberLoss(0.01), ceres::TAKE_OWNERSHIP);

        int counter_j_1 = 0;
        // for visualization
        points_j_1_valid_vec_tmp.resize(obj_num);
        points_j_1_transformed_recons_vec_tmp.resize(obj_num);
        overlap_idx_tmp.resize(obj_num);
        points_obj_recons_reference.resize(obj_num);
        points_obj_recons_updated.resize(obj_num);
        points_obj_tmp.resize(obj_num);
        points_j_valid_vec.clear();
        points_j_transformed_recons_vec.clear();
        overlap_idx.clear();
        points_obj.clear();
        for (int j = 0; j < obj_num; ++j) {
            residual_sum_pair[j].second = 0;
        }
        for (int i = start_frame; i < end_frame; ++i) {
            //            R_w_j_vec[i - 1] = Eigen::Quaterniond(R_w_j_vec[i -
            //            1]).normalized().toRotationMatrix(); R_w_j_vec[i] =
            //            Eigen::Quaterniond(R_w_j_vec[i]).normalized().toRotationMatrix();
            auto *R_w_j_1 = static_cast<double *>(R_w_j_vec[i - 1].data());
            auto *t_w_j_1 = static_cast<double *>(t_w_j_vec[i - 1].data());
            auto *R_w_j = static_cast<double *>(R_w_j_vec[i].data());
            auto *t_w_j = static_cast<double *>(t_w_j_vec[i].data());
            auto *T_w_j_1 = static_cast<double *>(T_w_j_vec[i - 1].data());
            if (local_BA_config.is_odometry && i < end_frame) { // optimize trajectory based on odometry
                ceres::CostFunction *odometry_cost_func = OdometryFactor::Create(odometry_noised[i - 1]);
                problem.AddResidualBlock(odometry_cost_func, nullptr, R_w_j, t_w_j, R_w_j_1, t_w_j_1);
                problem.SetManifold(R_w_j, SO3_Manifold);
                problem.SetManifold(R_w_j_1, SO3_Manifold);
            }
            if (local_BA_config.is_sph_fix_point_to_map) {
                for (int j = 0; j < obj_num; ++j) {
                    R_o_w_vec[j] = Eigen::Quaterniond(R_o_w_vec[j]).normalized().toRotationMatrix();
                    auto *R_o_w = static_cast<double *>(R_o_w_vec[j].data());
                    auto *t_o_w = static_cast<double *>(t_o_w_vec[j].data());
                    Eigen::Vector2d conformal_map_boundary;
                    conformal_map_boundary << conformal_map_vec[0][j].col(0).maxCoeff(),
                        conformal_map_vec[0][j].col(1).maxCoeff();
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1 = scans_in_lidar[counter_j_1][j].transpose();
                    Eigen::Map<Eigen::VectorXd> sph_coeff(sph_coeff_vec[0][j].data(), sph_coeff_vec[0][j].size());
                    Eigen::MatrixXd points_j_1_transformed = ((R_o_w_vec[j] * R_w_j_vec[i - 1] * points_j_1).colwise() +
                                                              (t_o_w_vec[j] + R_o_w_vec[j] * t_w_j_vec[i - 1]))
                                                                 .transpose();
                    double conformal_boundary_x[2] = {0, conformal_map_boundary(0)};
                    double conformal_boundary_y[2] = {0, conformal_map_boundary(1)};
                    std::pair<std::vector<double>, std::vector<int>> x0 = linear_interpolation_2(
                        points_j_1_transformed, interp_conformal_vec[0][j].first, conformal_boundary_x);
                    std::pair<std::vector<double>, std::vector<int>> x1 = linear_interpolation_2(
                        points_j_1_transformed, interp_conformal_vec[0][j].second, conformal_boundary_y, x0.first);
                    std::pair<std::vector<double>, std::vector<int>> x_2d;
                    std::pair<std::vector<double>, std::vector<int>> y_2d;
                    std::vector<int> &idx_vec = y_2d.second;
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_transformed_recons;
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_valid;
                    std::cout << "Iter: " << num + 1 << " Frame: " << i - start_frame << " Obj: " << j + 1 << std::endl;
                    DirectMethod direct_method(direct_method_config, &SH_table_config, conformal_map_boundary);
                    if (local_BA_config.is_direct_method) {
                        /****************** Test height Direct method ***************************/
                        timer_direct_method.start();
                        bool is_valid =
                            direct_method.height_img_generation(sph_coeff, x0, x1, points_j_1_transformed.col(2));
                        bool is_update_sph_coeff = false;
                        if (is_valid) {
                            if (direct_method_config.is_pyramid_track) {
                                is_update_sph_coeff = direct_method.pyramid_track(sph_coeff, SH_table_config.SH_scale);
                            } else {
                                is_update_sph_coeff =
                                    direct_method.track(sph_coeff, direct_method.img_interp_valid_mat,
                                                        direct_method.img_interp_valid_indices, false);
                            }
                        } else {
                            break;
                        }
                        if (direct_method_config.is_update_sph_coeff && is_update_sph_coeff) {
                            Timer update_coeff_timer;
                            update_coeff_timer.start();
                            //                            sph_coeff =
                            //                            direct_method.update_sph_coeff(
                            //                                sph_coeff, x0, x1,
                            //                                points_j_1_transformed.col(2));
                            direct_method.update_sph_coeff_residual(sph_coeff, x0, x1, points_j_1_transformed.col(2),
                                                                    residual_sum_pair[j]);
                            std::cout << "update_sph_coeff: " << update_coeff_timer.elapsedMilliseconds() << " ms"
                                      << std::endl;
                        }

                        Eigen::MatrixXd sph_dirs;
                        Eigen::MatrixXd transformed_conformal_coord;
                        // ToDo: update sph_coeff in each large optimization (Using SH_table_grid to
                        // store the residual)
                        direct_method.update_points(sph_coeff, x0, x1, interp_inv_conformal_vec[0][j],
                                                    points_j_1_transformed, x_2d, y_2d, points_j_1_transformed_recons);
                        double direct_time_current = timer_direct_method.elapsedMilliseconds();
                        std::cout << "Image warping: " << direct_time_current << " ms" << std::endl;
                        direct_time += direct_time_current;
                        points_j_1_valid = points_j_1(Eigen::all, idx_vec);
                    } else {
                        /****************** Old method ***************************/
                        idx_vec = x1.second;
                        Eigen::VectorXd phis_tmp =
                            Eigen::Map<Eigen::VectorXd>(x0.first.data(), static_cast<long>(x0.first.size()));
                        Eigen::VectorXd thetas_tmp =
                            Eigen::Map<Eigen::VectorXd>(x1.first.data(), static_cast<long>(x1.first.size()));
                        Eigen::VectorXd phis =
                            ((phis_tmp / conformal_map_boundary(0) * 2 * M_PI * SH_table_config.SH_scale).array() +
                             M_PI * (1 - SH_table_config.SH_scale))(idx_vec);
                        Eigen::VectorXd thetas =
                            ((thetas_tmp / conformal_map_boundary(1) * M_PI * SH_table_config.SH_scale).array() +
                             M_PI_2 * (1 - SH_table_config.SH_scale))(idx_vec);
                        Eigen::VectorXd z_recons;
                        if (!idx_vec.empty()) {
                            Eigen::MatrixXd sph_dirs(thetas.size(), 2);
                            sph_dirs << phis, thetas;
                            z_recons =
                                curl::invLeastSquaresSHT<double>(sph_coeff, sph_dirs, SH_table_config.max_SH_degree);
                        }
                        points_j_1_valid = points_j_1(Eigen::all, idx_vec);
                        points_j_1_transformed_recons = points_j_1_transformed(idx_vec, Eigen::all).transpose();
                        points_j_1_transformed_recons.row(2) = z_recons.transpose();
                    }
                    /************************************************************************/
                    // points_j_1_valid, points_j_1_transformed and
                    // points_j_transformed_recons_vec are for visualization
                    points_j_1_valid_vec_tmp[j] = DynamicMatrix(points_j_1.transpose());
                    points_j_1_transformed_recons_vec_tmp[j] = DynamicMatrix(points_j_1_transformed_recons.transpose());
                    overlap_idx_tmp[j] = idx_vec;
                    if (i == start_frame && num == 0) {
                        points_obj_recons_reference[j] = points_j_1_transformed_recons.transpose();
                    }
                    if (i == start_frame && num != 0) {
                        points_obj_recons_updated[j] = points_j_1_transformed_recons.transpose();
                    }
                    points_obj_tmp[j] = points_j_1_transformed;
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_valid_transformed;
                    if (local_BA_config.is_pure_ICP) {
                        points_j_1_valid_transformed = points_j_1_transformed.transpose();
                    } else {
                        points_j_1_valid_transformed = points_j_1_transformed(idx_vec, Eigen::all).transpose();
                    }
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
                    int result_size = 1;
                    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_pcl;
                    std::vector<int> pointIdxNKNSearch(result_size);
                    std::vector<float> pointNKNSquaredDistance(result_size);
                    if (local_BA_config.is_nn_data_association) {
                        if (local_BA_config.is_pure_ICP) {
                            points_j_1_transformed_recons =
                                (R_o_w_vec[j] * R_w_j_vec[start_frame - 1] * scans_in_lidar[0][j].transpose())
                                    .colwise() +
                                (t_o_w_vec[j] + R_o_w_vec[j] * t_w_j_vec[start_frame - 1]);
                        }
                        curl::convert_to_point_cloud(Eigen::MatrixXd(points_j_1_transformed_recons.transpose()),
                                                     cloud_ptr);
                        kdtree_pcl.setInputCloud(cloud_ptr);
                    }
                    // kd tree to find the nearst neighbour data association

                    if (local_BA_config.is_debug) {
                        std::string root_file_projection_2d = "/media/zkc/Extreme/omniverse_issac/"
                                                              "lidar_simulator/patch_00/experiment/projection_2d";
                        FileReaderBase::write_txt_file(root_file_projection_2d + "/input_" + std::to_string(i - 1000) +
                                                           "_" + std::to_string(j + 1) + ".txt",
                                                       points_j_1_transformed);
                        FileReaderBase::write_vector_txt_file(root_file_projection_2d + "/c++_result_x0_" +
                                                                  std::to_string(i - 1000) + "_" +
                                                                  std::to_string(j + 1) + ".txt",
                                                              x0.first);
                        FileReaderBase::write_vector_txt_file(root_file_projection_2d + "/c++_result_x1_" +
                                                                  std::to_string(i - 1000) + "_" +
                                                                  std::to_string(j + 1) + ".txt",
                                                              x1.first);
                        // height image
                        std::string root_file_height_image =
                            "/media/zkc/Extreme/omniverse_issac/lidar_simulator/patch_00/"
                            "experiment/height_image";
                        FileReaderBase::write_txt_file(root_file_height_image + "/img_recons_" +
                                                           std::to_string(i - 1000) + "_" + std::to_string(j + 1) +
                                                           ".txt",
                                                       direct_method.img_recons);
                        FileReaderBase::write_txt_file(root_file_height_image + "/img_interp_" +
                                                           std::to_string(i - 1000) + "_" + std::to_string(j + 1) +
                                                           ".txt",
                                                       direct_method.img_interp);
                        FileReaderBase::write_txt_file(root_file_height_image + "/boundary_recons_" +
                                                           std::to_string(i - 1000) + "_" + std::to_string(j + 1) +
                                                           ".txt",
                                                       Eigen::MatrixXd(direct_method.boundary_of_recons));
                        FileReaderBase::write_txt_file(root_file_height_image + "/boundary_interp_" +
                                                           std::to_string(i - 1000) + "_" + std::to_string(j + 1) +
                                                           ".txt",
                                                       Eigen::MatrixXd(direct_method.boundary_of_interp));
                        FileReaderBase::write_txt_file(
                            root_file_height_image + "/initial_displace_" + std::to_string(i - 1000) + "_" +
                                std::to_string(j + 1) + ".txt",
                            Eigen::MatrixXd(direct_method.pixel_interp_valid_displace.cast<double>()));

                        //
                        FileReaderBase::write_txt_file(root_file_height_image + "/img_valid_" +
                                                           std::to_string(i - 1000) + "_" + std::to_string(j + 1) +
                                                           ".txt",
                                                       direct_method.img_interp_valid);
                    }
                    if (local_BA_config.is_obj_fix) {
                        //                        for (int k = 0; k < idx_vec.size(); ++k) {
                        for (int k = 0; k < points_j_1_valid_transformed.cols(); ++k) {
                            Eigen::Vector3d p_j_1;
                            if (local_BA_config.is_pure_ICP) {
                                p_j_1 = points_j_1.col(k);
                            } else {
                                p_j_1 = points_j_1_valid.col(k);
                            }

                            Eigen::Vector3d p_j_1_transformed = points_j_1_valid_transformed.col(k);
                            Eigen::Vector3d p_o;
                            if (local_BA_config.is_nn_data_association) {
                                pcl::PointXYZ search_point;
                                search_point.getVector3fMap() = p_j_1_transformed.cast<float>();
                                kdtree_pcl.nearestKSearch(search_point, result_size, pointIdxNKNSearch,
                                                          pointNKNSquaredDistance);
                                p_o = points_j_1_transformed_recons.col(pointIdxNKNSearch[0]);
                            } else {
                                p_o = points_j_1_transformed_recons.col(k);
                            }

                            ceres::CostFunction *sph_fix_point_to_map_fix_cost_func =
                                SphFixPointToMapFixFactor::Create(R_o_w_vec[j], t_o_w_vec[j], p_j_1, p_o);
                            problem.AddResidualBlock(sph_fix_point_to_map_fix_cost_func, nullptr, R_w_j_1, t_w_j_1);
                            problem.SetManifold(R_w_j_1, SO3_Manifold);
                        }
                        if (local_BA_config.is_set_constant) {
                            if (i == start_frame) {
                                problem.SetParameterBlockConstant(R_w_j_1);
                                problem.SetParameterBlockConstant(t_w_j_1);
                            }
                        }
                    } else {
                        for (int k = 0; k < idx_vec.size(); ++k) {
                            Eigen::Vector3d p_j = points_j_1_valid.col(k);
                            Eigen::Vector3d p_o = points_j_1_transformed_recons.col(k);
                            ceres::CostFunction *sph_fix_point_to_map_cost_func =
                                SphFixPointToMapFactor::Create(p_j, p_o);
                            problem.AddResidualBlock(sph_fix_point_to_map_cost_func, nullptr, R_w_j_1, t_w_j_1, R_o_w,
                                                     t_o_w);
                            problem.SetManifold(R_w_j_1, SO3_Manifold);
                            problem.SetManifold(R_o_w, SO3_Manifold);
                        }
                        if (local_BA_config.is_set_constant) {
                            if (i == start_frame) {
                                problem.SetParameterBlockConstant(R_w_j_1);
                                problem.SetParameterBlockConstant(t_w_j_1);
                            }
                        }
                    }
                }
            }

            if (local_BA_config.is_couple) {
                for (int j = 0; j < obj_num; ++j) {
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1 = scans_in_lidar[counter_j_1][j].transpose();
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_valid;
                    Eigen::Map<Eigen::VectorXd> sph_coeff(sph_coeff_vec[0][j].data(), sph_coeff_vec[0][j].size());
                    Eigen::Vector2d conformal_map_boundary;
                    conformal_map_boundary << conformal_map_vec[0][j].col(0).maxCoeff(),
                        conformal_map_vec[0][j].col(1).maxCoeff();
                    double conformal_boundary_x[2] = {0, conformal_map_boundary(0)};
                    double conformal_boundary_y[2] = {0, conformal_map_boundary(1)};
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_transformed_recons;
                    Eigen::MatrixXd points_j_1_transformed = ((R_o_w_vec[j] * R_w_j_vec[i - 1] * points_j_1).colwise() +
                                                              (t_o_w_vec[j] + R_o_w_vec[j] * t_w_j_vec[i - 1]))
                                                                 .transpose();
                    std::pair<std::vector<double>, std::vector<int>> x0 = linear_interpolation_2(
                        points_j_1_transformed, interp_conformal_vec[0][j].first, conformal_boundary_x);
                    std::pair<std::vector<double>, std::vector<int>> x1 = linear_interpolation_2(
                        points_j_1_transformed, interp_conformal_vec[0][j].second, conformal_boundary_y, x0.first);
                    std::pair<std::vector<double>, std::vector<int>> x_2d;
                    std::pair<std::vector<double>, std::vector<int>> y_2d;
                    std::vector<int> &idx_vec = y_2d.second;
                    idx_vec = x1.second;
                    Eigen::VectorXd phis_tmp =
                        Eigen::Map<Eigen::VectorXd>(x0.first.data(), static_cast<long>(x0.first.size()));
                    Eigen::VectorXd thetas_tmp =
                        Eigen::Map<Eigen::VectorXd>(x1.first.data(), static_cast<long>(x1.first.size()));
                    Eigen::VectorXd phis =
                        ((phis_tmp / conformal_map_boundary(0) * 2 * M_PI * SH_table_config.SH_scale).array() +
                         M_PI * (1 - SH_table_config.SH_scale))(idx_vec);
                    Eigen::VectorXd thetas =
                        ((thetas_tmp / conformal_map_boundary(1) * M_PI * SH_table_config.SH_scale).array() +
                         M_PI_2 * (1 - SH_table_config.SH_scale))(idx_vec);
                    Eigen::VectorXd z_recons;
                    if (!idx_vec.empty()) {
                        Eigen::MatrixXd sph_dirs(thetas.size(), 2);
                        sph_dirs << phis, thetas;
                        z_recons = curl::invLeastSquaresSHT<double>(sph_coeff, sph_dirs, SH_table_config.max_SH_degree);
                    }
                    points_j_1_valid = points_j_1(Eigen::all, idx_vec);
                    points_j_1_transformed_recons = points_j_1_transformed(idx_vec, Eigen::all).transpose();
                    points_j_1_transformed_recons.row(2) = z_recons.transpose();
                    points_j_1_valid_vec_tmp[j] = DynamicMatrix(points_j_1.transpose());
                    points_j_1_transformed_recons_vec_tmp[j] = DynamicMatrix(points_j_1_transformed_recons.transpose());
                    overlap_idx_tmp[j] = idx_vec;
                    if (i == start_frame && num != 0) {
                        Eigen::VectorXd z_init;
                        if (!idx_vec.empty()) {
                            Eigen::Map<Eigen::VectorXd> sph_coeff_init(sph_coeff_vec_init[0][j].data(),
                                                                       sph_coeff_vec_init[0][j].size());
                            Eigen::MatrixXd sph_dirs(thetas.size(), 2);
                            sph_dirs << phis, thetas;
                            z_init = curl::invLeastSquaresSHT<double>(sph_coeff_init, sph_dirs,
                                                                      SH_table_config.max_SH_degree);
                        }
                        Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_transformed_recons_init =
                            points_j_1_transformed_recons;
                        points_j_1_transformed_recons_init.row(2) = z_init.transpose();
                        points_obj_recons_reference[j] = points_j_1_transformed_recons_init.transpose();
                        points_obj_recons_updated[j] = points_j_1_transformed_recons.transpose();
                    }
                    points_obj_tmp[j] = points_j_1_transformed;
                    if (num == 0) {
                        for (int k = 0; k < points_j_1.cols(); ++k) {
                            Eigen::Vector3d pts = points_j_1.col(k);
                            ceres::CostFunction *couple_fix_sph_cost_func =
                                CoupleFixSphFactor::Create(T_o_w_vec[j], pts, &patch_procession[0][j]);
                            problem.AddResidualBlock(couple_fix_sph_cost_func, nullptr, T_w_j_1);
                            problem.SetManifold(T_w_j_1, SE3_manifold);
                        }
                    }
                }
                if (num == 0) {
                    if (local_BA_config.is_set_constant) {
                        if (i == start_frame) {
                            problem.SetParameterBlockConstant(T_w_j_1);
                            std::cout << T_w_j_vec[start_frame - 1] << std::endl;
                        }
                    }
                }
            }

            if (local_BA_config.is_couple_sph) {
                for (int j = 0; j < obj_num; ++j) {
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1 = scans_in_lidar[counter_j_1][j].transpose();
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_valid;
                    Eigen::Map<Eigen::VectorXd> sph_coeff(sph_coeff_vec[0][j].data(), sph_coeff_vec[0][j].size());
                    Eigen::Vector2d conformal_map_boundary;
                    conformal_map_boundary << conformal_map_vec[0][j].col(0).maxCoeff(),
                        conformal_map_vec[0][j].col(1).maxCoeff();
                    double conformal_boundary_x[2] = {0, conformal_map_boundary(0)};
                    double conformal_boundary_y[2] = {0, conformal_map_boundary(1)};
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_transformed_recons;
                    Eigen::MatrixXd points_j_1_transformed = ((R_o_w_vec[j] * R_w_j_vec[i - 1] * points_j_1).colwise() +
                                                              (t_o_w_vec[j] + R_o_w_vec[j] * t_w_j_vec[i - 1]))
                                                                 .transpose();
                    std::pair<std::vector<double>, std::vector<int>> x0 = linear_interpolation_2(
                        points_j_1_transformed, interp_conformal_vec[0][j].first, conformal_boundary_x);
                    std::pair<std::vector<double>, std::vector<int>> x1 = linear_interpolation_2(
                        points_j_1_transformed, interp_conformal_vec[0][j].second, conformal_boundary_y, x0.first);
                    std::pair<std::vector<double>, std::vector<int>> x_2d;
                    std::pair<std::vector<double>, std::vector<int>> y_2d;
                    std::vector<int> &idx_vec = y_2d.second;
                    idx_vec = x1.second;
                    Eigen::VectorXd phis_tmp =
                        Eigen::Map<Eigen::VectorXd>(x0.first.data(), static_cast<long>(x0.first.size()));
                    Eigen::VectorXd thetas_tmp =
                        Eigen::Map<Eigen::VectorXd>(x1.first.data(), static_cast<long>(x1.first.size()));
                    Eigen::VectorXd phis =
                        ((phis_tmp / conformal_map_boundary(0) * 2 * M_PI * SH_table_config.SH_scale).array() +
                         M_PI * (1 - SH_table_config.SH_scale))(idx_vec);
                    Eigen::VectorXd thetas =
                        ((thetas_tmp / conformal_map_boundary(1) * M_PI * SH_table_config.SH_scale).array() +
                         M_PI_2 * (1 - SH_table_config.SH_scale))(idx_vec);
                    Eigen::VectorXd z_recons;
                    if (!idx_vec.empty()) {
                        Eigen::MatrixXd sph_dirs(thetas.size(), 2);
                        sph_dirs << phis, thetas;
                        z_recons = curl::invLeastSquaresSHT<double>(sph_coeff, sph_dirs, SH_table_config.max_SH_degree);
                    }
                    points_j_1_valid = points_j_1(Eigen::all, idx_vec);
                    points_j_1_transformed_recons = points_j_1_transformed(idx_vec, Eigen::all).transpose();
                    points_j_1_transformed_recons.row(2) = z_recons.transpose();
                    points_j_1_valid_vec_tmp[j] = DynamicMatrix(points_j_1.transpose());
                    points_j_1_transformed_recons_vec_tmp[j] = DynamicMatrix(points_j_1_transformed_recons.transpose());
                    overlap_idx_tmp[j] = idx_vec;
                    if (i == start_frame && num != 0) {
                        Eigen::VectorXd z_init;
                        if (!idx_vec.empty()) {
                            Eigen::Map<Eigen::VectorXd> sph_coeff_init(sph_coeff_vec_init[0][j].data(),
                                                                       sph_coeff_vec_init[0][j].size());
                            Eigen::MatrixXd sph_dirs(thetas.size(), 2);
                            sph_dirs << phis, thetas;
                            z_init = curl::invLeastSquaresSHT<double>(sph_coeff_init, sph_dirs,
                                                                      SH_table_config.warping_SH_degree);
                        }
                        Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_transformed_recons_init =
                            points_j_1_transformed_recons;
                        points_j_1_transformed_recons_init.row(2) = z_init.transpose();
                        points_obj_recons_reference[j] = points_j_1_transformed_recons_init.transpose();
                        points_obj_recons_updated[j] = points_j_1_transformed_recons.transpose();
                    }
                    points_obj_tmp[j] = points_j_1_transformed;
                    if (num == 0) {
                        for (int k = 0; k < points_j_1.cols(); ++k) {
                            Eigen::Vector3d pts = points_j_1.col(k);
                            ceres::CostFunction *couple_sph_cost_func = CoupleSphFactor::Create(
                                T_o_w_vec[j], pts, &patch_procession[0][j], &SH_table_config, sph_coeff_vec[0][j]);
                            problem.AddResidualBlock(couple_sph_cost_func, nullptr, T_w_j_1);
                            problem.SetManifold(T_w_j_1, SE3_manifold);
                        }
                    }
                }
                if (num == 0) {
                    if (local_BA_config.is_set_constant) {
                        if (i == start_frame) {
                            problem.SetParameterBlockConstant(T_w_j_1);
                            std::cout << T_w_j_vec[start_frame - 1] << std::endl;
                        }
                    }
                }
            }
            if (local_BA_config.is_couple_sph_mod) {
                for (int j = 0; j < obj_num; ++j) {
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1 = scans_in_lidar[counter_j_1][j].transpose();
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_valid;
                    Eigen::Map<Eigen::VectorXd> sph_coeff(sph_coeff_vec[0][j].data(), sph_coeff_vec[0][j].size());
                    Eigen::Vector2d conformal_map_boundary;
                    conformal_map_boundary << conformal_map_vec[0][j].col(0).maxCoeff(),
                        conformal_map_vec[0][j].col(1).maxCoeff();
                    double conformal_boundary_x[2] = {0, conformal_map_boundary(0)};
                    double conformal_boundary_y[2] = {0, conformal_map_boundary(1)};
                    Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_transformed_recons;
                    Eigen::MatrixXd points_j_1_transformed = ((R_o_w_vec[j] * R_w_j_vec[i - 1] * points_j_1).colwise() +
                                                              (t_o_w_vec[j] + R_o_w_vec[j] * t_w_j_vec[i - 1]))
                                                                 .transpose();
                    std::pair<std::vector<double>, std::vector<int>> x0 = linear_interpolation_2(
                        points_j_1_transformed, interp_conformal_vec[0][j].first, conformal_boundary_x);
                    std::pair<std::vector<double>, std::vector<int>> x1 = linear_interpolation_2(
                        points_j_1_transformed, interp_conformal_vec[0][j].second, conformal_boundary_y, x0.first);
                    std::pair<std::vector<double>, std::vector<int>> x_2d;
                    std::pair<std::vector<double>, std::vector<int>> y_2d;
                    std::vector<int> &idx_vec = y_2d.second;
                    idx_vec = x1.second;
                    Eigen::VectorXd phis_tmp =
                        Eigen::Map<Eigen::VectorXd>(x0.first.data(), static_cast<long>(x0.first.size()));
                    Eigen::VectorXd thetas_tmp =
                        Eigen::Map<Eigen::VectorXd>(x1.first.data(), static_cast<long>(x1.first.size()));
                    Eigen::VectorXd phis =
                        ((phis_tmp / conformal_map_boundary(0) * 2 * M_PI * SH_table_config.SH_scale).array() +
                         M_PI * (1 - SH_table_config.SH_scale))(idx_vec);
                    Eigen::VectorXd thetas =
                        ((thetas_tmp / conformal_map_boundary(1) * M_PI * SH_table_config.SH_scale).array() +
                         M_PI_2 * (1 - SH_table_config.SH_scale))(idx_vec);
                    Eigen::VectorXd z_recons;
                    if (!idx_vec.empty()) {
                        Eigen::MatrixXd sph_dirs(thetas.size(), 2);
                        sph_dirs << phis, thetas;
                        z_recons =
                            curl::invLeastSquaresSHT<double>(sph_coeff, sph_dirs, SH_table_config.warping_SH_degree);
                    }
                    points_j_1_valid = points_j_1(Eigen::all, idx_vec);
                    points_j_1_transformed_recons = points_j_1_transformed(idx_vec, Eigen::all).transpose();
                    points_j_1_transformed_recons.row(2) = z_recons.transpose();
                    points_j_1_valid_vec_tmp[j] = DynamicMatrix(points_j_1.transpose());
                    points_j_1_transformed_recons_vec_tmp[j] = DynamicMatrix(points_j_1_transformed_recons.transpose());
                    overlap_idx_tmp[j] = idx_vec;
                    if (i == start_frame && num != 0) {
                        Eigen::VectorXd z_init;
                        if (!idx_vec.empty()) {
                            Eigen::Map<Eigen::VectorXd> sph_coeff_init(sph_coeff_vec_init[0][j].data(),
                                                                       sph_coeff_vec_init[0][j].size());
                            Eigen::MatrixXd sph_dirs(thetas.size(), 2);
                            sph_dirs << phis, thetas;
                            z_init = curl::invLeastSquaresSHT<double>(sph_coeff_init, sph_dirs,
                                                                      SH_table_config.warping_SH_degree);
                        }
                        Eigen::Matrix<double, 3, Eigen::Dynamic> points_j_1_transformed_recons_init =
                            points_j_1_transformed_recons;
                        points_j_1_transformed_recons_init.row(2) = z_init.transpose();
                        points_obj_recons_reference[j] = points_j_1_transformed_recons_init.transpose();
                        points_obj_recons_updated[j] = points_j_1_transformed_recons.transpose();
                    }
                    points_obj_tmp[j] = points_j_1_transformed;
                    if (num == 0) {
                        for (int k = 0; k < points_j_1.cols(); ++k) {
                            Eigen::Vector3d pts = points_j_1.col(k);
                            ceres::CostFunction *couple_sph_cost_func = CoupleSphModFactor<121>::Create(
                                T_o_w_vec[j], pts, &patch_procession[0][j], &SH_table_config);
                            problem.AddResidualBlock(couple_sph_cost_func, nullptr, T_w_j_1,
                                                     sph_coeff_vec[0][j].data());
                            problem.SetManifold(T_w_j_1, SE3_manifold);
                        }
                    }
                }
                if (num == 0) {
                    if (local_BA_config.is_set_constant) {
                        if (i == start_frame) {
                            problem.SetParameterBlockConstant(T_w_j_1);
                            std::cout << T_w_j_vec[start_frame - 1] << std::endl;
                        }
                    }
                }
            }
            points_j_valid_vec.push_back(points_j_1_valid_vec_tmp);
            points_j_transformed_recons_vec.push_back(points_j_1_transformed_recons_vec_tmp);
            overlap_idx.push_back(overlap_idx_tmp);
            points_obj.push_back(points_obj_tmp);
            ++counter_j_1;
        }
        // update sph_coeff
        if (direct_method_config.is_update_sph_coeff && (local_BA_config.is_sph_fix_point_to_map)) {
            Timer update_coeff_timer;
            update_coeff_timer.start();
            for (int j = 0; j < obj_num; ++j) {
                Eigen::Map<Eigen::VectorXd> sph_coeff(sph_coeff_vec[0][j].data(), sph_coeff_vec[0][j].size());
                if (residual_sum_pair[j].second != 0) {
                    sph_coeff = curl::IRF_least_square_table(
                        (residual_sum_pair[j].first / double(residual_sum_pair[j].second))(std_idx), std_idx,
                        SH_table_config, 1, 1, sph_coeff, SH_table_config.inv_method);
                }
            }
            std::cout << "Update sph coeff: " << update_coeff_timer.elapsedMilliseconds() << " ms" << std::endl;
        }
        if (num == 0 || local_BA_config.is_sph_fix_point_to_map) {
            ceres::Solver::Options options;
            options.max_num_iterations = local_BA_config.max_num_iterations;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = true;
            options.update_state_every_iteration = local_BA_config.update_state_every_iteration;
            //            options.num_threads = omp_get_max_threads();
            ceres::Solver::Summary summary;
            if (!local_BA_config.is_couple_switch) {
                if (local_BA_config.is_couple_sph_mod) {
                    if (local_BA_config.is_set_constant) {
                        for (int obj_id = 0; obj_id < obj_num; ++obj_id) {
                            problem.SetParameterBlockConstant(sph_coeff_vec[0][obj_id].data());
                        }
                    }
                }
                Solve(options, &problem, &summary);
                std::cout << summary.FullReport() << std::endl;
                if (local_BA_config.is_couple_sph_mod) {
                    if (local_BA_config.is_set_constant) {
                        for (int obj_id = 0; obj_id < obj_num; ++obj_id) {
                            problem.SetParameterBlockVariable(sph_coeff_vec[0][obj_id].data());
                        }
                        VisCallback callback(start_frame, end_frame, obj_num, &ros_handler, &R_w_j_vec, &t_w_j_vec,
                                             &R_o_w_vec, &t_o_w_vec, &points_j_valid_vec, &overlap_idx,
                                             &points_j_transformed_recons_vec);
                        options.callbacks.push_back(&callback);
                        //                        loss_function->Reset(new ceres::HuberLoss(0.01),
                        //                        ceres::TAKE_OWNERSHIP);
                        Solve(options, &problem, &summary);
                        std::cout << summary.FullReport() << std::endl;
                    }
                }
                std::cout << "Cost change: " << (summary.initial_cost - summary.final_cost) << std::endl;
                if ((summary.initial_cost - summary.final_cost) < local_BA_config.threshold) {
                    std::cout << "At " << num + 1 << "-th iteration Finish!" << std::endl;
                    //                break;
                }
            } else if (local_BA_config.is_couple_switch && local_BA_config.is_couple_sph_mod) {
                for (int obj_id = 0; obj_id < obj_num; ++obj_id) {
                    problem.SetParameterBlockConstant(sph_coeff_vec[0][obj_id].data());
                }
                Solve(options, &problem, &summary);
                std::cout << summary.FullReport() << std::endl;

                for (int obj_id = 0; obj_id < obj_num; ++obj_id) {
                    problem.SetParameterBlockVariable(sph_coeff_vec[0][obj_id].data());
                }
                for (int frame_id = start_frame + 1; frame_id < end_frame; ++frame_id) {
                    auto *T_w_j_1 = static_cast<double *>(T_w_j_vec[frame_id - 1].data());
                    problem.SetParameterBlockConstant(T_w_j_1);
                }
                Solve(options, &problem, &summary);

                std::cout << summary.FullReport() << std::endl;
                for (int frame_id = start_frame + 1; frame_id < end_frame; ++frame_id) {
                    auto *T_w_j_1 = static_cast<double *>(T_w_j_vec[frame_id - 1].data());
                    problem.SetParameterBlockVariable(T_w_j_1);
                }
                for (int obj_id = 0; obj_id < obj_num; ++obj_id) {
                    problem.SetParameterBlockConstant(sph_coeff_vec[0][obj_id].data());
                }
                Solve(options, &problem, &summary);
                std::cout << summary.FullReport() << std::endl;
            }
        }
        std::cout << "At " << num + 1 << " iteration" << std::endl;
        std::cout << T_w_j_vec[start_frame - 1] << std::endl;
        if (!local_BA_config.is_sph_fix_point_to_map) {
            for (int i = 0; i < T_w_j_vec.size(); ++i) {
                R_w_j_vec[i] = T_w_j_vec[i](Eigen::seq(0, 2), Eigen::seq(0, 2));
                t_w_j_vec[i] = T_w_j_vec[i](Eigen::seq(0, 2), 3);
            }
        }
        save_optimized_pose("/media/zkc/Extreme/omniverse_issac/lidar_simulator/patch_00/"
                            "experiment/trajectory_optimized.txt",
                            R_w_j_vec, t_w_j_vec);
        save_optimized_pose("/media/zkc/Extreme/omniverse_issac/lidar_simulator/patch_00/"
                            "experiment/obj_optimized.txt",
                            R_o_w_vec, t_o_w_vec);
        save_optimized_sph_coeff("/media/zkc/Extreme/omniverse_issac/lidar_simulator/patch_00/experiment",
                                 sph_coeff_vec[0]);
    }
    double total_time = timer.elapsedMilliseconds();

    std::cout << "Total direct method time: " << direct_time / 1000 << " s " << direct_time / total_time * 100
              << "% of total time" << std::endl;
    std::cout << "Total Time: " << total_time / 1000 << " s" << std::endl;
    //    if (!local_BA_config.is_couple) {
    ros::Rate r(5);
    while (ros::ok()) {
        RosHandler::plot_rviz("init", start_frame, end_frame, obj_num, &ros_handler, &R_w_j_init_vec, &t_w_j_init_vec,
                              &R_o_w_vec, &t_o_w_vec, &points_j_valid_vec, &overlap_idx,
                              &points_j_transformed_recons_vec);
        RosHandler::plot_rviz("final", start_frame, end_frame, obj_num, &ros_handler, &R_w_j_vec, &t_w_j_vec,
                              &R_o_w_vec, &t_o_w_vec, &points_j_valid_vec, &overlap_idx,
                              &points_j_transformed_recons_vec);
        RosHandler::plot_rviz_a_pair_of_a_patch("reference", start_frame, end_frame, obj_num, &ros_handler,
                                                &points_obj_recons_reference, &points_obj);
        RosHandler::plot_rviz_a_pair_of_a_patch("updated", start_frame, end_frame, obj_num, &ros_handler,
                                                &points_obj_recons_updated, &points_obj);
        r.sleep();
    }
    //    }
}

template <typename BasicType, typename SemanticType>
void CURL_local_BA<BasicType, SemanticType>::CURL_local_BA::processing() {
    BA();
}

template class CURL_local_BA<BT, ST>; // this is very important