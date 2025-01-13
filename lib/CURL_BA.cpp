//
// Created by zkc on 18/07/22.
//

#include "CurlBA.h"

namespace curl {
CurlBA::CurlBA(std::vector<Eigen::MatrixXd> _poses, std::vector<Matrix3dCol> _scans,
               std::vector<Eigen::MatrixXi> _semantics, int _num_channels, int _degree,
               int _num_objects)
    : poses(std::move(_poses)), scans(std::move(_scans)), semantics(std::move(_semantics)),
      num_channels(_num_channels), degree(_degree), num_objects(_num_objects),
      num_frames(int(_semantics.size())) {
    num_cols = int(semantics[0].size()) / num_channels;
    coefficients = new std::vector<Eigen::VectorXd>(
        num_frames * num_objects, Eigen::VectorXd::Zero((degree + 1) * (degree + 1)));
    sph_cor = new std::vector<Matrix3dCol>(num_frames, Matrix3dCol::Zero(semantics[0].size(), 3));
    sem_idx = new std::vector<std::vector<int>>(num_frames * num_objects);
    non_zero_idx = new std::vector<std::vector<int>>(num_frames);
    meshes = new std::vector<Eigen::Matrix<int, Eigen::Dynamic, 3>>(num_frames);
    face_normals = new std::vector<Matrix3dCol>(num_frames);
    vertice_normals = new std::vector<Matrix3dCol>(num_frames);
    corner_normals = new std::vector<Matrix3dCol>(num_frames);
}

CurlBA::~CurlBA() {
    delete coefficients;
    delete sph_cor;
    delete sem_idx;
}

void CurlBA::getCoefficients() {
    for (int i = 0; i < num_frames; ++i) {
        cart2sphTable(scans[i], (*sph_cor)[i]);
        for (int j = 0; j < semantics[i].size(); ++j) {
            for (int k = 0; k < num_objects; ++k) {
                if (semantics[i](j) == k) {
                    (*sem_idx)[i * num_objects + k].push_back(j);
                }
            }
        }
        for (int k = 1; k < num_objects; ++k) {
            (*coefficients)[i * num_objects + k] =
                leastSquaresSHT(degree, (*sph_cor)[i]((*sem_idx)[i * num_objects + k], 2),
                                (*sph_cor)[i]((*sem_idx)[i * num_objects + k], Eigen::seq(0, 1)));
        }
    }
}

void CurlBA::meshing() {
    std::vector<double> projections;
    double r;
    double theta;
    for (int i = 0; i < sph_cor->size(); ++i) {
        projections.clear();
        for (int j = 0; j < (*sph_cor)[i].rows(); ++j) {
            if ((*sph_cor)[i](j, 2) > 0) {
                r = std::ceil(double(j) / double(num_cols));
                theta = (j % num_cols) * 2 * M_PI / double(num_cols);
                projections.push_back(r * cos(theta));
                projections.push_back(r * sin(theta));
                (*non_zero_idx)[i].push_back(j);
            }
        }
        /// Build mesh for 2D projection
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>> V_row(
            projections.data(), int((*non_zero_idx)[i].size()), 2);
        Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::ColMajor> V_col =
            Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::ColMajor>::Zero(V_row.rows(), 2);
        V_col.col(0) = V_row.col(0);
        V_col.col(1) = V_row.col(1);
        cgal::delaunay_triangulation<Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::ColMajor>,
                                     Eigen::Matrix<int, Eigen::Dynamic, 3>>(V_col, (*meshes)[i]);
        Eigen::MatrixXi convex_hull_mesh;
        cgal::convex_hull_2D_mesh<Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::ColMajor>,
                                  Eigen::MatrixXi>(V_col, convex_hull_mesh);
        int k = int((*meshes)[i].rows());
        (*meshes)[i].conservativeResize((*meshes)[i].rows() + convex_hull_mesh.rows(),
                                        3); // keep original memory
        (*meshes)[i](Eigen::seq(k, (*meshes)[i].rows() - 1), Eigen::all) = convex_hull_mesh;
        /// Calculate normals cost lot of computation resources, maybe use GPU or multi-threads or
        /// simpler way to do the calculation Try CGAL when having time, mesh simplify
        igl::per_face_normals(scans[i]((*non_zero_idx)[i], Eigen::all), (*meshes)[i],
                              (*face_normals)[i]);
        igl::per_vertex_normals(scans[i]((*non_zero_idx)[i], Eigen::all), (*meshes)[i],
                                (*vertice_normals)[i]);
        igl::per_corner_normals(scans[i]((*non_zero_idx)[i], Eigen::all), (*meshes)[i],
                                (*corner_normals)[i]);
        /* Plot mesh with normals
        igl::per_face_normals(scans[i]((*non_zero_idx)[i], Eigen::all), (*meshes)[i], N_faces);
        igl::per_vertex_normals(scans[i]((*non_zero_idx)[i], Eigen::all), (*meshes)[i], N_vertices);
        igl::per_corner_normals(scans[i]((*non_zero_idx)[i], Eigen::all), (*meshes)[i], N_corners);
        Eigen::MatrixXd V = scans[i]((*non_zero_idx)[i], Eigen::all);
        igl::opengl::glfw::Viewer viewer;
        viewer.callback_key_down = &keyDown;
        viewer.data().show_lines = false;
        viewer.data().set_mesh(V, (*meshes)[i]);
        viewer.data().set_normals(N_vertices);
        const Eigen::RowVector3d red(0.8, 0.2, 0.2), blue(0.2, 0.2, 0.8);
        viewer.data().add_edges(V,
                                V + N_vertices, red);

        viewer.data().set_face_based(true);
        std::cout <<
                  "Press '1' for per-face normals." << std::endl <<
                  "Press '2' for per-vertex normals." << std::endl <<
                  "Press '3' for per-corner normals." << std::endl;

        viewer.launch();
        //*/
    }
}

void CurlBA::optimization() {
    for (int j = 0; j < num_frames - 1; ++j) {
        double q_init[4] = {0.0261769, 0, 0, 0.999657};
        Eigen::Map<Eigen::Quaterniond> q_init_eigen(q_init);
        //            q_init_eigen = Eigen::Quaterniond::Identity();
        ceres::Manifold *q_parameterization = new ceres::EigenQuaternionManifold();
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(q_init, 4, q_parameterization);
        for (int i = 1; i < num_objects; ++i) {
            ceres::CostFunction *cost_function = SPHCoeffRotError::Create(
                (*coefficients)[j * num_objects + i], (*coefficients)[j + 1 * num_objects + i]);
            problem.AddResidualBlock(cost_function, nullptr, q_init);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 200;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";
        std::cout << q_init_eigen << std::endl;
        //* Test result
        Eigen::Quaterniond q_1(poses[j](4), poses[0](5), poses[j](6), poses[j](3));
        Eigen::Quaterniond q_2(poses[j + 1](4), poses[j + 1](5), poses[j + 1](6), poses[j + 1](3));
        Eigen::Matrix3d R_1 = q_1.toRotationMatrix();
        Eigen::Matrix3d R_2 = q_2.toRotationMatrix();
        Eigen::Matrix3d R_sh = q_init_eigen.toRotationMatrix();
        auto R_gt = (R_1.transpose() * R_2).eulerAngles(0, 1, 2);
        Eigen::Quaterniond Q_gt(R_1.transpose() * R_2);
        std::cout << "Quaternion gt: " << Q_gt << std::endl;
        std::cout << "Should be roll, pitch, yaw: \n" << R_gt << std::endl;
        std::cout << "Result roll, pitch, yaw:\n " << R_sh.eulerAngles(0, 1, 2) << std::endl;
        std::cout << "**************************************************** \n";
        //*/
    }
}

void CurlBA::saveCoefficients() {
    std::string root_dir = "/home/user/project/CURL_SLAM/tmp_data/coefficients";
    for (int frame_num = 0; frame_num <= num_frames; ++frame_num) {
        std::string folder_name = root_dir + "/" + std::to_string(frame_num);
        if (!std::filesystem::is_directory(folder_name)) {
            std::filesystem::create_directories(folder_name);
        }
        std::ofstream file;
        typedef std::numeric_limits<double> dbl;
        for (int i = 1; i < num_objects; ++i) {
            file.open(root_dir + "/" + std::to_string(frame_num) + "/" + std::to_string(i) +
                      ".txt");
            file.precision(17);
            for (int j = 0; j < (*coefficients)[frame_num * num_objects + i].size(); ++j) {
                file << (*coefficients)[frame_num * num_objects + i](j) << '\n';
            }
            file.close();
        }
    }
}

void CurlBA::processing() {
    /// getCoefficients() and meshing() can merge together to improve speed if necessary
    getCoefficients();
    //        meshing();
    //        optimization();
    saveCoefficients();
}

bool CurlBA::keyDown(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) {
    switch (key) {
    case '1':
        viewer.data().set_normals(N_faces);
        return true;
    case '2':
        viewer.data().set_normals(N_vertices);
        return true;
    case '3':
        viewer.data().set_normals(N_corners);
        return true;
    case '4':
        viewer.data().show_lines = false;
    case '5':
        viewer.data().show_lines = true;
    default:
        break;
    }
    return false;
}
} // namespace curl