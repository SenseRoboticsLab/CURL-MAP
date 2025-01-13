//
// Created by zkc on 18/07/22.
//

#ifndef EXAMPLE_CURL_BA_H
#define EXAMPLE_CURL_BA_H

#include "BA_error_term.h"
#include "Timer.h"
#include "convex_hull_2D.h"
#include "curl_tools.h"
#include "delaunay_triangulation.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "igl/opengl/glfw/Viewer.h"
#include <Eigen/Core>
#include <cmath>
#include <filesystem>
#include <igl/per_corner_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <string>
#include <sys/stat.h>
#include <utility>
#include <vector>

namespace curl {
Eigen::MatrixXd N_vertices;
Eigen::MatrixXd N_faces;
Eigen::MatrixXd N_corners;
class CurlBA {
  private:
    std::vector<Eigen::VectorXd> *coefficients;
    std::vector<Matrix3dCol> *sph_cor;
    std::vector<std::vector<int>> *sem_idx;
    std::vector<std::vector<int>> *non_zero_idx;
    std::vector<Eigen::Matrix<int, Eigen::Dynamic, 3>> *meshes;
    std::vector<Matrix3dCol> *face_normals;
    std::vector<Matrix3dCol> *vertice_normals;
    std::vector<Matrix3dCol> *corner_normals;

    int num_frames;
    int num_channels;
    int num_cols;

    static bool keyDown(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier);

    void getCoefficients();

    void meshing();

    void optimization();

    void saveCoefficients();

  public:
    std::vector<Eigen::MatrixXd> poses;
    std::vector<Matrix3dCol> scans;
    std::vector<Eigen::MatrixXi> semantics;
    int degree;
    int num_objects;

    /// Functions
    CurlBA(std::vector<Eigen::MatrixXd> _poses, std::vector<Matrix3dCol> _scans,
           std::vector<Eigen::MatrixXi> _semantics, int _num_channels, int _degree,
           int _num_objects);

    ~CurlBA();

    void processing();
};
} // namespace curl
#endif // EXAMPLE_CURL_BA_H
