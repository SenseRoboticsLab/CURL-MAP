/**
 * @file DebugBase.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef EXAMPLE_DEBUG_BASE_H
#define EXAMPLE_DEBUG_BASE_H

#include "BA_error_term.h"
#include "Timer.h"
#include "convex_hull_2D.h"
#include "curl_tools.h"
#include "delaunay_triangulation.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "igl/opengl/glfw/Viewer.h"
#include "triangle_on_sphere_range.h"
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

template <typename BasicType> class DebugBase {
  private:
    // normal variables
    std::deque<Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic>> *scans;
    std::deque<Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic>> *semantics;
    // debug variables
    std::deque<Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic>> *semantics_debug;
    std::vector<Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic>> *poses_debug;
};

#endif // EXAMPLE_DEBUG_BASE_H