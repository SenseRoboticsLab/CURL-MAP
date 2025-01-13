//
// Created by zkc on 07/02/23.
//
#ifndef CURL_SLAM_TYPES_H
#define CURL_SLAM_TYPES_H

#define PCL_NO_PRECOMPILE
#include "FileReader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define INVALID_IDX -1
struct PointXYZILID {
    PCL_ADD_POINT4D;                // quad-word XYZ
    float intensity;                ///< laser intensity reading
    uint16_t label;                 ///< point label
    uint16_t id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID,
                                  (float, x, x)(float, y, y)(float, z,
                                                             z)(float, intensity,
                                                                intensity)(uint16_t, label,
                                                                           label)(uint16_t, id, id))
typedef PointXYZILID PointT;
typedef FileReader<double, std::size_t> FR;
typedef FR::Basic_Type BT;
typedef FR::Semantic_Type ST;

#endif // CURL_SLAM_TYPES_H
