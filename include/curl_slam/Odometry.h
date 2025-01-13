/**
 * @file Odometry.h
 * @author zkc (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-11
 *
 * @copyright Copyright (c) 2022
 *
 */
#if !defined(EXAMPLE_ODOMETRY)
#define EXAMPLE_ODOMETRY
#include "CurlGeometry.h"
#include "FileReader.h"
#include "curl_tools.h"
#include "delaunay_triangulation.h"
#include "lock_manager.h"
#include "region_growing_segmentation.h"
#include "surface_points_approximation.h"
#include <igl/gaussian_curvature.h>
#include <igl/opengl/glfw/Viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/cloud_viewer.h>

template <typename BasicType, typename SemanticType> class Odometry {
  public:
    // define the types used in FileReader
    typedef typename FileReader<BasicType, SemanticType>::DynamicMatrix DynamicMatrix;
    typedef typename FileReader<BasicType, SemanticType>::DynamicVector DynamicVector;
    typedef typename FileReader<BasicType, SemanticType>::DynamicMatrixSem DynamicMatrixSem;
    // config parameters
    PRE_PROCESSION_CONFIG pre_procession_config;
    VSA_CONFIG vsa_config;
    PCL_NORMAL_CONFIG pcl_normal_config;
    REGION_GROWING_CONFIG region_growing_config;
    CURVATURE_CONFIG curvature_config;
    PROBABILITY_CLUSTERING_CONFIG probability_clustering_config;
    // object pointer of FileReader
    std::shared_ptr<FileReader<BasicType, SemanticType>> file_loader_ptr;
    // data poped from the queue
    DynamicMatrix original_scan;
    DynamicMatrix current_scan;
    DynamicMatrixSem current_semantic;
    DynamicMatrix current_pose;
    DynamicMatrix processed_scan;
    Eigen::MatrixXi faces;
    Eigen::VectorXi scan_segmentation;
    DynamicMatrix processed_normal;
    Eigen::Matrix<double, Eigen::Dynamic, 3> colors;
    std::shared_ptr<Ubpa::HEMeshInherit> mesh_eigen;

    // pcl format
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr processed_cloud_normal;
    // pose counter
    int frame_counter{0};
    // check if whether does the preprocessing is needed
    bool is_preprocessing{false};
    //
    std::size_t initial_point_index{0};

  public:
    // constructor
    Odometry(std::shared_ptr<FileReader<BasicType, SemanticType>> _file_loader_ptr,
             const std::string &filename);

  public:
    // update the pcl cloud
    void update_pcl_cloud(DynamicMatrix &V, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    // update cleaned scan
    void update_matrix_scan(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, DynamicMatrix &V);
    // pcl statistical outlier remove
    void pcl_outlier_remove(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, double mean,
                            double std);
    // remove zero points
    bool remove_zero_points(DynamicMatrix &M);
    // Eigen vector sorting
    std::vector<size_t> sort_indexes(DynamicVector &v);
    bool no_load_data_from_queue();
    // get initial point
    std::size_t get_initial_face_index();
    // preprocessing function that can remove outlier points
    void preprocessing();
    // main procedure
    void procedure();
};
#include "../lib/Odometry.cpp"
#endif // EXAMPLE_ODOMETRY