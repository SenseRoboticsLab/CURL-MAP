#if !defined(REGION_GROWING_SEGMENTATION)
#define REGION_GROWING_SEGMENTATION
#include "FileReaderBase.h"
#include "load_config.h"
#include <igl/avg_edge_length.h>
#include <igl/cotmatrix.h>
#include <igl/invert_diag.h>
#include <igl/massmatrix.h>
#include <igl/parula.h>
#include <igl/per_corner_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/principal_curvature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
/**
 * @brief todo Extract segmentation index from cluster variable and generate colormap from previous
 * one. only use curvature is enough
 *
 * @tparam BasicType
 * @tparam DerivedV
 * @tparam DerivedF
 * @param pcl_normal_config
 * @param region_growing_config
 * @param V
 * @param cloud
 * @param F
 * @param V_normal
 * @param cloud_normal
 * @param V_seg
 */
template <typename BasicType, typename DerivedV, typename DerivedF>
void region_growing_segmentation(
    PCL_NORMAL_CONFIG &pcl_normal_config, REGION_GROWING_CONFIG &region_growing_config,
    Eigen::MatrixBase<DerivedV> &V, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    Eigen::MatrixBase<DerivedF> &F, Eigen::PlainObjectBase<DerivedV> &V_normal,
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, Eigen::VectorXi &V_seg,
    Eigen::Matrix<double, Eigen::Dynamic, 3> &colors) {
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    if (region_growing_config.is_igl) {
        igl::per_vertex_normals(V, F, V_normal);
        Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> HN;
        Eigen::SparseMatrix<BasicType> L, M, Minv;
        igl::cotmatrix(V, F, L);
        igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_VORONOI, M);
        igl::invert_diag(M, Minv);
        // Laplace-Beltrami of position
        HN = -Minv * (L * V);
        // Extract magnitude as mean curvature
        Eigen::Matrix<BasicType, Eigen::Dynamic, 1> H = HN.rowwise().norm();
        H = H.array() / H.maxCoeff();
        std::cout << "Min value: " << H.minCoeff() << " Max value: " << H.maxCoeff() << std::endl;
        std::cout << "Average value: " << H.sum() / H.size() << std::endl;
        cloud_normal->resize(V.rows());
        for (int i = 0; i < cloud->size(); ++i) {
            // cloud_normal->points[i].Normal(V_normal(i, 0), V_normal(i, 1), V_normal(i, 2),
            // HN(i));
            cloud_normal->points[i].normal_x = V_normal(i, 0);
            cloud_normal->points[i].normal_y = V_normal(i, 1);
            cloud_normal->points[i].normal_z = V_normal(i, 2);
            cloud_normal->points[i].curvature = HN(i);
        }
    } else {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(cloud);
        normal_estimator.setKSearch(pcl_normal_config.search_region);
        normal_estimator.compute(*cloud_normal);
    }
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(region_growing_config.min_cluster_size);
    reg.setMaxClusterSize(region_growing_config.max_cluster_size);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(region_growing_config.number_of_neighbours);
    reg.setInputCloud(cloud);
    // reg.setIndices (indices);
    reg.setInputNormals(cloud_normal);
    reg.setSmoothnessThreshold(region_growing_config.smoothness_threshold);
    reg.setCurvatureThreshold(region_growing_config.curvature_threshold);
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    // TODO: Normals might not be needed and a more efficient segmentation id representation will
    // need
    V_normal.resize(cloud_normal->size(), 3);
    V_seg.resize(cloud->size());
    for (int i = 0; i < cloud->size(); ++i) {
        V_normal(i, 0) = cloud_normal->points[i].normal_x;
        V_normal(i, 1) = cloud_normal->points[i].normal_y;
        V_normal(i, 2) = cloud_normal->points[i].normal_z;
    }
    for (int i = 0; i < clusters.size(); ++i) {
        for (int j = 0; j < clusters[i].indices.size(); ++j) {
            V_seg[clusters[i].indices[j]] = i;
        }
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    colors.resize(colored_cloud->size(), 3);
    for (int i = 0; i < colored_cloud->size(); ++i) {
        V(i, 0) = colored_cloud->points[i].x;
        V(i, 1) = colored_cloud->points[i].y;
        V(i, 2) = colored_cloud->points[i].z;
        colors(i, 0) = colored_cloud->points[i].r / 255.0;
        colors(i, 1) = colored_cloud->points[i].g / 255.0;
        colors(i, 2) = colored_cloud->points[i].b / 255.0;
    }
    pcl::copyPointCloud(*colored_cloud, *cloud);

    // pcl::visualization::CloudViewer viewer("Cluster viewer");
    // viewer.showCloud(colored_cloud);
    // while (!viewer.wasStopped()) {
    // }
}
#endif // REGION_GROWING_SEGMENTATION
