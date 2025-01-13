//
// Created by zkc on 09/02/23.
//
#pragma once
/**
 *  Odometry
 */
// constructors
template <typename BasicType, typename SemanticType>
Odometry<BasicType, SemanticType>::Odometry(std::shared_ptr<FileReader<BasicType, SemanticType>> _file_loader_ptr,
                                            const std::string &filename)
    : file_loader_ptr(_file_loader_ptr), current_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      processed_cloud(new pcl::PointCloud<pcl::PointXYZ>), processed_cloud_normal(new pcl::PointCloud<pcl::Normal>),
      frame_counter(_file_loader_ptr->start_frame) {
    is_preprocessing = load_preprocessing_config(filename, pre_procession_config);
    load_vsa_config(filename, vsa_config);
    load_pcl_normal_config(filename, pcl_normal_config);
    load_region_growing_config(filename, region_growing_config);
    load_curvature_config(filename, curvature_config);
    load_probability_clustering_config(filename, probability_clustering_config);
    if (probability_clustering_config.th_normal_angle != 0) {
        // change degree to radians
        probability_clustering_config.th_normal_angle = probability_clustering_config.th_normal_angle * M_PI / 180;
    }
    procedure();
}

// member functions
template <typename BasicType, typename SemanticType>
void Odometry<BasicType, SemanticType>::update_pcl_cloud(DynamicMatrix &V, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    cloud->resize(V.rows());
    for (int i = 0; i < cloud->size(); ++i) {
        cloud->points[i].x = V(i, 0);
        cloud->points[i].y = V(i, 1);
        cloud->points[i].z = V(i, 2);
    }
}

template <typename BasicType, typename SemanticType>
void Odometry<BasicType, SemanticType>::update_matrix_scan(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                           DynamicMatrix &V) {
    V.resize(cloud->size(), 3);
    for (int i = 0; i < cloud->size(); ++i) {
        V(i, 0) = cloud->points[i].x;
        V(i, 1) = cloud->points[i].y;
        V(i, 2) = cloud->points[i].z;
    }
}

template <typename BasicType, typename SemanticType>
void Odometry<BasicType, SemanticType>::pcl_outlier_remove(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                           pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                                                           double mean, double std) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(mean);
    sor.setStddevMulThresh(std);
    sor.filter(*output_cloud);
}

template <typename BasicType, typename SemanticType>
bool Odometry<BasicType, SemanticType>::remove_zero_points(DynamicMatrix &M) {
    std::vector<int> nonzero_idx;
    for (int i = 0; i < M.rows(); ++i) {
        if (M(i, 0) != 0 || M(i, 1) != 0 || M(i, 2) != 0) {
            nonzero_idx.push_back(i);
        }
    }
    if (nonzero_idx.size() > 0) {
        M = M(nonzero_idx, Eigen::all);
        return false;
    } else {
        return true;
    }
}

template <typename BasicType, typename SemanticType>
std::vector<size_t> Odometry<BasicType, SemanticType>::sort_indexes(DynamicVector &v) {
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);
    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    stable_sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v(i1) < v(i2); });
    return idx;
}

template <typename BasicType, typename SemanticType> bool Odometry<BasicType, SemanticType>::no_load_data_from_queue() {
    if (!file_loader_ptr->scans_queue.empty()) {
        // fetch out needed data
        m_queue_lock.lock();
        original_scan = file_loader_ptr->scans_queue.front();
        current_scan = original_scan;
        file_loader_ptr->scans_queue.pop();
        if (file_loader_ptr->is_debug) {
            if (file_loader_ptr->is_semantic) {
                current_semantic = file_loader_ptr->semantics_queue.front();
                file_loader_ptr->semantics_queue.pop();
            }
            if (file_loader_ptr->is_pose) {
                current_pose = file_loader_ptr->poses_gt.row(frame_counter);
            }
        }
        m_queue_lock.unlock();
        ++frame_counter;
        if (remove_zero_points(current_scan)) {
            return true;
        }
        processed_scan = current_scan;
        update_pcl_cloud(current_scan, current_cloud);
        update_pcl_cloud(processed_scan, processed_cloud);
        return false;
    }
    return false;
}

template <typename BasicType, typename SemanticType> void Odometry<BasicType, SemanticType>::preprocessing() {
    // remove furtherest points and closest points
    DynamicVector r_square_col =
        current_scan.col(0).array().pow(2) + current_scan.col(1).array().pow(2) + current_scan.col(2).array().pow(2);
    std::vector<size_t> r_sorted_idx = sort_indexes(r_square_col);
    size_t higher_idx = static_cast<size_t>(r_sorted_idx.size() * pre_procession_config.length_percentage);
    pre_procession_config.max_length = std::sqrt(r_square_col(r_sorted_idx[higher_idx]));
    size_t lower_idx = 0;
    for (int i = 0; i < r_sorted_idx.size(); ++i) {
        if (r_square_col(r_sorted_idx[i]) >= pre_procession_config.min_length) {
            lower_idx = i;
            break;
        }
    }
    processed_scan = current_scan(
        std::vector<size_t>(r_sorted_idx.begin() + lower_idx, r_sorted_idx.begin() + higher_idx), Eigen::all);
    // pcl library to remove outlier points
    update_pcl_cloud(processed_scan, processed_cloud);
    pcl_outlier_remove(processed_cloud, processed_cloud, pre_procession_config.filter_mean,
                       pre_procession_config.filter_std);
    update_matrix_scan(processed_cloud, processed_scan);
}

template <typename BasicType, typename SemanticType>
std::size_t Odometry<BasicType, SemanticType>::get_initial_face_index() {
    std::size_t point_init_idx = std::numeric_limits<std::size_t>::max();
    std::size_t face_init_idx = std::numeric_limits<std::size_t>::max();
    Eigen::MatrixXd scan = processed_scan;
    Eigen::MatrixXd sph = curl::cart2sph(scan);
    double yaw = std::numeric_limits<double>::max();
    for (int i = 0; i < sph.rows(); ++i) {
        if (sph(i, 1) < yaw) {
            yaw = sph(i, 1);
            point_init_idx = i;
        }
    }
    for (int i = 0; i < faces.rows(); ++i) {
        if (faces(i, 0) == point_init_idx || faces(i, 1) == point_init_idx || faces(i, 2) == point_init_idx) {
            face_init_idx = i;
            break;
        }
    }

    return face_init_idx;
}

template <typename BasicType, typename SemanticType> void Odometry<BasicType, SemanticType>::procedure() {
    Timer total_timer;
    Timer preprocessing_timer;
    Timer triangulation_timer;
    Timer vsa_timer;
    Timer region_growing_timer;
    Timer igl_gaussian_curvature;
    Timer self_gaussian_curvature;
    Timer half_edge_initialization;
    total_timer.start();
    while (frame_counter <= file_loader_ptr->end_frame) {
        int wait_counter = 0;
        while (file_loader_ptr->scans_queue.empty()) {
            ++wait_counter;
            std::cout << "Wait data to load in queue!!! " << wait_counter << std::endl;
        }
        if (no_load_data_from_queue()) {
            continue;
        }
        if (is_preprocessing) {
            // preprocessing
            preprocessing_timer.start();
            preprocessing();
            std::cout << "Preprocessing: " << preprocessing_timer.elapsedMilliseconds() / 1000.0 << "s" << std::endl;
        }
        // triangulation
        triangulation_timer.start();
        cgal::delaunay_triangulation_on_sphere_func(processed_scan, faces, processed_scan);
        std::cout << "Triangulation: " << triangulation_timer.elapsedMilliseconds() / 1000.0 << "s" << std::endl;
        igl_gaussian_curvature.start();
        // calculate gaussian curvature and plot it
        DynamicMatrix K;
        // Compute integral of Gaussian curvature
        igl::gaussian_curvature(processed_scan, faces, K);
        // Compute mass matrix
        Eigen::SparseMatrix<double> M, Minv;
        igl::massmatrix(processed_scan, faces, igl::MASSMATRIX_TYPE_VORONOI, M);
        igl::invert_diag(M, Minv);
        // Divide by area to get integral average
        K = (Minv * K).eval();
        std::cout << "Gaussian Curvature: " << igl_gaussian_curvature.elapsedMilliseconds() / 1000.0 << "s"
                  << std::endl;
        //        if (file_loader_ptr->is_debug) {
        //            curl::points_curvature_viewer<BasicType>(processed_scan, faces, K);
        //
        //            FileReaderBase::write_txt_file("/home/user/project/CURL_SLAM/tmp_data/gaussian_curvature.txt",
        //            K);
        //            //        curl::mesh_data_viewer(processed_scan, faces, K);
        //        }

        // initialize half-edge data structure
        half_edge_initialization.start();
        mesh_eigen = std::make_shared<Ubpa::HEMeshInherit>();
        mesh_eigen->Init(processed_scan, faces);
        std::cout << "Half-edge Initialization: " << half_edge_initialization.elapsedMilliseconds() / 1000.0 << "s"
                  << std::endl;
        self_gaussian_curvature.start();
        mesh_eigen->calculate_face_normal_and_area();
        mesh_eigen->calculate_vertex_normal_and_curvature(curvature_config.gaussian_curvature_is_selected,
                                                          curvature_config.mean_curvature_is_selected,
                                                          curvature_config.area_method); // MIXED_VORONOI; BARYCENTRIC
        std::cout << "Vertex Normal and Gaussian Curvature: " << self_gaussian_curvature.elapsedMilliseconds() / 1000.0
                  << "s" << std::endl;
        initial_point_index = get_initial_face_index();
        mesh_eigen->probability_clustering(initial_point_index, probability_clustering_config);
        int counter = 0;
        for (auto *p : mesh_eigen->Polygons()) {
            if (p->semantic == ST_INVALID) {
                ++counter;
            }
        }
        std::cerr << "Error Counter: " << counter << std::endl;
        if (file_loader_ptr->is_debug) {
            // clustering color and pedigree chart
            // ToDo: max_cluster_idx will no longer be used after change the code
            // eigen matrix will have noise data if not set zero.
            //            Eigen::MatrixXd pedigree_chart(mesh_eigen->max_cluster_idx, 6);
            //            for (int i = 0; i < mesh_eigen->max_cluster_idx; ++i) {
            //                ST parent_map_idx = mesh_eigen->regions_map[i].parent_semantic;
            //                pedigree_chart(i, Eigen::seq(0, 2)) =
            //                mesh_eigen->regions_map[parent_map_idx].seed->get_centroid();
            //                pedigree_chart(i, Eigen::seq(3, 5)) =
            //                mesh_eigen->regions_map[i].seed->get_centroid();
            //            }
            // debug probability_clustering
            scan_segmentation.resize(faces.rows());
            int index = 0;
            DynamicMatrix scan_segmentation_matrix(faces.rows(), 1);
            DynamicMatrix face_gaussian_curvature(faces.rows(), 1);
            DynamicMatrix face_normals(faces.rows(), 3);
            for (auto *f : mesh_eigen->Polygons()) {
                scan_segmentation(index) = static_cast<int>(f->semantic);
                scan_segmentation_matrix(index, 0) = static_cast<BT>(f->semantic);
                f->HalfEdge()->Origin()->semantic = f->semantic;
                f->HalfEdge()->End()->semantic = f->semantic;
                f->HalfEdge()->Pre()->Origin()->semantic = f->semantic;
                face_normals(index, 0) = f->normal[0];
                face_normals(index, 1) = f->normal[1];
                face_normals(index, 2) = f->normal[2];
                face_gaussian_curvature(index, 0) = f->get_gaussian_curvature();
                //                std::cout << f->semantic << std::endl;
                ++index;
            }
            FileReaderBase::write_txt_file("/home/user/project/CURL_SLAM/tmp_data/probability_clustering.txt",
                                           scan_segmentation_matrix);
            //            curl::mesh_color_viewer<BasicType>(processed_scan, faces,
            //            scan_segmentation_matrix);
            //            curl::random_color_points_plot_with_pedigree_chart(processed_scan, faces,
            //            scan_segmentation,
            //                                                               pedigree_chart);
            // plot specific region
            std::vector<BT *> region_points_vec;
            for (auto *v : mesh_eigen->Vertices()) {
                if (v->semantic == probability_clustering_config.region_num) {
                    region_points_vec.push_back(v->position);
                }
            }
            DynamicMatrix region_points(region_points_vec.size(), 3);
            DynamicMatrix blue(region_points_vec.size(), 3);
            for (int i = 0; i < region_points_vec.size(); ++i) {
                region_points(i, 0) = region_points_vec[i][0];
                region_points(i, 1) = region_points_vec[i][1];
                region_points(i, 2) = region_points_vec[i][2];
                blue(i, 0) = 0.8;
                blue(i, 1) = 0.2;
                blue(i, 2) = 0.2;
            }
            curl::point_viewer(region_points, 3, blue);
            // mesh
            index = 0;
            std::vector<std::size_t> mesh_idx;
            for (auto *p : mesh_eigen->Polygons()) {
                if (p->semantic == probability_clustering_config.region_num) {
                    mesh_idx.push_back(index);
                }
                ++index;
            }
            Eigen::MatrixXi partial_faces = faces(mesh_idx, Eigen::all);
            curl::mesh_viewer_no_color(processed_scan, partial_faces);
            curl::vsa_color_points_plot<BasicType>(processed_scan, faces, scan_segmentation);
            curl::mesh_curvature_viewer(processed_scan, faces, face_gaussian_curvature);
            //            curl::mesh_curvature_viewer(processed_scan, faces,
            //            face_gaussian_curvature, face_normals);

            // debug curvature
            processed_normal.resize(processed_scan.rows(), 3);
            DynamicMatrix processed_gaussian_curvature(processed_scan.rows(), 1);
            DynamicMatrix processed_mean_curvature(processed_scan.rows(), 1);
            index = 0;
            for (auto *v : mesh_eigen->Vertices()) {
                processed_gaussian_curvature(index) = v->gaussian_curvature;
                processed_mean_curvature(index) = v->mean_curvature;
                for (int i = 0; i < 3; ++i) {
                    processed_normal(index, i) = v->normal[i];
                }
                ++index;
            }
            FileReaderBase::write_txt_file("/home/user/project/CURL_SLAM/tmp_data/cal_gaussian_curvature.txt",
                                           processed_gaussian_curvature);
            curl::points_curvature_viewer<BasicType>(processed_scan, faces, processed_gaussian_curvature);
            curl::points_curvature_viewer<BasicType>(processed_scan, faces, processed_mean_curvature);

            curl::points_curvature_viewer(processed_scan, faces, processed_mean_curvature, processed_normal);
            //            Ubpa::Print(mesh_eigen);
        }

        // clustering
        /**
         * @brief mesh clustering
         */

        /**
         * @brief vsa clustering
         */
        //*
        if (vsa_config.is_selected) {
            vsa_timer.start();
            cgal::vsa_approximation(processed_scan, faces, vsa_config, scan_segmentation);
            std::cout << "VSA: " << vsa_timer.elapsedMilliseconds() / 1000.0 << "s" << std::endl;
            curl::vsa_color_points_plot<BasicType>(processed_scan, faces, scan_segmentation);
        }

        //*/
        /**
         * @brief region growing clustering
         */
        //*
        if (region_growing_config.is_selected) {
            region_growing_timer.start();
            region_growing_segmentation<BasicType>(pcl_normal_config, region_growing_config, processed_scan,
                                                   processed_cloud, faces, processed_normal, processed_cloud_normal,
                                                   scan_segmentation, colors);
            std::cout << "Region growing: " << region_growing_timer.elapsedMilliseconds() / 1000.0 << "s" << std::endl;
            curl::region_growing_points_plot<BasicType>(processed_scan, colors);
        }
    }
    std::cout << "Average time for each loop: "
              << total_timer.elapsedMilliseconds() / 1000.0 /
                     (file_loader_ptr->end_frame - file_loader_ptr->start_frame)
              << "s" << std::endl;
}