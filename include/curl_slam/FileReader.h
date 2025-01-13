/**
 * @file FileReader.h
 * @author zkc (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef EXAMPLE_FILE_READER
#define EXAMPLE_FILE_READER
#include "FileReaderBase.h"
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

// declarations
template <typename BasicType, typename SemanticType = BasicType> class FileReader;
template <typename BasicType, typename SemanticType = BasicType>
void add_to_queue(std::shared_ptr<FileReader<BasicType, SemanticType>> file_loader);
template <typename BasicType, typename SemanticType = BasicType>
void add_to_queue_ref(FileReader<BasicType, SemanticType> &file_loader);

/**
 * @brief This is the class that read files and saving in a queue
 *
 * @tparam BasicType
 */
template <typename BasicType, typename SemanticType> class FileReader : public FileReaderBase {
  public:
    // define basic type so can be access from the outside
    typedef BasicType Basic_Type;
    typedef SemanticType Semantic_Type;
    // type define local eigen dynamic matrix type
    typedef typename Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> DynamicMatrix;
    typedef typename Eigen::Matrix<BasicType, Eigen::Dynamic, 1> DynamicVector;
    typedef typename Eigen::Matrix<SemanticType, Eigen::Dynamic, Eigen::Dynamic> DynamicMatrixSem;

  private:
    // directories
    std::string scans_dir;
    std::string semantics_dir; // debug
    std::string poses_dir;     // debug
    // filenames
    std::string scans_filename;
    std::string semantics_filename; //  debug
    std::string poses_filename;     // debug
    // file types
    std::string scan_file_type;
    std::string semantic_file_type;
    std::string poses_file_type;
    // scale factor
    double scan_scale_factor = 1;
    // check if the directory/file exists
    struct stat info;
    // simulator optimization files
    std::string transformation_obj_LiDAR_dir;
    std::string conformal_map_dir;
    std::string gamma_gradient_dir;
    std::string valid_idx_dir;
    std::string sph_coeff_dir;
    std::string odometry_file;
    std::string odometry_noised_file;
    std::string trajectory_file;
    std::string trajectory_noised_file;

  public:
    int obj_num;
    int start_frame = 0;
    int end_frame = 0;
    bool is_debug = false;
    bool is_semantic = false;
    bool is_pose = false;
    std::string data_set;

    std::queue<DynamicMatrix> scans_queue;
    std::queue<DynamicMatrixSem> semantics_queue;
    std::vector<DynamicMatrix> T_obj_LiDAR;
    std::queue<std::vector<DynamicMatrix>> T_obj_LiDAR_queue;
    std::vector<DynamicMatrix> conformal_map;
    std::vector<DynamicMatrix> m_x_Gx_vec;
    std::vector<DynamicMatrix> m_x_Gy_vec;
    std::vector<DynamicMatrix> m_y_Gx_vec;
    std::vector<DynamicMatrix> m_y_Gy_vec;
    std::vector<DynamicMatrixSem> valid_idx_vec;
    std::queue<std::vector<DynamicMatrix>> m_x_Gx_queue;
    std::queue<std::vector<DynamicMatrix>> m_x_Gy_queue;
    std::queue<std::vector<DynamicMatrix>> m_y_Gx_queue;
    std::queue<std::vector<DynamicMatrix>> m_y_Gy_queue;
    std::queue<std::vector<DynamicMatrixSem>> valid_idx_queue;
    std::vector<DynamicMatrix> sph_coeff;
    std::queue<std::vector<DynamicMatrix>> sph_coeff_queue;
    std::queue<std::vector<DynamicMatrix>> conformal_map_queue;
    DynamicMatrix odometry;
    DynamicMatrix odometry_noised;
    DynamicMatrix trajectory;
    DynamicMatrix trajectory_noised;
    DynamicMatrix poses_gt;

    // constructor
    FileReader() : FileReaderBase() {}
    FileReader(std::string _root_file, int _start_frame, int _end_frame)
        : FileReaderBase(_root_file), start_frame(_start_frame), end_frame(_end_frame) {
        scans_dir = _root_file + "/scans";
        semantics_dir = _root_file + "/semantics";
        poses_dir = _root_file + "/poses";
    };
    // initilize with yaml file
    FileReader(std::string filename) {
        FILE_READER_CONFIG file_reader_config;
        load_config_file(filename, file_reader_config);
        data_set = file_reader_config.data_set;
        root_file_0 = file_reader_config.root_dir;
        bool is_dir_good = true;
        if (stat(root_file_0.c_str(), &info) != 0) {
            std::cerr << "Root directory: " << root_file_0 << " doesn't exist!!!" << std::endl;
            is_dir_good = false;
        }
        scans_dir = root_file_0 + file_reader_config.sequence + file_reader_config.scans_dir;
        if (stat(scans_dir.c_str(), &info) != 0) {
            std::cerr << "Scans directory: " << scans_dir << " doesn't exist!!!" << std::endl;
            is_dir_good = false;
        }
        scan_scale_factor = file_reader_config.scan_scale_factor;
        if (file_reader_config.semantics_dir != "") {
            semantics_dir =
                root_file_0 + file_reader_config.sequence + file_reader_config.semantics_dir;
            is_semantic = true;
            if (stat(semantics_dir.c_str(), &info) != 0) {
                std::cerr << "Semantics directory: " << semantics_dir << " doesn't exist!!!"
                          << std::endl;
                is_dir_good = false;
            }
        }
        if (file_reader_config.poses_dir != "") {
            poses_dir = root_file_0 + file_reader_config.poses_dir + file_reader_config.sequence;
            is_pose = true;
            if (stat((root_file_0 + file_reader_config.poses_dir).c_str(), &info) != 0) {
                std::cerr << "Pose directory: " << root_file_0 + file_reader_config.poses_dir
                          << " doesn't exist!!!" << std::endl;
                is_dir_good = false;
            }
        }
        if (file_reader_config.transformation_obj_LiDAR_dir != "") {
            transformation_obj_LiDAR_dir = root_file_0 + file_reader_config.sequence +
                                           file_reader_config.transformation_obj_LiDAR_dir;
            if (stat(transformation_obj_LiDAR_dir.c_str(), &info) != 0) {
                std::cerr << "Transformation_obj_LiDAR directory: " << transformation_obj_LiDAR_dir
                          << " doesn't exist!!!" << std::endl;
                is_dir_good = false;
            }
        }
        if (file_reader_config.conformal_map_dir != "") {
            conformal_map_dir =
                root_file_0 + file_reader_config.sequence + file_reader_config.conformal_map_dir;
            if (stat(conformal_map_dir.c_str(), &info) != 0) {
                std::cerr << "Conformal map directory: " << conformal_map_dir << " doesn't exist!!!"
                          << std::endl;
                is_dir_good = false;
            }
        }
        if (file_reader_config.gamma_gradient_dir != "") {
            std::ostringstream out;
            out << std::fixed << std::setprecision(2) << file_reader_config.gamma_rso;
            gamma_gradient_dir = root_file_0 + file_reader_config.sequence +
                                 file_reader_config.gamma_gradient_dir + "/gamma_rso_" + out.str();
            if (stat(gamma_gradient_dir.c_str(), &info)) {
                std::cerr << "Gamma gradient directory: " << gamma_gradient_dir
                          << " doesn't exist!!!" << std::endl;
                is_dir_good = false;
            }
        }
        if (file_reader_config.valid_idx_dir != "") {
            valid_idx_dir =
                root_file_0 + file_reader_config.sequence + file_reader_config.valid_idx_dir;
            if (stat(valid_idx_dir.c_str(), &info)) {
                std::cerr << "Gamma gradient directory: " << valid_idx_dir << " doesn't exist!!!"
                          << std::endl;
                is_dir_good = false;
            }
        }
        if (file_reader_config.sph_coeff_dir != "") {
            sph_coeff_dir =
                root_file_0 + file_reader_config.sequence + file_reader_config.sph_coeff_dir;
            if (stat(sph_coeff_dir.c_str(), &info) != 0) {
                std::cerr << "SPH coefficients directory: " << sph_coeff_dir << " doesn't exist!!!"
                          << std::endl;
                is_dir_good = false;
            }
        }
        if (file_reader_config.odometry_file != "") {
            odometry_file =
                root_file_0 + file_reader_config.sequence + file_reader_config.odometry_file;
            if (stat(odometry_file.c_str(), &info) != 0) {
                std::cerr << "Odometry file: " << odometry_file << " doesn't exist!!!" << std::endl;
                is_dir_good = false;
            }
        }
        if (file_reader_config.odometry_noised_file != "") {
            odometry_noised_file =
                root_file_0 + file_reader_config.sequence + file_reader_config.odometry_noised_file;
            if (stat(odometry_noised_file.c_str(), &info) != 0) {
                std::cerr << "Odometry noised file: " << odometry_noised_file << " doesn't exist!!!"
                          << std::endl;
                is_dir_good = false;
            }
        }
        if (file_reader_config.trajectory_file != "") {
            trajectory_file =
                root_file_0 + file_reader_config.sequence + file_reader_config.trajectory_file;
            if (stat(trajectory_file.c_str(), &info) != 0) {
                std::cerr << "Trajectory file: " << trajectory_file << " doesn't exist!!!"
                          << std::endl;
                is_dir_good = false;
            }
        }
        if (file_reader_config.trajectory_noised_file != "") {
            trajectory_noised_file = root_file_0 + file_reader_config.sequence +
                                     file_reader_config.trajectory_noised_file;
            if (stat(trajectory_noised_file.c_str(), &info) != 0) {
                std::cerr << "Trajectory noised file: " << trajectory_noised_file
                          << " doesn't exist!!!" << std::endl;
                is_dir_good = false;
            }
        }
        if (is_dir_good == false) {
            std::exit(EXIT_FAILURE);
        }
        if (file_reader_config.obj_num != 0) {
            obj_num = file_reader_config.obj_num;
        }
        start_frame = file_reader_config.start_frame;
        end_frame = file_reader_config.end_frame;
        scan_file_type = file_reader_config.scan_file_type;
        semantic_file_type = file_reader_config.semantic_file_type;
        poses_file_type = file_reader_config.poses_file_type;
        is_debug = file_reader_config.is_debug;
    }
    // set directories
    void set_scans_dir(std::string dir) { scans_dir = std::move(dir); }
    void set_semantics_dir(std::string dir) { semantics_dir = std::move(dir); }
    void set_poses_dir(std::string dir) { poses_dir = std::move(dir); }

    // friend void add_to_queue<BasicType, SemanticType>(FileReader &file_loader);
    friend void add_to_queue<BasicType, SemanticType>(std::shared_ptr<FileReader> file_loader_ptr);
    friend void add_to_queue_ref<BasicType, SemanticType>(FileReader &file_loader);
};

// for loop to read files in folder
template <typename BasicType, typename SemanticType>
void add_to_queue(std::shared_ptr<FileReader<BasicType, SemanticType>> file_loader_ptr) {
    typedef typename FileReader<BasicType, SemanticType>::DynamicMatrix DynamicMatrix;
    DynamicMatrix (*read_cloud)(std::string filename);
    if (file_loader_ptr->scan_file_type == "txt") {
        read_cloud = &FileReaderBase::read_xyz<BasicType>;
    } else if (file_loader_ptr->scan_file_type == "ply") {
        read_cloud = &FileReaderBase::read_ply<BasicType>;
    } else if (file_loader_ptr->scan_file_type == "pcd") {
        read_cloud = &FileReaderBase::read_pcd<BasicType>;
    } else {
        std::cerr << "Only support: 'txt', 'ply', and 'pcd'" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if (file_loader_ptr->is_pose) {
        file_loader_ptr->poses_filename =
            file_loader_ptr->poses_dir + "." + file_loader_ptr->poses_file_type;
        file_loader_ptr->poses_gt =
            FileReaderBase::read_txt_file<BasicType>(file_loader_ptr->poses_filename);
    }
    if (file_loader_ptr->data_set == "SIMULATOR_OPTIMIZATION") {
        file_loader_ptr->odometry =
            FileReaderBase::read_txt_file<BasicType>(file_loader_ptr->odometry_file);
        file_loader_ptr->odometry_noised =
            FileReaderBase::read_txt_file<BasicType>(file_loader_ptr->odometry_noised_file);
        file_loader_ptr->trajectory =
            FileReaderBase::read_txt_file<BasicType>(file_loader_ptr->trajectory_file);
        file_loader_ptr->trajectory_noised =
            FileReaderBase::read_txt_file<BasicType>(file_loader_ptr->trajectory_noised_file);
    }
    for (int i = file_loader_ptr->start_frame; i <= file_loader_ptr->end_frame; ++i) {
        m_queue_lock.lock();
        file_loader_ptr->scans_filename = file_loader_ptr->scans_dir + "/" + std::to_string(i) +
                                          "." + file_loader_ptr->scan_file_type;
        file_loader_ptr->scans_queue.push(
            ((*read_cloud)(file_loader_ptr->scans_filename).array() *
             static_cast<BasicType>(file_loader_ptr->scan_scale_factor)));
        if (file_loader_ptr->is_semantic) {
            file_loader_ptr->semantics_filename = file_loader_ptr->semantics_dir + "/" +
                                                  std::to_string(i) + "." +
                                                  file_loader_ptr->semantic_file_type;
            file_loader_ptr->semantics_queue.push(
                FileReaderBase::read_txt_file<SemanticType>(file_loader_ptr->semantics_filename));
        }
        if (file_loader_ptr->data_set == "SIMULATOR_OPTIMIZATION") {
            file_loader_ptr->T_obj_LiDAR.clear();
            file_loader_ptr->conformal_map.clear();
            file_loader_ptr->sph_coeff.clear();
            for (int j = 1; j <= file_loader_ptr->obj_num; ++j) {
                std::string T_obj_LiDAR_file = file_loader_ptr->transformation_obj_LiDAR_dir + "/" +
                                               std::to_string(i) + "_obj_" + std::to_string(j) +
                                               ".txt";
                std::string conformal_map_file = file_loader_ptr->conformal_map_dir + "/" +
                                                 std::to_string(i) + "_obj_" + std::to_string(j) +
                                                 ".txt";
                std::string sph_coeff_file = file_loader_ptr->sph_coeff_dir + "/" +
                                             std::to_string(i) + "_obj_" + std::to_string(j) +
                                             ".txt";
                file_loader_ptr->T_obj_LiDAR.push_back(
                    FileReaderBase::read_txt_file<BasicType>(T_obj_LiDAR_file));
                file_loader_ptr->conformal_map.push_back(
                    FileReaderBase::read_txt_file<BasicType>(conformal_map_file));
                file_loader_ptr->sph_coeff.push_back(
                    FileReaderBase::read_txt_file<BasicType>(sph_coeff_file));
            }
            file_loader_ptr->T_obj_LiDAR_queue.push(file_loader_ptr->T_obj_LiDAR);
            file_loader_ptr->conformal_map_queue.push(file_loader_ptr->conformal_map);
            file_loader_ptr->sph_coeff_queue.push(file_loader_ptr->sph_coeff);
        }
        m_queue_lock.unlock();
    }
    // while (true) {
    //     std::cout << "I am a thread" << std::endl;
    // }
}

template <typename BasicType, typename SemanticType>
void add_to_queue_ref(FileReader<BasicType, SemanticType> &file_loader) {
    typedef typename FileReader<BasicType, SemanticType>::DynamicMatrix DynamicMatrix;
    DynamicMatrix (*read_cloud)(std::string filename);
    if (file_loader.scan_file_type == "txt") {
        read_cloud = &FileReaderBase::read_xyz<BasicType>;
    } else if (file_loader.scan_file_type == "ply") {
        read_cloud = &FileReaderBase::read_ply<BasicType>;
    } else if (file_loader.scan_file_type == "pcd") {
        read_cloud = &FileReaderBase::read_pcd<BasicType>;
    } else {
        std::cerr << "Only support: 'txt', 'ply', and 'pcd'" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if (file_loader.is_pose) {
        file_loader.poses_filename = file_loader.poses_dir + "." + file_loader.poses_file_type;
        file_loader.poses_gt = FileReaderBase::read_txt_file<BasicType>(file_loader.poses_filename);
    }
    if (file_loader.data_set == "SIMULATOR_OPTIMIZATION") {
        file_loader.odometry = FileReaderBase::read_txt_file<BasicType>(file_loader.odometry_file);
        file_loader.odometry_noised =
            FileReaderBase::read_txt_file<BasicType>(file_loader.odometry_noised_file);
        file_loader.trajectory =
            FileReaderBase::read_txt_file<BasicType>(file_loader.trajectory_file);
        file_loader.trajectory_noised =
            FileReaderBase::read_txt_file<BasicType>(file_loader.trajectory_noised_file);
    }
    for (int i = file_loader.start_frame; i <= file_loader.end_frame; ++i) {
        m_queue_lock.lock();
        file_loader.scans_filename =
            file_loader.scans_dir + "/" + std::to_string(i) + "." + file_loader.scan_file_type;
        file_loader.scans_queue.push(((*read_cloud)(file_loader.scans_filename).array() *
                                      static_cast<BasicType>(file_loader.scan_scale_factor)));
        if (file_loader.is_semantic) {
            file_loader.semantics_filename = file_loader.semantics_dir + "/" + std::to_string(i) +
                                             "." + file_loader.semantic_file_type;
            file_loader.semantics_queue.push(
                FileReaderBase::read_txt_file<SemanticType>(file_loader.semantics_filename));
        }
        if (file_loader.data_set == "SIMULATOR_OPTIMIZATION") {
            file_loader.T_obj_LiDAR.clear();
            file_loader.conformal_map.clear();
            file_loader.sph_coeff.clear();
            file_loader.m_x_Gx_vec.clear();
            file_loader.m_x_Gy_vec.clear();
            file_loader.m_y_Gx_vec.clear();
            file_loader.m_y_Gy_vec.clear();
            file_loader.valid_idx_vec.clear();
            struct stat info;
            for (int j = 1; j <= file_loader.obj_num; ++j) {
                std::string T_obj_LiDAR_file = file_loader.transformation_obj_LiDAR_dir + "/" +
                                               std::to_string(i) + "_obj_" + std::to_string(j) +
                                               ".txt";
                std::string conformal_map_file = file_loader.conformal_map_dir + "/" +
                                                 std::to_string(i) + "_obj_" + std::to_string(j) +
                                                 ".txt";
                std::string sph_coeff_file = file_loader.sph_coeff_dir + "/" + std::to_string(i) +
                                             "_obj_" + std::to_string(j) + ".txt";
                std::string m_x_Gx_file = file_loader.gamma_gradient_dir + "/m_x_Gx_" +
                                          std::to_string(i) + "_obj_" + std::to_string(j) + ".txt";
                std::string m_x_Gy_file = file_loader.gamma_gradient_dir + "/m_x_Gy_" +
                                          std::to_string(i) + "_obj_" + std::to_string(j) + ".txt";
                std::string m_y_Gx_file = file_loader.gamma_gradient_dir + "/m_y_Gx_" +
                                          std::to_string(i) + "_obj_" + std::to_string(j) + ".txt";
                std::string m_y_Gy_file = file_loader.gamma_gradient_dir + "/m_y_Gy_" +
                                          std::to_string(i) + "_obj_" + std::to_string(j) + ".txt";
                std::string valid_idx_file = file_loader.valid_idx_dir + "/" + std::to_string(i) +
                                             "_obj_" + std::to_string(j) + ".txt";
                file_loader.T_obj_LiDAR.push_back(
                    FileReaderBase::read_txt_file<BasicType>(T_obj_LiDAR_file));
                file_loader.conformal_map.push_back(
                    FileReaderBase::read_txt_file<BasicType>(conformal_map_file));
                file_loader.sph_coeff.push_back(
                    FileReaderBase::read_txt_file<BasicType>(sph_coeff_file));
                file_loader.m_x_Gx_vec.push_back(
                    FileReaderBase::read_txt_file<BasicType>(m_x_Gx_file));
                file_loader.m_x_Gy_vec.push_back(
                    FileReaderBase::read_txt_file<BasicType>(m_x_Gy_file));
                file_loader.m_y_Gx_vec.push_back(
                    FileReaderBase::read_txt_file<BasicType>(m_y_Gx_file));
                file_loader.m_y_Gy_vec.push_back(
                    FileReaderBase::read_txt_file<BasicType>(m_y_Gy_file));
                if (stat(valid_idx_file.c_str(), &info) == 0) {
                    file_loader.valid_idx_vec.push_back(
                        FileReaderBase::read_txt_file<BasicType>(valid_idx_file)
                            .template cast<SemanticType>());
                } else {
                    file_loader.valid_idx_vec.push_back(
                        Eigen::Matrix<SemanticType, Eigen::Dynamic, Eigen::Dynamic>());
                }
            }
            file_loader.T_obj_LiDAR_queue.push(file_loader.T_obj_LiDAR);
            file_loader.conformal_map_queue.push(file_loader.conformal_map);
            file_loader.sph_coeff_queue.push(file_loader.sph_coeff);
            file_loader.m_x_Gx_queue.push(file_loader.m_x_Gx_vec);
            file_loader.m_x_Gy_queue.push(file_loader.m_x_Gy_vec);
            file_loader.m_y_Gx_queue.push(file_loader.m_y_Gx_vec);
            file_loader.m_y_Gy_queue.push(file_loader.m_y_Gy_vec);
            file_loader.valid_idx_queue.push(file_loader.valid_idx_vec);
        }
        m_queue_lock.unlock();
    }
}

#endif // EXAMPLE_FILE_READER