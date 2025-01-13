/**
 * @file FileReaderBase.h
 * @author zkc (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef EXMAPLE_FILE_READER_BASE
#define EXMAPLE_FILE_READER_BASE

#include "load_config.h"
#include "lock_manager.h"
#include <Eigen/Core>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <queue>
#include <string>
#include <utility>

class FileReaderBase {
  public:
    std::string root_file_0;
    std::string root_file_1;
    std::string root_file_2;

    FileReaderBase() = default;
    explicit FileReaderBase(std::string _root_file_0) : root_file_0(std::move(_root_file_0)) {}
    FileReaderBase(std::string _root_file_0, std::string _root_file_1)
        : root_file_0(std::move(_root_file_0)), root_file_1(std::move(_root_file_1)) {}
    FileReaderBase(std::string _root_file_0, std::string _root_file_1, std::string _root_file_2)
        : root_file_0(std::move(_root_file_0)), root_file_1(std::move(_root_file_1)),
          root_file_2(std::move(_root_file_2)) {}

    // read matrix from txt ascii file
    template <typename T>
    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
    read_txt_file(const std::string &filename, int col_value = std::numeric_limits<int>::max()) {
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrix;
        std::ifstream infile;
        std::string STRING, ITEM;
        std::vector<std::vector<T>> vecMatrix;
        std::vector<T> vec;
        infile.open(filename);
        while (!infile.eof()) // To get you all the lines.
        {
            std::getline(infile, STRING); // Saves the line in STRING.
            if (STRING.empty()) {
                break;
            }
            std::stringstream ss(STRING);
            vec.clear();
            while (ss >> ITEM) {
                vec.push_back(std::stod(ITEM));
            }
            if (vec.size() > col_value) {
                std::cerr << "Column of matrix larger than the requirements" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            vecMatrix.push_back(vec);
        }
        infile.close();
        matrix.resize(int(vecMatrix.size()), int(vecMatrix[0].size()));
        for (int i = 0; i < int(vecMatrix.size()); ++i) {
            for (int j = 0; j < int(vecMatrix[0].size()); ++j) {
                matrix(i, j) = vecMatrix[i][j];
            }
        }
        return matrix;
    }

    // read matrix from txt ascii file save as the vector
    template <typename BasicType>
    static std::vector<BasicType>
    read_vector_txt_file(const std::string &filename,
                         int col_value = std::numeric_limits<int>::max()) {
        Eigen::Matrix<BasicType, Eigen::Dynamic, Eigen::Dynamic> matrix;
        std::ifstream infile;
        std::string STRING, ITEM;
        std::vector<std::vector<BasicType>> vecMatrix;
        std::vector<BasicType> vec;
        infile.open(filename);
        while (!infile.eof()) // To get you all the lines.
        {
            std::getline(infile, STRING); // Saves the line in STRING.
            if (STRING.empty()) {
                break;
            }
            std::stringstream ss(STRING);
            vec.clear();
            while (ss >> ITEM) {
                vec.push_back(std::stod(ITEM));
            }
            if (vec.size() > col_value) {
                std::cerr << "Column of matrix larger than the requirements" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            vecMatrix.push_back(vec);
        }
        infile.close();
        matrix.resize(int(vecMatrix.size()), int(vecMatrix[0].size()));
        for (int i = 0; i < int(vecMatrix.size()); ++i) {
            for (int j = 0; j < int(vecMatrix[0].size()); ++j) {
                matrix(i, j) = vecMatrix[i][j];
            }
        }
        if (matrix.cols() == 1) {
            vec.clear();
            for (int i = 0; i < matrix.rows(); ++i) {
                vec.push_back(matrix(i, 0));
            }
            return vec;
        } else {
            std::cout << "Not a vector" << std::endl;
            return vec;
        }
    }

    // save matrix to ascii txt file
    template <typename T>
    static void write_txt_file(const std::string &filename,
                               Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrix) {
        std::ofstream file;
        file.open(filename);
        file.precision(17);
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                file << matrix(i, j) << ' ';
            }
            file << '\n';
        }
        file.close();
    }
    // save matrix to ascii vector txt file
    template <typename T>
    static void write_vector_txt_file(const std::string &filename, std::vector<T> vec) {
        std::ofstream file;
        file.open(filename);
        file.precision(17);
        for (int i = 0; i < vec.size(); ++i) {
            file << vec[i] << '\n';
        }
        file.close();
    }
    // read point clouds
    template <typename T>
    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> read_xyz(std::string filename) {
        return read_txt_file<T>(filename, 3);
    }
    template <typename T>
    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> read_ply(std::string filename) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPLYFile(filename, *cloud);
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrix(cloud->size(), 3);
        for (int i = 0; i < cloud->size(); ++i) {
            matrix(i, 0) = cloud->points[i].x;
            matrix(i, 1) = cloud->points[i].y;
            matrix(i, 2) = cloud->points[i].z;
        }
        return matrix;
    }
    template <typename T>
    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> read_pcd(std::string filename) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(filename, *cloud);
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrix(cloud->size(), 3);
        for (int i = 0; i < cloud->size(); ++i) {
            matrix(i, 0) = cloud->points[i].x;
            matrix(i, 1) = cloud->points[i].y;
            matrix(i, 2) = cloud->points[i].z;
        }
        return matrix;
    }
};

#endif // EXMAPLE_FILE_READER_BASE