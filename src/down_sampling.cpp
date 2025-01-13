//
// Created by zkc on 25/08/23.
//
#include "FileReaderBase.h"
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

template <typename T>
std::vector<Eigen::MatrixX<T>> divide_patches(const Eigen::MatrixX<T> &points, T leaf_size_x, T leaf_size_y,
                                              T leaf_size_z) {
    if (points.cols() != 3) {
        throw std::invalid_argument("Input should be an Nx3 matrix for point clouds.");
    }
    if (leaf_size_x <= 0 || leaf_size_y <= 0 || leaf_size_z <= 0) {
        throw std::invalid_argument("Leaf size should be non-negative");
    }
    std::vector<Eigen::MatrixX<T>> patch_vec;
    T min_x = points.col(0).minCoeff();
    T min_y = points.col(1).minCoeff();
    T min_z = points.col(2).minCoeff();
    T max_x = points.col(0).maxCoeff();
    T max_y = points.col(1).maxCoeff();
    T max_z = points.col(2).maxCoeff();

    auto x_seg_num = static_cast<unsigned int>(std::ceil((max_x - min_x) / leaf_size_x));
    auto y_seg_num = static_cast<unsigned int>(std::ceil((max_y - min_y) / leaf_size_y));
    auto z_seg_num = static_cast<unsigned int>(std::ceil((max_z - min_z) / leaf_size_z));

    std::uint64_t total_voxels = static_cast<std::uint64_t>(x_seg_num) * static_cast<std::uint64_t>(y_seg_num) *
                                 static_cast<std::uint64_t>(z_seg_num);
    if (total_voxels < std::numeric_limits<unsigned int>::max()) {
        std::vector<std::vector<unsigned int>> patch_idx_vec(total_voxels);
        T new_x_cut_threshold = (max_x - min_x) / static_cast<T>(x_seg_num);
        T new_y_cut_threshold = (max_y - min_y) / static_cast<T>(y_seg_num);
        T new_z_cut_threshold = (max_z - min_z) / static_cast<T>(z_seg_num);
        for (int i = 0; i < points.rows(); ++i) {
            unsigned int x_idx = std::floor((points(i, 0) - min_x) / new_x_cut_threshold);
            x_idx = std::min(x_idx, x_seg_num - 1);

            unsigned int y_idx = std::floor((points(i, 1) - min_y) / new_y_cut_threshold);
            y_idx = std::min(y_idx, y_seg_num - 1);

            unsigned int z_idx = std::floor((points(i, 2) - min_z) / new_z_cut_threshold);
            z_idx = std::min(z_idx, z_seg_num - 1);

            unsigned int grid_index = z_idx * (x_seg_num * y_seg_num) + y_idx * x_seg_num + x_idx;
            patch_idx_vec[grid_index].push_back(i);
        }
        patch_vec.reserve(total_voxels);
        for (const auto &patch_idx : patch_idx_vec) {
            if (!patch_idx.empty()) {
                patch_vec.emplace_back(points(patch_idx, Eigen::all));
            }
        }

    } else {
        throw std::invalid_argument("Index overflow!");
    }

    return patch_vec;
}

template <typename T>
std::vector<Eigen::MatrixX<T>> voxel_grid_downsampling_recursive(const std::vector<Eigen::MatrixX<T>> &points_vec,
                                                                 T leaf_size_x, T leaf_size_y, T leaf_size_z) {
    if (points_vec.empty()) {
        throw std::invalid_argument("Input should be non-empty points vectors");
    }
    if (leaf_size_x <= 0 || leaf_size_y <= 0 || leaf_size_z <= 0) {
        throw std::invalid_argument("Leaf size should be non-negative");
    }
    std::vector<Eigen::MatrixX<T>> downsampled_points_vec;
    for (const auto &points : points_vec) {
        if (points.size() == 0) {
            std::cout << "jump this vector" << std::endl;
            continue;
        }
        if (points.cols() != 3) {
            throw std::invalid_argument("Input should be an Nx3 matrix for point clouds.");
        }
        T min_x = points.col(0).minCoeff();
        T min_y = points.col(1).minCoeff();
        T min_z = points.col(2).minCoeff();
        T max_x = points.col(0).maxCoeff();
        T max_y = points.col(1).maxCoeff();
        T max_z = points.col(2).maxCoeff();

        auto x_seg_num = static_cast<unsigned int>(std::ceil((max_x - min_x) / leaf_size_x));
        x_seg_num = std::max(static_cast<unsigned int>(1), x_seg_num);
        auto y_seg_num = static_cast<unsigned int>(std::ceil((max_y - min_y) / leaf_size_y));
        y_seg_num = std::max(static_cast<unsigned int>(1), y_seg_num);
        auto z_seg_num = static_cast<unsigned int>(std::ceil((max_z - min_z) / leaf_size_z));
        z_seg_num = std::max(static_cast<unsigned int>(1), z_seg_num);

        std::uint64_t total_voxels = static_cast<std::uint64_t>(x_seg_num) * static_cast<std::uint64_t>(y_seg_num) *
                                     static_cast<std::uint64_t>(z_seg_num);
        if (total_voxels < std::numeric_limits<unsigned int>::max() / 2) {
            std::vector<std::vector<unsigned int>> patch_idx_vec(static_cast<unsigned int>(total_voxels));

            T new_x_cut_threshold = (max_x - min_x) / static_cast<T>(x_seg_num);
            T new_y_cut_threshold = (max_y - min_y) / static_cast<T>(y_seg_num);
            T new_z_cut_threshold = (max_z - min_z) / static_cast<T>(z_seg_num);

            for (unsigned int i = 0; i < points.rows(); ++i) {
                unsigned int x_idx = std::floor((points(i, 0) - min_x) / new_x_cut_threshold);
                x_idx = std::min(x_idx, x_seg_num - 1);

                unsigned int y_idx = std::floor((points(i, 1) - min_y) / new_y_cut_threshold);
                y_idx = std::min(y_idx, y_seg_num - 1);

                unsigned int z_idx = std::floor((points(i, 2) - min_z) / new_z_cut_threshold);
                z_idx = std::min(z_idx, z_seg_num - 1);

                unsigned int grid_index = z_idx * (x_seg_num * y_seg_num) + y_idx * x_seg_num + x_idx;
                patch_idx_vec[grid_index].push_back(i);
            }
            std::vector<Eigen::RowVector3<T>> downsampled_data;
            for (const auto &patch_idx : patch_idx_vec) {
                if (!patch_idx.empty()) {
                    Eigen::RowVector3<T> mean_point = points(patch_idx, Eigen::all).colwise().mean();
                    downsampled_data.push_back(mean_point);
                }
            }
            Eigen::MatrixX<T> down_sampled_points(downsampled_data.size(), 3);
            for (size_t i = 0; i < downsampled_data.size(); ++i) {
                down_sampled_points.row(i) = downsampled_data[i];
            }
            downsampled_points_vec.push_back(down_sampled_points);
        } else {
            // Determine which dimension is the longest
            T max_dim_length = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
            T sub_leaf_size_x = leaf_size_x;
            T sub_leaf_size_y = leaf_size_y;
            T sub_leaf_size_z = leaf_size_z;
            if (max_dim_length == (max_x - min_x)) {
                sub_leaf_size_x = max_dim_length / 2;
            } else if (max_dim_length == (max_y - min_y)) {
                sub_leaf_size_y = max_dim_length / 2;
            } else {
                sub_leaf_size_z = max_dim_length / 2;
            }
            std::vector<Eigen::MatrixX<T>> sub_points_vec =
                divide_patches<T>(points, sub_leaf_size_x, sub_leaf_size_y, sub_leaf_size_z);
            // Call the function recursively with the adjusted leaf size
            return voxel_grid_downsampling_recursive(sub_points_vec, leaf_size_x, leaf_size_y, leaf_size_z);
        }
    }
    return downsampled_points_vec;
}

template <typename T>
Eigen::MatrixX<T> voxel_grid_downsampling(const Eigen::MatrixX<T> &input_points, T leaf_size_x, T leaf_size_y,
                                          T leaf_size_z) {
    std::vector<Eigen::MatrixX<T>> input_points_vec;
    input_points_vec.push_back(input_points);
    std::vector<Eigen::MatrixX<T>> output_points_vec =
        voxel_grid_downsampling_recursive<T>(input_points_vec, leaf_size_x, leaf_size_y, leaf_size_z);
    unsigned int points_counter = 0;
    for (const auto &pt_matrix : output_points_vec) {
        points_counter += pt_matrix.rows();
    }
    Eigen::MatrixX<T> output_points(points_counter, 3);
    points_counter = 0;
    for (const auto &pt_matrix : output_points_vec) {
        output_points(Eigen::seq(points_counter, pt_matrix.rows() - 1), Eigen::all) = pt_matrix;
        points_counter += pt_matrix.rows();
    }
    return output_points;
}

template <typename T> std::string floatToString(T value) {
    std::string result = std::to_string(value);

    // Remove trailing zeros
    while (result.size() > 0 && result.back() == '0') {
        result.pop_back();
    }

    // If the last character is a dot, remove it
    if (result.size() > 0 && result.back() == '.') {
        result.pop_back();
    }
    return result;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        throw std::invalid_argument("Please provide filename as the first argument");
    }
    std::string input_points_file = argv[1];
    std::cout << "start reading" << std::endl;
    Eigen::MatrixX<float> input_points = FileReaderBase::read_xyz<float>(input_points_file);
    std::vector<Eigen::MatrixX<float>> input_points_vec;
    input_points_vec.push_back(input_points);
    std::vector<float> leaf_size_vec{0.0075, 0.01, 0.025, 0.05, 0.075, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4};
    for (const auto &leaf_size : leaf_size_vec) {
        std::cout << "start downsampling" << std::endl;
        std::vector<Eigen::MatrixX<float>> output_points_vec =
            voxel_grid_downsampling_recursive<float>(input_points_vec, leaf_size, leaf_size, leaf_size);
        // write the output file
        std::cout << "start saving" << std::endl;
        std::string output_points_file =
            input_points_file.substr(0, input_points_file.size() - 4) + "_" + floatToString(leaf_size) + ".txt";
        std::ofstream outfile(output_points_file);
        outfile.precision(std::numeric_limits<float>::max_digits10);
        for (const auto &points : output_points_vec) {
            for (int j = 0; j < points.rows(); ++j) {
                outfile << points(j, 0) << " " << points(j, 1) << " " << points(j, 2) << "\n";
            }
        }
    }
}