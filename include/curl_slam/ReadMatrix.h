//
// Created by zkc on 05/04/2022.
//

#ifndef EXAMPLE_READMATRIX_H
#define EXAMPLE_READMATRIX_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <vector>

template <typename T> class ReadMatrix {
  public:
    int rows;
    int cols;
    T matrix;

    explicit ReadMatrix(const std::string &read_dir) {
        std::ifstream infile;
        std::string STRING, ITEM;
        std::vector<std::vector<double>> vecMatrix;
        std::vector<double> vec;
        infile.open(read_dir);
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
            vecMatrix.push_back(vec);
        }
        infile.close();
        rows = vecMatrix.size();
        cols = vecMatrix[0].size();
        matrix.resize(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                matrix(i, j) = vecMatrix[i][j];
            }
        }
    }
};

#endif // EXAMPLE_READMATRIX_H
