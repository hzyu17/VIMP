/**
 * @file MatrixHelper.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Define some special matrix and vector classes.
 * @version 0.1
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "common_includes.h"
// #include "EigenWrapper.h"


using namespace Eigen;

namespace vimp{

class Matrix3D : public MatrixXd{
public:
    Matrix3D(){}
    Matrix3D(int row, int col, int nt):MatrixXd(row*col, nt) {}    
    Matrix3D(const Matrix3D& mat): Eigen::MatrixXd(mat) {}
    Matrix3D(const MatrixXd & mat): MatrixXd(mat){}

    // Overloaded assignment operator that takes an Eigen::MatrixXd as input
    Matrix3D& operator=(const Eigen::MatrixXd& other) {
        Eigen::MatrixXd::operator=(other); // call base class assignment operator
        // additional custom logic for MyMatrix
        return *this;
    }
    
};

class Vector3D : public VectorXd{
public:
    Vector3D(){}
    Vector3D(int row, int nt):VectorXd(row, nt){}   

};

class MatrixIO{
    public:
        
        MatrixIO(){}

        template <typename T>
        void saveData(const std::string& fileName, const T& matrix, bool verbose=true) const{
            if (verbose){
                std::cout << "Saving data to: " << fileName << std::endl;
            }
            std::ofstream file(fileName);
            if (file.is_open()){
                file << matrix.format(CSVFormat);
                file.close();
            }
        }

        /**
         * @brief read a sdf map from csv file, which can be the output of some gpmp2 functions.
         * modified from an online code piece.
         * @param filename 
         * @return Matrix 
         */
        /// https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix

        Eigen::MatrixXd load_csv (const std::string & path) {
            std::ifstream indata;
            indata.open(path);
            if (indata.peek() == std::ifstream::traits_type::eof()){
                throw std::runtime_error(std::string("File dose not exist ...: ") + path);
            }
            
            std::string line;
            std::vector<double> values;
            uint rows = 0;
            while (std::getline(indata, line)) {
                std::stringstream lineStream(line);
                std::string cell;
                while (std::getline(lineStream, cell, ',')) {
                    values.push_back(std::stod(cell));
                }
                rows++;
            }
            return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
        }

    };


struct Block{
public:
    Block(){}
    Block(int start_row, int start_col, int nrows, int ncols):
    _start_row(start_row),
    _start_col(start_col),
    _nrows(nrows),
    _ncols(ncols){}

    int row(){ return _start_row;}

    int nrows(){ return _nrows;}

    int col(){ return _start_col;}

    int ncols(){ return _ncols;}

    int _start_row;
    int _start_col;
    int _nrows;
    int _ncols;
    
}; // struct Block

class TrajectoryBlock{

public:
    TrajectoryBlock(){}

    TrajectoryBlock(int state_dim, int num_states, int start_index, int block_length):
    _state_dim(state_dim),
    _num_states(num_states),
    _start_index(start_index),
    _block_length(block_length){
        _block = Block{_start_index*state_dim, _start_index*state_dim, _block_length, _block_length};
    };

    SpMat extract(const SpMat & m){
        return m.middleRows(_block.row(), _block.nrows()).middleCols(_block.col(), _block.ncols());
    }

    Eigen::VectorXd extract_vector(const Eigen::VectorXd & vec){
        return vec.block(_block.row(), 0, _block.nrows(), 1);
    }

    void fill(Eigen::MatrixXd & block, SpMat & matrix){
        Eigen::MatrixXd mat_full{matrix};
        mat_full.block(_block.row(), _block.col(), _block.nrows(), _block.ncols()) = block;
        matrix = mat_full.sparseView();
    }

    void fill_vector(Eigen::VectorXd & vec, const Eigen::VectorXd & vec_block){
        vec.setZero();
        vec.block(_block.row(), 0, _block.nrows(), 1) = vec_block;
    }

    double start_element(){
        return _start_index*_state_dim;
    }

    double block_length(){
        return _block_length;
    }

    void print(){
        std::cout << "(starting index, block length): " << "(" << _start_index << ", " << _block_length << ")" << std::endl;
    }

private:
    int _state_dim;
    int _num_states;
    int _start_index, _block_length;
    Block _block = Block();

}; // class TrajectoryBlock

}