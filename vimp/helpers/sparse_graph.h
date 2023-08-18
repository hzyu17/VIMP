/**
 * @file graph.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief A class for sparse graph definition, covariance initialization, and manipulations.
 * @version 0.1
 * @date 2023-01-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "EigenWrapper.h"

namespace vimp{
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

} // namespace vimp
