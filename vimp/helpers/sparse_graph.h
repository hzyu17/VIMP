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

#include "square_block.h"
#include "eigen_wrapper.h"
typedef Eigen::SparseMatrix<double> SpMat; 

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
};

class TrajectoryGraph{
public:
    TrajectoryGraph(){};
    TrajectoryGraph(int state_dim, int num_state): _state_dim(state_dim),
                                                   _num_states(num_state), 
                                                   _graph_dim(state_dim*num_state), 
                                                   _graph_precision_matrix(_graph_dim, _graph_dim),
                                                   _graph_covariance(_graph_dim, _graph_dim)
    {
        _graph_precision_matrix.setZero();
        _graph_covariance.setZero();
        // fill in the precision matrix to the known sparsity pattern
        for (int i=0; i<num_state-1; i++){
            Eigen::MatrixXd block = Eigen::MatrixXd::Ones(2*state_dim, 2*state_dim) * 0.01;
            _eigen_wrapper.block_insert_sparse(_graph_precision_matrix, i*state_dim, i*state_dim, 2*state_dim, 2*state_dim, block);
        }

    }

    SpMat precision_matrix(){
        return _graph_precision_matrix;
    }

    SpMat covariance_matrix(){
        return _graph_covariance;
    }

    bool precision_matrix_eq(const Eigen::MatrixXd& spm){
        return _eigen_wrapper.matrix_equal(spm, _graph_precision_matrix);
    }

    void inverse_precision(){
        _graph_covariance.setZero();
        _eigen_wrapper.inv_sparse(_graph_precision_matrix, _graph_covariance, _I, _J);
    }

    void set_precision(const SpMat & precision){
        _graph_precision_matrix = precision;
        find_nnz_tables();
    }

    void find_nnz_tables(){
        // LDLT, get the pattern in L.
        SparseLDLT ldlt_sp(_graph_precision_matrix);
        SpMat Lsp = ldlt_sp.matrixL(); 
        _eigen_wrapper.print_spmatrix(Lsp);
        Eigen::VectorXd V;
        _eigen_wrapper.find_nnz(Lsp, _I, _J, V);
    }

    SpMat covariance_block(const Block& block_index){
        return _eigen_wrapper.block_extract_sparse(_graph_covariance, 
                                                    block_index._start_row, block_index._start_col, 
                                                    block_index._nrows, block_index._ncols);
    }
    

private:
    int _state_dim;
    int _num_states;
    int _graph_dim;
    EigenWrapper _eigen_wrapper = EigenWrapper();
    SpMat _graph_precision_matrix;
    SpMat _graph_covariance;
    Eigen::VectorXd _I, _J; // NNZ element positions in L
};

} //namespace vimp
