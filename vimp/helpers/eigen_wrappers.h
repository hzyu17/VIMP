/**
 * @file eigen_wrappers.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Wrappers for Eigen libraries
 * @version 0.1
 * @date 2023-01-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include<Eigen/Dense>
#include<Eigen/Sparse>
#include"random.h"

typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> T;

class MatrixClass{
public:
    MatrixClass(){};
    
    Matrix random_matrix(int m, int n){
        return Matrix::Random(m ,n);
    }

    SpMat random_sparse_matrix(int m, int n, int nnz){
        std::vector<T> tripletList;
        tripletList.reserve(nnz);

        Random random;

        for (int i=0; i<nnz; i++){
            int i_row = random.randint(0, m);
            int j_col = random.randint(0, n);
            double val = random.rand_double(0.0, 10.0);
            tripletList.push_back(T(i_row, j_col, val));
        }

        SpMat mat(m, n);
        mat.setFromTriplets(tripletList.begin(), tripletList.end());

        return mat;
    }
};