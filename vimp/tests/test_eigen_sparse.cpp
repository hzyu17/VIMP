/**
 * @file test_eigen_sparse.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test sparse matrices in eigen, compare the speed with full matrices.
 * @version 0.1
 * @date 2023-01-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include<Eigen/Sparse>
#include<Eigen/Dense>
#include<gtest/gtest.h>
#include"../helpers/timer.h"
#include"../helpers/eigen_wrappers.h"

typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> T;

/**
 * @brief Test initialization of a sparse matrix.
 */
TEST(TestSparse, initialization){
    int m = 4; // number of rows and cols
    int n = m*m; // the matrix dimension

    MatrixClass matrix_class;

    Timer timer;

    timer.start();
    
    Matrix rand_mat = matrix_class.random_matrix(n, n);
    SpMat rand_spmat = matrix_class.random_sparse_matrix(n, n, 4);

    timer.end();

}

int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}