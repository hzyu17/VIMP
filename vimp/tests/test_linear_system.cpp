/**
 * @file test_eigen_sparse.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test linear system (A, B) in eigen.
 * @version 0.1
 * @date 2023-02-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include<gtest/gtest.h>

#include"helpers/timer.h"
#include"helpers/EigenWrapper.h"
#include"gp/linear_dynamics.h"

#include<Eigen/IterativeLinearSolvers>


typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SpMat; // declares a col-major sparse matrix type of double
typedef Eigen::Triplet<double> T;

using namespace vimp;
using namespace Eigen;
EigenWrapper eigen_wrapper;
Timer timer;
MatrixIO m_io;

/**
 * @brief Test initialization of a sparse matrix.
 */
TEST(TestLinSys, initialization){
    // ground truth matrices
    int state_dim = 4;
    MatrixXd A_0{MatrixXd::Zero(state_dim, state_dim)};
    MatrixXd block{MatrixXd::Identity(2, 2)};
    eigen_wrapper.block_insert(A_0, 0, 2, 2, 2, block);

    double t = 0.3;
    MatrixXd ground_truth_Phi = A_0 + MatrixXd::Identity(state_dim, state_dim);
    ground_truth_Phi = ground_truth_Phi * t;

    LinearSystem sys(A_0);

    ASSERT_LE((sys.Phi(t) - ground_truth_Phi).norm(), 1e-10);
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}