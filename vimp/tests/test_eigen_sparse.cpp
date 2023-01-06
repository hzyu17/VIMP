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

#include<Eigen/SparseCholesky>
#include<Eigen/IterativeLinearSolvers>


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

    MatrixClass m_class;

    Timer timer;

    timer.start();
    Matrix rand_mat = m_class.random_matrix(n, n);
    // m_class.print_matrix(rand_mat);
    std::cout << "full matrix init" << std::endl;
    timer.end();

    timer.start();
    SpMat rand_spmat = m_class.random_sparse_matrix(n, n, 10);
    // m_class.print_spmatrix(rand_spmat);
    std::cout << "sparse matrix init" << std::endl;
    timer.end();

}


TEST(TestSparse, computation_time){
    Timer timer;

    int m=4;
    int n=m*m;

    MatrixClass m_class;

    SpMat spm1 = m_class.random_sparse_matrix(n, n, 4);
    SpMat spm2 = m_class.random_sparse_matrix(n, n, 4);
    SpMat spm3 = m_class.random_sparse_matrix(n, n, 4);

    Matrix m1(spm1);
    Matrix m2(spm2);
    Matrix m3(n, n);

    timer.start();
    spm3 = spm1 * spm2;
    std::cout << "sparse multiplication " << std::endl;
    timer.end();
    
    timer.start();
    m3 = m1 * m2;
    std::cout << "full multiplication " << std::endl;
    timer.end();
}

TEST(TestSparse, solve_psd_eqn){
    Timer timer;

    int m=4;
    int n=m*m;

    MatrixClass m_class;

    SpMat spm = m_class.randomd_sparse_psd(n, 10);
    m_class.print_spmatrix(spm);

    ASSERT_TRUE(m_class.is_sparse_positive(spm));

    // RHS
    Vector b(m_class.random_vector(n));

    // solve full
    Matrix fullm(spm);
    timer.start();
    Vector x = fullm.llt().solve(b);
    std::cout << "solving time full matrix " << std::endl;
    timer.end();

    m_class.print_matrix(x);

    // solve sparse
    timer.start();
    Eigen::ConjugateGradient<SpMat, Eigen::Upper> solver;
    Vector spx = solver.compute(spm).solve(b);

    // Eigen::SimplicialLLT<SpMat> sp_llt(spm);
    // Vector sp_x = sp_llt.solve(b);
    std::cout << "solving time sparse matrix " << std::endl;
    timer.end();

    m_class.print_matrix(spx);

    ASSERT_LE((x - spx).norm(), 1e-7);

}

TEST(TestSparse, ldlt_decomp){
    ASSERT_TRUE(true);
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}