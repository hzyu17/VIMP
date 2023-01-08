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
#include"../helpers/data_io.h"

#include<Eigen/SparseCholesky>
#include<Eigen/IterativeLinearSolvers>


typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SpMat; // declares a col-major sparse matrix type of double
typedef Eigen::Triplet<double> T;

using namespace vimp;
EigenWrapper eigen_wrapper;
Timer timer;
MatrixIO m_io;

// ================== read ground truth matrices ==================
Matrix precision = m_io.load_csv("precision.csv");
SpMat precision_sp = precision.sparseView();
Matrix cov = m_io.load_csv("expected_cov.csv");
SpMat cov_sp = cov.sparseView();
Matrix D_true = m_io.load_csv("expected_D.csv");
Matrix L_true = m_io.load_csv("expected_L.csv");
typedef Eigen::SimplicialLDLT<SpMat, Eigen::Lower, Eigen::NaturalOrdering<int>> SparseLDLT;

/**
 * @brief Test initialization of a sparse matrix.
 */
TEST(TestSparse, initialization){
    int m = 4; // number of rows and cols
    int n = m*m; // the matrix dimension

    timer.start();
    Matrix rand_mat = eigen_wrapper.random_matrix(n, n);
    std::cout << "full matrix init" << std::endl;
    timer.end();

    // by copy
    timer.start();
    SpMat rand_spmat = eigen_wrapper.random_sparse_matrix(n, n, 10);
    std::cout << "sparse matrix init by copy" << std::endl;
    timer.end();

    // by reference
    SpMat rand_spm_ref(n, n);
    timer.start();
    eigen_wrapper.random_sparse_matrix(rand_spm_ref, n, n, 10);
    std::cout << "sparse matrix init by reference" << std::endl;
    timer.end();

}


TEST(TestSparse, computation_time){
    int m=4;
    int n=m*m;

    SpMat spm1 = eigen_wrapper.random_sparse_matrix(n, n, 4);
    SpMat spm2 = eigen_wrapper.random_sparse_matrix(n, n, 4);
    SpMat spm3 = eigen_wrapper.random_sparse_matrix(n, n, 4);

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

/**
 * @brief test the time of solving equations
 */
TEST(TestSparse, solve_psd_eqn){
    int m=4;
    int n=m*m;

    SpMat spm = eigen_wrapper.randomd_sparse_psd(n, 10) + eigen_wrapper.sp_eye(n);
    
    ASSERT_TRUE(eigen_wrapper.is_sparse_positive(spm));

    // RHS
    Vector b(eigen_wrapper.random_vector(n));

    // solve full
    Matrix fullm(spm);
    timer.start();
    Vector x = eigen_wrapper.solve_llt(fullm, b);
    std::cout << "solving time full matrix " << std::endl;
    timer.end();

    // solve sparse
    timer.start();
    Vector spx = eigen_wrapper.solve_cgd_sp(spm, b);
    std::cout << "solving time sparse matrix " << std::endl;
    timer.end();

    ASSERT_LE((x - spx).norm(), 1e-7);

}

/**
 * @brief test ldlt decomposition and time
 */
TEST(TestSparse, ldlt_decomp){
    ASSERT_LE((precision - L_true*D_true*L_true.transpose()).norm(), 1e-10);
    
    const Eigen::Index size = precision.rows();
    Matrix eye(size,size);
    eye.setIdentity();

    ASSERT_LE((precision - precision_sp).norm(), 1e-10);

    // ================== full ldlt decomposition ==================
    auto ldlt_full = eigen_wrapper.ldlt_full(precision);
    Matrix Lf = ldlt_full.matrixL();
    Matrix Uf = ldlt_full.matrixU();
    Matrix Df = ldlt_full.vectorD().real().asDiagonal();
    auto Pf = ldlt_full.transpositionsP();
    ASSERT_LE((precision - ldlt_full.reconstructedMatrix()).norm(), 1e-10);

    // reconstruction
    Matrix res(size,size);
    // important: the transpositions matrix must operate on another matrix.
    res = Pf * eye; 
    // L^* P
    res = Uf * res;
    // D(L^*P)
    res = Df * res;
    // L(DL^*P)
    res = Lf * res;
    // P^T (LDL^*P)
    res = Pf.transpose() * res;

    ASSERT_LE((ldlt_full.reconstructedMatrix() - res).norm(), 1e-10);

    // ================== sparse ldlt decomposition ==================
    SparseLDLT ldlt_sp(precision_sp);
    SpMat Lsp = ldlt_sp.matrixL(); Matrix Lf_sp{Lsp};
    SpMat Usp = ldlt_sp.matrixU(); Matrix Uf_sp{Usp};
    Matrix Dsp = ldlt_sp.vectorD().real().asDiagonal();
    
    // reconstruction sparse
    Matrix res_sp = Lsp*Dsp*Usp;

    ASSERT_TRUE(eigen_wrapper.matrix_equal(L_true, Lf_sp));
    ASSERT_TRUE(eigen_wrapper.matrix_equal(D_true, Dsp));
    ASSERT_TRUE(eigen_wrapper.matrix_equal(L_true.transpose(), Usp));
    ASSERT_TRUE(eigen_wrapper.matrix_equal(precision, res_sp));
    
}

TEST(TestSparse, manipulation_sparse){
    typedef Eigen::SimplicialLDLT<SpMat, Eigen::Lower, Eigen::NaturalOrdering<int>> SparseLDLT;
    SparseLDLT ldlt_sp(precision_sp);
    SpMat Lsp = ldlt_sp.matrixL();

    Vector I, J, V;

    Eigen::SparseMatrix<double, Eigen::RowMajor> X(10, 10);

    // Lsp is in column fist order.
    eigen_wrapper.find_nnz(Lsp, I, J, V);
    ASSERT_EQ(Lsp.coeff(I(1), J(1)), Lsp.coeff(1, 0));

    // assemble the matrix from nnz elements.
    int size = Lsp.rows();
    SpMat assemblied(size, size);
    eigen_wrapper.assemble(assemblied, I, J, V);
    ASSERT_TRUE(eigen_wrapper.matrix_equal(assemblied, Lsp));

}

/**
 * @brief Test of following functions:
 * block extraction (sparse): EigenWrapper::block_extract_sparse()
 * block extraction for vector: EigenWrapper::block_extract()
 * block insertion (sparse): EigenWrapper::block_insert_sparse()
 * block insertion for vector: EigenWrapper::block_insert()
 */
TEST(TestSparse, sparse_permute){
    Matrix block_true(4, 4);
    block_true << 7.268329260163210,    2.961292370892880,         -4.500997556437790e-16,          0, 
                  2.961292370892880,    5.598315242672160,          2.316465704151060e-16,         0,
                 -4.500997556437790e-16, 2.316465704151060e-16,     6.250000000000390,              0,
                 0,                     0,                          0,                              6.250000000000380;
    SpMat block_extracted = eigen_wrapper.block_extract_sparse(precision_sp, 0, 3, 0, 3);
    ASSERT_TRUE(eigen_wrapper.matrix_equal(block_extracted, block_true));

    // zero block
    SpMat zero_block(4, 4);
    zero_block.setZero();
    eigen_wrapper.block_insert_sparse(precision_sp, 0, 3, 0, 3, zero_block);
    SpMat new_block = eigen_wrapper.block_extract_sparse(precision_sp, 0, 3, 0, 3);
    ASSERT_EQ(new_block.norm(), 0);

    // insert back the original block
    eigen_wrapper.block_insert_sparse(precision_sp, 0, 3, 0, 3, block_extracted);
    block_extracted = eigen_wrapper.block_extract_sparse(precision_sp, 0, 3, 0, 3);
    ASSERT_TRUE(eigen_wrapper.matrix_equal(block_extracted, block_true));

    // block for vector
    Vector vec = eigen_wrapper.random_matrix(10, 1);
    Vector v_blk = eigen_wrapper.random_matrix(4, 1);

    eigen_wrapper.block_insert(vec, 0, 3, 0, 0, v_blk);
    Vector v_block_extract = eigen_wrapper.block_extract(vec, 0, 4, 0, 0);
    ASSERT_EQ((v_block_extract - v_blk).norm(), 0);

}

/**
 * @brief Test for the following functions:
 * 
 * 
 */
TEST(TestSparse, sparse_inverse){
    SparseLDLT ldlt_sp(precision_sp);
    int size = precision_sp.rows();
    SpMat Lsp = ldlt_sp.matrixL();
    SpMat Usp = ldlt_sp.matrixU();
    Matrix Dsp = ldlt_sp.vectorD().real().asDiagonal();
    Matrix Dsp_inv = ldlt_sp.vectorD().real().cwiseInverse().asDiagonal();

    ASSERT_TRUE(eigen_wrapper.matrix_equal(Lsp, L_true));
    ASSERT_TRUE(eigen_wrapper.matrix_equal(Dsp, D_true));

    Matrix eye = Dsp*Dsp_inv;
    ASSERT_TRUE(eigen_wrapper.matrix_equal(eye, Matrix::Identity(size, size)));
    
    // find nnz
    Vector I,J,V;
    eigen_wrapper.find_nnz(Lsp, I, J, V);
    SpMat precision_inv_sp(size, size);

    // compare time
    std::cout << "sparse inverse time" << std::endl;
    timer.start();
    eigen_wrapper.inv_sparse(precision_sp, precision_inv_sp, I, J, V);
    timer.end();

    std::cout << "full inverse time" << std::endl;
    timer.start();
    Matrix inv_eigen = precision.inverse();
    timer.end();

    // assert inversion success
    Matrix inv_computed{precision_inv_sp};
    ASSERT_TRUE(eigen_wrapper.matrix_equal(inv_computed, cov));

    // inverse cov to match precision
    SpMat cov_inv_sp(size, size);
    eigen_wrapper.inv_sparse(cov_sp, cov_inv_sp, I, J, V);
    ASSERT_TRUE(eigen_wrapper.matrix_equal(cov_inv_sp, precision));

}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}