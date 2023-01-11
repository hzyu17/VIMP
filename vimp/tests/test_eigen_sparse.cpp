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

#include<gtest/gtest.h>

#include"../helpers/timer.h"
#include"../helpers/eigen_wrapper.h"
#include"../helpers/data_io.h"

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
Matrix precision = m_io.load_csv("precision_16.csv");
SpMat precision_sp = precision.sparseView();
Matrix cov = precision.inverse();
SpMat cov_sp = cov.sparseView();
Matrix D_true = m_io.load_csv("D_cpp.csv");
Matrix L_true = m_io.load_csv("L_cpp.csv");

// SpMat precision16_sp = precision.sparseView();

/**
 * @brief Test initialization of a sparse matrix.
 */
TEST(TestSparse, initialization){
    int m = 4; // number of rows and cols
    int n = m*m; // the matrix dimension

    // SparseLDLT ldlt_sp(precision_sp);
    // SpMat Lsp = ldlt_sp.matrixL();
    // Matrix D_cpp = ldlt_sp.vectorD().real().asDiagonal();
    // Matrix L_cpp{Lsp};
    // // Matrix D_cpp{Dsp};
    // m_io.saveData("L_cpp.csv", L_cpp);
    // m_io.saveData("D_cpp.csv", D_cpp);

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
    Matrix precision = m_io.load_csv("precision.csv");
    SpMat precision_sp = precision.sparseView();
    Matrix block_true(4, 4);
    block_true << 7.268329260163210,    2.961292370892880,         -4.500997556437790e-16,          0, 
                  2.961292370892880,    5.598315242672160,          2.316465704151060e-16,         0,
                 -4.500997556437790e-16, 2.316465704151060e-16,     6.250000000000390,              0,
                 0,                     0,                          0,                              6.250000000000380;
    SpMat block_extracted = eigen_wrapper.block_extract_sparse(precision_sp, 0, 0, 4, 4);
    ASSERT_TRUE(eigen_wrapper.matrix_equal(block_extracted, block_true));

    // zero block
    SpMat zero_block(4, 4);
    zero_block.setZero();
    eigen_wrapper.block_insert_sparse(precision_sp, 0, 0, 4, 4, zero_block);
    SpMat new_block = eigen_wrapper.block_extract_sparse(precision_sp, 0, 0, 4, 4);
    ASSERT_EQ(new_block.norm(), 0);

    // insert back the original block
    eigen_wrapper.block_insert_sparse(precision_sp, 0, 0, 4, 4, block_extracted);
    block_extracted = eigen_wrapper.block_extract_sparse(precision_sp, 0, 0, 4, 4);
    ASSERT_TRUE(eigen_wrapper.matrix_equal(block_extracted, block_true));

    // block for vector
    Vector vec = eigen_wrapper.random_matrix(10, 1);
    Vector v_blk = eigen_wrapper.random_matrix(4, 1);

    eigen_wrapper.block_insert(vec, 0, 0, 4, 1, v_blk);
    Vector v_block_extract = eigen_wrapper.block_extract(vec, 0, 0, 4, 1);
    ASSERT_TRUE(eigen_wrapper.matrix_equal(v_block_extract, v_blk));

}

TEST(TestSparse, masked_equality){
    SpMat spm = eigen_wrapper.random_sparse_matrix(40, 40, 50);
    Eigen::VectorXi I, J;
    Eigen::VectorXd K;
    eigen_wrapper.find_nnz(spm, I, J, K);

    SpMat disturbed_spm = eigen_wrapper.random_sparse_matrix(40, 40, 50) + spm;
    ASSERT_TRUE(eigen_wrapper.masked_equal(spm, spm, I, J));
    ASSERT_FALSE(eigen_wrapper.masked_equal(disturbed_spm, spm, I, J));
}

/**
 * @brief Test for the following functions:
 */
TEST(TestSparse, sparse_inverse){
    Matrix precision = m_io.load_csv("precision_16.csv");
    int size = precision.rows();

    SpMat precision_sp = precision.sparseView();
    SparseLDLT ldlt_sp(precision_sp);
    SpMat Lsp = ldlt_sp.matrixL();
    Matrix Dsp = ldlt_sp.vectorD().real().asDiagonal();

    // find nnz
    Eigen::VectorXi I,J;
    Eigen::VectorXd V;
    eigen_wrapper.find_nnz(Lsp, I, J, V);
    int nnz = I.rows();
    SpMat precision_inv_sp(size, size);
    SpMat precision_inv_trj(size, size);
    Matrix inv_full(size, size);

    // compare time
    std::cout << "sparse inverse time" << std::endl;
    timer.start();
    for (int i=0; i<100; i++){
        eigen_wrapper.inv_sparse(precision_sp, precision_inv_sp, I, J, nnz);
    }
    timer.end();

    std::cout << "sparse inverse trj time" << std::endl;
    timer.start();
    for (int i=0; i<100; i++){
        eigen_wrapper.inv_sparse_trj(precision_sp, precision_inv_trj, nnz, 4);
    }
    timer.end();

    std::cout << "full inverse time" << std::endl;
    timer.start();
    for (int i=0; i<100; i++){
        inv_full = precision.inverse();
    }
    timer.end();

    // assert inversion success
    Matrix inv_computed = precision_inv_sp;
    Matrix inv_computed_trj = precision_inv_trj;
    // m_io.saveData("inv_computed.csv", inv_computed);
    ASSERT_TRUE(eigen_wrapper.masked_equal(inv_computed, inv_full, I, J));
    ASSERT_TRUE(eigen_wrapper.masked_equal(inv_computed_trj, inv_full, I, J));
    
}

// TEST(TestSparse, sparse_view){
//     int size = 10;
//     int nnz = 20;
//     Eigen::MatrixXd m{eigen_wrapper.random_sparse_matrix(size, size, nnz)};
//     SpMat spm = m.sparseView();

//     Vector I,J,V;
//     eigen_wrapper.find_nnz(spm, I, J, V);
    
//     ASSERT_LE((spm - eigen_wrapper.sparse_view(m, I, J)).norm(), 1e-10);

//     SpMat spm_1 = spm + eigen_wrapper.random_sparse_matrix(size, size, nnz);

//     ASSERT_GE((spm_1 - eigen_wrapper.sparse_view(m, I, J)).norm(), 1e-10);
    
// }


// TEST(TestSparse, random_inverse){
//     int size = 10;
//     int nnz = 20;
//     Eigen::MatrixXd m{eigen_wrapper.random_sparse_matrix(size, size, nnz)};
//     Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(size, size);
//     m = m*m.transpose() + eye;

//     // Eigen::MatrixXd m(8, 8);
//     // m << 19718.95,  -0.00,      10.58,      0.00,       -126.96,    0.00,       10.58,     -0.00,  
//     //     -0.00,      19718.95,   0.00,       10.58,      -0.00,      -126.96,    0.00,       10.58,      
//     //     10.58,      0.00,       19593.17,   -0.00,      -10.58,     -0.00,      0.59,       0.00,
//     //     0.00,       10.58,      -0.00,      19593.17,   0.00,       -10.58,     -0.00,      0.59,
//     //     -126.96,    -0.00,      -10.58,     0.00,       253.93,     -0.00,      0.00,       0.00,
//     //     0.00,       -126.96,    -0.00,      -10.58,     -0.00,      253.94,     0.00,       0.00,
//     //     10.58,      0.00,       0.59,       -0.00,      0.00,       0.00,       2.37,       0.00,   
//     //     -0.00,      10.58,      0.00,       0.59,       0.00,       0.00,       0.00,       2.37;

//     ASSERT_TRUE(eigen_wrapper.is_psd(m));
//     SpMat spm = m.sparseView();

//     SparseLDLT ldlt_sp(spm);
//     SpMat L = ldlt_sp.matrixL();
//     SpMat Usp = ldlt_sp.matrixU();
//     Matrix Dsp = ldlt_sp.vectorD().real().asDiagonal();
//     Matrix Dsp_inv = ldlt_sp.vectorD().real().cwiseInverse();

//     // std::cout << "L " << std::endl;
//     // eigen_wrapper.print_matrix(L);

//     // std::cout << "D_inv " << std::endl;
//     // eigen_wrapper.print_matrix(Dsp_inv);
    
//     // reconstruction sparse
//     // Matrix recons_sp = L*Dsp*Usp;

//     // ASSERT_LE((recons_sp - spm).norm(), 1e-10);

//     Eigen::VectorXd I, J, V;
//     eigen_wrapper.find_nnz(L, I, J, V);
//     SpMat L_assemble(size, size);
//     eigen_wrapper.assemble(L_assemble, I, J, V);
//     ASSERT_LE((L_assemble - L).norm(), 1e-10);

//     SpMat spm_inv(size, size);
//     eigen_wrapper.inv_sparse(spm, spm_inv, I, J);
//     Eigen::MatrixXd spm_inv_full{spm_inv};

//     m_io.saveData("spm_inv_full.csv", spm_inv_full);

//     Eigen::MatrixXd inv_full = m.inverse();

//     std::cout << "X_inv" << std::endl;
//     eigen_wrapper.print_matrix(spm_inv_full);

//     SpMat inv_full_spview = eigen_wrapper.sparse_view(inv_full, I, J);
//     inv_full_spview = eigen_wrapper.sparse_view(inv_full_spview, I, I);

//     Eigen::MatrixXd diff = spm_inv_full - inv_full_spview;
//     std::cout << "difference " << std::endl;
//     eigen_wrapper.print_matrix(diff);

//     ASSERT_LE((spm_inv_full - inv_full_spview).norm(), 1e-10);
// }


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}