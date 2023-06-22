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

#include "helpers/timer.h"
#include "helpers/eigen_wrapper.h"
#include "helpers/MatrixIO.h"

#include<Eigen/IterativeLinearSolvers>


typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SpMat; // declares a col-major sparse matrix type of double
typedef Eigen::Triplet<double> T;

using namespace vimp;
EigenWrapper eigen_wrapper;
Timer timer;
MatrixIO m_io;

// ================== read ground truth matrices ==================
Eigen::MatrixXd precision = m_io.load_csv("data/precision_16.csv");
SpMat precision_sp = precision.sparseView();
Eigen::MatrixXd cov = precision.inverse();
SpMat cov_sp = cov.sparseView();
Eigen::MatrixXd D_true = m_io.load_csv("data/D_cpp.csv");
Eigen::MatrixXd L_true = m_io.load_csv("data/L_cpp.csv");

// SpMat precision16_sp = precision.sparseView();

/**
 * @brief Test initialization of a sparse matrix.
 */
TEST(TestSparse, initialization){
    int m = 4; // number of rows and cols
    int n = m*m; // the matrix dimension

    // SparseLDLT ldlt_sp(precision_sp);
    // SpMat Lsp = ldlt_sp.matrixL();
    // Eigen::MatrixXd D_cpp = ldlt_sp.vectorD().real().asDiagonal();
    // Eigen::MatrixXd L_cpp{Lsp};
    // // Eigen::MatrixXd D_cpp{Dsp};
    // m_io.saveData("L_cpp.csv", L_cpp);
    // m_io.saveData("D_cpp.csv", D_cpp);

    timer.start();
    Eigen::MatrixXd rand_mat = eigen_wrapper.random_matrix(n, n);
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

    Eigen::MatrixXd m1(spm1);
    Eigen::MatrixXd m2(spm2);
    Eigen::MatrixXd m3(n, n);

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
    Eigen::VectorXd b(eigen_wrapper.random_vector(n));

    // solve full
    Eigen::MatrixXd fullm(spm);
    timer.start();
    Eigen::VectorXd x = eigen_wrapper.solve_llt(fullm, b);
    std::cout << "solving time full matrix " << std::endl;
    timer.end();

    // solve sparse
    timer.start();
    Eigen::VectorXd spx = eigen_wrapper.solve_cgd_sp(spm, b);
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
    Eigen::MatrixXd eye(size,size);
    eye.setIdentity();

    ASSERT_LE((precision - precision_sp).norm(), 1e-10);

    // ================== full ldlt decomposition ==================
    auto ldlt_full = eigen_wrapper.ldlt_full(precision);
    Eigen::MatrixXd Lf = ldlt_full.matrixL();
    Eigen::MatrixXd Uf = ldlt_full.matrixU();
    Eigen::MatrixXd Df = ldlt_full.vectorD().real().asDiagonal();
    auto Pf = ldlt_full.transpositionsP();
    ASSERT_LE((precision - ldlt_full.reconstructedMatrix()).norm(), 1e-10);

    // reconstruction
    Eigen::MatrixXd res(size,size);
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
    SpMat Lsp = ldlt_sp.matrixL(); Eigen::MatrixXd Lf_sp{Lsp};
    SpMat Usp = ldlt_sp.matrixU(); Eigen::MatrixXd Uf_sp{Usp};
    Eigen::MatrixXd Dsp = ldlt_sp.vectorD().real().asDiagonal();
    
    // reconstruction sparse
    Eigen::MatrixXd res_sp = Lsp*Dsp*Usp;

    ASSERT_TRUE(eigen_wrapper.matrix_equal(L_true, Lf_sp));
    ASSERT_TRUE(eigen_wrapper.matrix_equal(D_true, Dsp));
    ASSERT_TRUE(eigen_wrapper.matrix_equal(L_true.transpose(), Usp));
    ASSERT_TRUE(eigen_wrapper.matrix_equal(precision, res_sp));
    
}

TEST(TestSparse, manipulation_sparse){
    typedef Eigen::SimplicialLDLT<SpMat, Eigen::Lower, Eigen::NaturalOrdering<int>> SparseLDLT;
    SparseLDLT ldlt_sp(precision_sp);
    SpMat Lsp = ldlt_sp.matrixL();

    Eigen::VectorXd I, J, V;

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
    Eigen::MatrixXd precision = m_io.load_csv("data/precision.csv");
    SpMat precision_sp = precision.sparseView();
    Eigen::MatrixXd block_true(4, 4);
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
    Eigen::VectorXd vec = eigen_wrapper.random_matrix(10, 1);
    Eigen::VectorXd v_blk = eigen_wrapper.random_matrix(4, 1);

    eigen_wrapper.block_insert(vec, 0, 0, 4, 1, v_blk);
    Eigen::VectorXd v_block_extract = eigen_wrapper.block_extract(vec, 0, 0, 4, 1);
    ASSERT_TRUE(eigen_wrapper.matrix_equal(v_block_extract, v_blk));

}

// TEST(TestSparse, masked_equality){
//     SpMat spm = eigen_wrapper.random_sparse_matrix(40, 40, 50);
//     Eigen::VectorXi I, J;
//     Eigen::VectorXd K;
//     eigen_wrapper.find_nnz(spm, I, J, K);

//     SpMat disturbed_spm = spm;
//     disturbed_spm.coeffRef(I(0), J(0)) = 0;
//     ASSERT_TRUE(eigen_wrapper.masked_equal(spm, spm, I, J));
//     ASSERT_FALSE(eigen_wrapper.masked_equal(disturbed_spm, spm, I, J));
// }

TEST(TestSparse, compare_block_operations){
    Eigen::MatrixXd precision = m_io.load_csv("data/precision_large.csv");
    SpMat precision_sp = precision.sparseView();
    int ndim = precision.rows();
    int dim_state = 4;

    // Pk
    Eigen::MatrixXd Pk{Eigen::MatrixXd::Zero(2*dim_state, ndim)};
    Pk.block(0, 3*dim_state, 2*dim_state, 2*dim_state) = std::move(Eigen::MatrixXd::Identity(2*dim_state, 2*dim_state));

    // results
    Eigen::MatrixXd Block_Pk(2*dim_state, 2*dim_state);
    Eigen::MatrixXd Block(2*dim_state, 2*dim_state);

    // compare the time between Pk*M*Pk^T and M.block(i,j,h,l);
    std::cout << "time elapsed: block by eigen" << std::endl;
    timer.start();
    Block = precision.block(3*dim_state-1, 3*dim_state-1, 2*dim_state, 2*dim_state);
    timer.end();

    std::cout << "time elapsed: block by Pk" << std::endl;
    timer.start();
    Block_Pk = Pk * precision * Pk.transpose();
    timer.end();

    ASSERT_TRUE(eigen_wrapper.matrix_equal(Block, Block_Pk));

    // compare the time between the two for block insertion
    Eigen::MatrixXd precision_inserted_Pk(ndim, ndim);
    precision_inserted_Pk.setZero();

    SpMat precision_inserted_block(ndim, ndim);
    precision_inserted_block.setZero();

    std::cout << "insertion time for block operations" << std::endl;
    timer.start();
    eigen_wrapper.block_insert_sparse(precision_inserted_block, 3*dim_state-1, 3*dim_state-1, 2*dim_state, 2*dim_state, Block);
    timer.end();

    std::cout << "insertion time for Pk" << std::endl;
    timer.start();
    precision_inserted_Pk = Pk.transpose() * Block_Pk * Pk;
    timer.end();

    ASSERT_TRUE(eigen_wrapper.matrix_equal(precision_inserted_block, precision_inserted_Pk));
}

/**
 * @brief Test for the sparse inverse function
 */
TEST(TestSparse, sparse_inverse){
    Eigen::MatrixXd precision = m_io.load_csv("data/precision_large.csv");
    int size = precision.rows();

    SpMat precision_sp = precision.sparseView();
    SparseLDLT ldlt_sp(precision_sp);
    SpMat Lsp = ldlt_sp.matrixL();
    Eigen::MatrixXd Dsp = ldlt_sp.vectorD().real().asDiagonal();

    // find nnz
    Eigen::VectorXi I,J;
    Eigen::VectorXd V;
    eigen_wrapper.find_nnz(Lsp, I, J, V);
    int nnz = I.rows();
    Eigen::MatrixXd inv_full(size, size);

    SpMat precision_inv_1_sp(size, size);
    Eigen::VectorXi StartIndx(nnz);
    eigen_wrapper.find_nnz_known_ij(Lsp, I, J, V);

    // compare time
    std::cout << "sparse inverse time: " << std::endl;
    SpMat precision_inv_sp(size, size);
    timer.start();
    for (int i=0; i<100; i++){
        eigen_wrapper.inv_sparse(precision_sp, precision_inv_sp, I, J, nnz);
    }
    timer.end();

    std::cout << "sparse inverse with outside ldlt, time: " << std::endl;
    SpMat precision_inv_sp1(size, size);
    timer.start();
    for (int i=0; i<100; i++){
        SparseLDLT ldlt_sp1(precision_sp);
        SpMat Lsp = ldlt_sp1.matrixL();
        Eigen::VectorXd Dinv = ldlt_sp.vectorD().real().cwiseInverse();
        eigen_wrapper.inv_sparse(precision_sp, precision_inv_sp1, I, J, V, Dinv);
    }
    timer.end();

    std::cout << "sparse inverse_1 time: " << std::endl;
    
    eigen_wrapper.construct_iteration_order(precision_sp, I, J, StartIndx, nnz);
    timer.start();
    for (int i=0; i<100; i++){
        eigen_wrapper.inv_sparse_1(precision_sp, precision_inv_1_sp, I, J, V, StartIndx, nnz);
    }
    timer.end();

    std::cout << "sparse inverse trj time" << std::endl;
    SpMat precision_inv_trj(size, size);
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
    Eigen::MatrixXd inv_computed = precision_inv_sp;
    Eigen::MatrixXd inv_computed_2 = precision_inv_sp1;
    Eigen::MatrixXd inv_computed_1 = precision_inv_1_sp;
    Eigen::MatrixXd inv_computed_trj = precision_inv_trj;
    // m_io.saveData("inv_computed.csv", inv_computed);
    // m_io.saveData("inv_computed_trj.csv", inv_computed_trj);
    ASSERT_TRUE(eigen_wrapper.matrix_equal(inv_computed, inv_computed_1));
    ASSERT_TRUE(eigen_wrapper.matrix_equal(inv_computed, inv_computed_trj));
    ASSERT_TRUE(eigen_wrapper.matrix_equal(inv_computed, inv_computed_2));
    ASSERT_TRUE(eigen_wrapper.masked_equal(inv_computed_trj, inv_full, I, J));
    ASSERT_TRUE(eigen_wrapper.masked_equal(inv_computed, inv_full, I, J));
    ASSERT_TRUE(eigen_wrapper.masked_equal(inv_computed_1, inv_full, I, J));
    ASSERT_TRUE(eigen_wrapper.masked_equal(inv_computed_trj, inv_full, I, J));
    
}

TEST(TestSparse, sparse_view){
    int size = 10;
    int nnz = 20;
    Eigen::MatrixXd m{eigen_wrapper.random_sparse_matrix(size, size, nnz)};
    SpMat spm = m.sparseView();

    Eigen::VectorXd I,J,V;
    eigen_wrapper.find_nnz(spm, I, J, V);
    
    ASSERT_LE((spm - eigen_wrapper.sparse_view(m, I, J)).norm(), 1e-10);

    SpMat spm_1 = spm + eigen_wrapper.random_sparse_matrix(size, size, nnz);

    ASSERT_GE((spm_1 - eigen_wrapper.sparse_view(m, I, J)).norm(), 1e-10);
    
}

TEST(TestSparse, determinant){
    Eigen::MatrixXd precision = m_io.load_csv("data/precision_16.csv");
    SpMat precision_sp = precision.sparseView();

    SparseLDLT ldlt_sp(precision_sp);

    SpMat Lsp = ldlt_sp.matrixL(); Eigen::MatrixXd Lf_sp{Lsp};
    SpMat Usp = ldlt_sp.matrixU(); Eigen::MatrixXd Uf_sp{Usp};
    Eigen::MatrixXd Dsp = ldlt_sp.vectorD().real().asDiagonal();

    Eigen::MatrixXd LDLT = Lf_sp*Dsp*Uf_sp;

    ASSERT_TRUE(eigen_wrapper.matrix_equal(LDLT, precision));

    timer.start();
    double logdet_sp = log(ldlt_sp.determinant());
    timer.end_mus();

    timer.start();
    double logdet_origin = log(precision.determinant());
    timer.end_mus();

    timer.start();
    double logdet_D = log(Dsp.determinant());
    timer.end_mus();

    ASSERT_LE(abs(logdet_D - logdet_origin), 1e-10);
    ASSERT_LE(abs(logdet_sp - logdet_origin), 1e-10);
}


TEST(TestSparse, sqrtm){
    Eigen::MatrixXd m = eigen_wrapper.random_psd(5);
    Eigen::MatrixXd m_inv = m.inverse();

    Eigen::MatrixXd sqrtm = eigen_wrapper.psd_sqrtm(m);
    Eigen::MatrixXd inv_sqrtm = eigen_wrapper.psd_invsqrtm(m);

    ASSERT_LE((sqrtm*sqrtm - m).norm(), 1e-10);
    ASSERT_LE((inv_sqrtm*inv_sqrtm - m_inv).norm(), 1e-10);
}

TEST(TestMatrix3D, compress3d){
    Eigen::MatrixXd mat(3, 3);
    mat << 1,2,3,4,5,6,7,8,9;
    
    Matrix3D mat3d(3,3,2);
    mat3d.setZero();
    eigen_wrapper.compress3d(mat, mat3d, 0);

    Eigen::MatrixXd mat3d_groundtruth(9, 2);
    mat3d_groundtruth << 1,0,4,0,7,0,2,0,5,0,8,0,3,0,6,0,9,0;

    ASSERT_LE((mat3d - mat3d_groundtruth).norm(), 1e-10);

    Eigen::MatrixXd mat_decomposed{Eigen::MatrixXd::Zero(3,3)};
    eigen_wrapper.decomp3d(mat3d, mat_decomposed, 3, 3, 0);

    ASSERT_LE((mat_decomposed - mat).norm(), 1e-10);

}

TEST(TestMatrix3D, repmat){
    Eigen::MatrixXd mat(3,3);
    mat <<  0.7922, 0.0357, 0.6787,
            0.9595, 0.8491, 0.7577,
            0.6557, 0.9340, 0.7431;

    Eigen::MatrixXd res(9, 3);

    res = eigen_wrapper.replicate3d(mat, 3);

    Eigen::MatrixXd res_ground_truth(9, 3);
    res_ground_truth << 0.7922, 0.7922, 0.7922,
                        0.9595, 0.9595, 0.9595,
                        0.6557, 0.6557, 0.6557,
                        0.0357, 0.0357, 0.0357,
                        0.8491, 0.8491, 0.8491,
                        0.9340, 0.9340, 0.9340,
                        0.6787, 0.6787, 0.6787,
                        0.7577, 0.7577, 0.7577,
                        0.7431, 0.7431, 0.7431;
    
    ASSERT_LE((res - res_ground_truth).norm(), 1e-10);
    Eigen::MatrixXd randi(3, 3);
    for (int i=0; i<3; i++){
        randi = eigen_wrapper.decomp3d(res, mat.rows(), mat.cols(), i);
        ASSERT_LE((randi - mat).norm(), 1e-10);
    }
}

TEST(TestMatrix3D, linspace){
    int nt = 5;
    Eigen::VectorXd x0(4), xT(4);
    MatrixXd linspaced_gt(4, nt), linspaced(4, nt);
    x0.setZero();xT.setZero();linspaced.setZero();linspaced_gt.setZero();
    x0 << 0, 1, 2, 3;
    xT << 4.8, 5.8, 6.8, 7.8;
    linspaced_gt << 0, 1.2, 2.4, 3.6, 4.8,
                    1, 2.2, 3.4, 4.6, 5.8,
                    2, 3.2, 4.4, 5.6, 6.8,
                    3, 4.2, 5.4, 6.6, 7.8;
                    
    linspaced = eigen_wrapper.linspace(x0, xT, nt);
    ASSERT_LE((linspaced_gt - linspaced).norm(), 1e-10);

}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}