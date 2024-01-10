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
#include "GaussianVI/helpers/EigenWrapper.h"
#include "GaussianVI/helpers/MatrixHelper.h"

#include<Eigen/IterativeLinearSolvers>


typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SpMat; // declares a col-major sparse matrix type of double
typedef Eigen::Triplet<double> T;
typedef Eigen::SimplicialLDLT<SpMat, Eigen::Lower, Eigen::NaturalOrdering<int>> SparseLDLT;

gvi::EigenWrapper eigen_wrapper;
Timer timer;
gvi::MatrixIO m_io;

// ================== read ground truth matrices ==================
Eigen::MatrixXd precision = m_io.load_csv("data/precision_16.csv");
SpMat precision_sp = precision.sparseView();
Eigen::MatrixXd cov = precision.inverse();
SpMat cov_sp = cov.sparseView();
Eigen::MatrixXd D_true = m_io.load_csv("data/D_cpp.csv");
Eigen::MatrixXd L_true = m_io.load_csv("data/L_cpp.csv");


/**
 * @brief test the time of solving equations
 */
TEST(TestSparse, solve_psd_eqn){
    int m=4;
    int n=m*m;

    gvi::SpMat eye(n, n);
    eye.setIdentity();

    gvi::SpMat spm = eigen_wrapper.randomd_sparse_psd(n, 10) + eye;
    
    Eigen::MatrixXd mat{spm};
    Eigen::LDLT<Eigen::MatrixXd> ldlt(mat);        

    ASSERT_TRUE(ldlt.isPositive());

    // RHS
    Eigen::VectorXd b(Eigen::VectorXd::Random(n));

    // solve full
    Eigen::MatrixXd fullm(spm);
    timer.start();
    Eigen::VectorXd x = fullm.llt().solve(b); 
    std::cout << "solving time full matrix " << std::endl;
    timer.end();

    // solve sparse
    timer.start();
    
    Eigen::ConjugateGradient<SpMat, Eigen::Upper> solver;
    Eigen::VectorXd spx =  solver.compute(spm).solve(b);

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
    auto ldlt_full = precision.ldlt();
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
    
    SparseLDLT ldlt_sp(precision_sp);
    SpMat Lsp = ldlt_sp.matrixL();

    Eigen::VectorXd I, J, V;

    Eigen::SparseMatrix<double, Eigen::RowMajor> X(10, 10);

    // Lsp is in column fist order.
    eigen_wrapper.find_nnz(Lsp, I, J, V);
    ASSERT_EQ(Lsp.coeff(I(1), J(1)), Lsp.coeff(1, 0));

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
    Eigen::VectorXd vec = Eigen::MatrixXd::Random(10 ,1);
    Eigen::VectorXd v_blk = Eigen::MatrixXd::Random(4 ,1);

    eigen_wrapper.block_insert(vec, 0, 0, 4, 1, v_blk);
    Eigen::VectorXd v_block_extract = eigen_wrapper.block_extract(vec, 0, 0, 4, 1);
    ASSERT_TRUE(eigen_wrapper.matrix_equal(v_block_extract, v_blk));

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
    
    std::cout << "sparse inverse trj time" << std::endl;
    SpMat precision_inv_trj(size, size);

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


TEST(TestMatrix3D, compress3d){
    Eigen::MatrixXd mat(3, 3);
    mat << 1,2,3,4,5,6,7,8,9;
    
    gvi::Matrix3D mat3d(3,3,2);
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

MatrixXd linspace(const Eigen::VectorXd & x0, const Eigen::VectorXd & xT, int nt){
        int rows = x0.rows();
        Eigen::VectorXd step_vec(rows);
        step_vec.setZero();
        step_vec = (xT-x0)/(nt-1);
        Eigen::MatrixXd res(rows, nt);
        res.setZero();
        for (int i=0; i<nt; i++){
            res.col(i) = x0 + step_vec*i;
        }
        return res;
    }

TEST(TestMatrix3D, test_linspace){
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
                    
    linspaced = linspace(x0, xT, nt);
    ASSERT_LE((linspaced_gt - linspaced).norm(), 1e-10);

}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}