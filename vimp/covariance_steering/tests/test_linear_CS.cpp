/**
 * @file test_linear_CS.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the linear covariance steering code.
 * @version 0.1
 * @date 2023-03-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "covariance_steering/LinearCovarianceSteering.h"
#include <gtest/gtest.h>
#include "helpers/eigen_wrapper.h"
#include "helpers/data_io.h"

using namespace vimp;
using namespace Eigen;

TEST(LinearCS, initializations){
    // reading data
    MatrixIO m_io;
    EigenWrapper eigen_wrapper;

    MatrixXd At = m_io.load_csv("data/Aprior.csv");
    MatrixXd at = m_io.load_csv("data/aprior.csv");
    MatrixXd Qt = m_io.load_csv("data/Qk.csv");
    MatrixXd rt = m_io.load_csv("data/rk.csv");
    MatrixXd m0 = m_io.load_csv("data/m0.csv");
    MatrixXd m1 = m_io.load_csv("data/m1.csv");
    MatrixXd Sig0 = m_io.load_csv("data/Sig0.csv");
    MatrixXd Sig1 = m_io.load_csv("data/Sig1.csv");

    MatrixXd Mt_gt = m_io.load_csv("data/Mt.csv");
    MatrixXd Phi_gt = m_io.load_csv("data/Phi.csv");
    MatrixXd Phi11_gt = m_io.load_csv("data/Phi11.csv");
    MatrixXd Phi12_gt = m_io.load_csv("data/Phi12.csv");
    MatrixXd Pi_gt = m_io.load_csv("data/Pi.csv");

    MatrixXd K_gt = m_io.load_csv("data/K.csv");
    MatrixXd d_gt = m_io.load_csv("data/d.csv");

    int nx = 4;
    int nu = 2;
    int nt = 25;

    MatrixXd Bt = MatrixXd::Zero(nx, nu);
    Bt.block(2,0,2,2) = 5.0*MatrixXd::Identity(2,2);
    LinearCovarianceSteering linear_cs(At, Bt, at, 4, 2, nt, 0.01, Qt, rt, m0, Sig0, m1, Sig1);
    linear_cs.solve();

    MatrixXd M0{MatrixXd::Zero(8, 8)};
    MatrixXd Mt = linear_cs.Mt();
    eigen_wrapper.decompress3d(Mt, M0, 8, 8, 24);

    MatrixXd M0_gt{MatrixXd::Zero(8, 8)};
    eigen_wrapper.decompress3d(Mt_gt, M0_gt, 8, 8, 24);

    for (int i=0; i<25; i++){
        MatrixXd Mi{MatrixXd::Zero(2*nx, 2*nx)};
        MatrixXd Ai{MatrixXd::Zero(nx, nx)};
        MatrixXd Qi{MatrixXd::Zero(nx, nx)};
        MatrixXd Pii{MatrixXd::Zero(nx, nx)};

        eigen_wrapper.decompress3d(At, Ai, nx, nx, i);
        eigen_wrapper.decompress3d(Qt, Qi, nx, nx, i);
        eigen_wrapper.decompress3d(Mt_gt, Mi, 2*nx, 2*nx, i);
        eigen_wrapper.decompress3d(Pi_gt, Pii, nx, nx, i);

        ASSERT_LE((linear_cs.At(i)-Ai).norm(), 1e-10);
        ASSERT_LE((linear_cs.Qt(i)-Qi).norm(), 1e-10);
        ASSERT_LE((linear_cs.Mt(i)-Mi).norm(), 1e-10);
        
        ASSERT_LE((linear_cs.Pit(i)-Pii).norm(), 1e-10);
        
    }
    
    ASSERT_LE((linear_cs.Phi()-Phi_gt).norm(), 1e-10);
    ASSERT_LE((linear_cs.Phi11()-Phi11_gt).norm(), 1e-10);
    ASSERT_LE((linear_cs.Phi12()-Phi12_gt).norm(), 1e-10);
    
    ASSERT_LE((linear_cs.Kt()-K_gt).norm(), 1e-10);
    ASSERT_LE((linear_cs.dt()-d_gt).norm(), 1e-10);

}
