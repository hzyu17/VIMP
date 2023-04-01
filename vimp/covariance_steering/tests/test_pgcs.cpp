/**
 * @file test_pgcs.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the proximal gradient CS.
 * @version 0.1
 * @date 2023-03-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>
#include "dynamics/DoubleIntegratorDraged.h"
#include "covariance_steering/ProximalGradientCSNonlinearDyn.h"

using namespace Eigen;
using namespace vimp;

TEST(TestDynamics, linearization){
    MatrixIO m_io;
    EigenWrapper ei;
    int nx=4, nu=2, nt=25;
    MatrixXd At(nx*nx, nt), Bt(nx*nu, nt), St(nx*nx, nt), zt(nx, nt), hAt(nx*nx, nt), hat(nx, nt), nTr(nx, nt);
    MatrixXd Bt_gt(nx*nu, nt), hAt_gt(nx*nx, nt), hat_gt(nx, nt), nTr_gt(nx, nt);

    At = m_io.load_csv("data/Akt.csv");
    St = m_io.load_csv("data/Skt.csv");
    zt = m_io.load_csv("data/zkt.csv");

    Bt_gt = m_io.load_csv("data/Bt.csv");
    hAt_gt = m_io.load_csv("data/hAkt.csv");
    hat_gt = m_io.load_csv("data/hakt.csv");
    nTr_gt = m_io.load_csv("data/nTrt.csv");

    DoubleIntegrator dyn(nx, nu, nt);
    LinearDynamics lin_dyn(nx, nu, nt);

    std::tuple<LinearDynamics, Matrix3D> res;
    res = dyn.linearize(zt, 5.0, At, St);

    hAt = std::get<0>(res).At();
    Bt = std::get<0>(res).Bt();
    hat = std::get<0>(res).at();
    nTr = std::get<1>(res);

    ASSERT_LE((hAt-hAt_gt).norm(), 1e-10);
    ASSERT_LE((Bt - Bt_gt).norm(), 1e-10);
    ASSERT_LE((hat - hat_gt).norm(), 1e-10);
    ASSERT_LE((nTr - nTr_gt).norm(), 1e-10);

}

TEST(TestPGCS, solution){
    MatrixIO m_io;
    EigenWrapper ei;
    VectorXd m0(4), mT(4);
    MatrixXd Sig0(4,4), SigT(4,4);

    double sig = 5.0, eps=0.01, eta=1e-6;
    int nx=4, nu=2, nt=25;

    m0 << 1, 8, 2, 0;
    Sig0 = 0.01 * Eigen::MatrixXd::Identity(nx, nx);

    mT << 1, 2, -1, 0;
    SigT = 0.1 * Eigen::Matrix4d::Identity(nx, nx);

    MatrixXd A0(nx, nx), B(nx, nu), a0(nx, 1);
    std::shared_ptr<DoubleIntegrator> pdyn{new DoubleIntegrator(nx, nu, nt)};
    std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearized_0 = pdyn->linearize_timestamp(m0, sig, A0, Sig0);
    A0  = std::get<0>(linearized_0);
    B   = std::get<1>(linearized_0);
    a0  = std::get<2>(linearized_0);
    
    ProxGradCovSteerNLDyn pgcs(A0, a0, B, sig, nt, eta, eps, m0, Sig0, mT, SigT, pdyn);
    
    MatrixXd Q0(nx, nx);
    Q0 = 0.1*MatrixXd::Identity(nx, nx);
    pgcs.repliacteQt(Q0);
    MatrixXd Kt(nx*nu, nt), dt(nu, nt), Kt_gt(nx*nu, nt), dt_gt(nu, nt);
    std::tuple<MatrixXd, MatrixXd> res_Kd;

    double stop_err = 1e-4;
    res_Kd = pgcs.optimize(stop_err);

    MatrixXd Qkt(nx*nx, nt), rkt(nx, nt);
    MatrixXd Qkt_gt(nx*nx, nt), rkt_gt(nx, nt);
    Qkt_gt = m_io.load_csv("data/Qkt.csv");
    rkt_gt = m_io.load_csv("data/rkt.csv");
    Qkt = pgcs.Qkt();
    rkt = pgcs.rkt();
    
    ASSERT_LE((Qkt - Qkt_gt).norm(), 1e-8);
    ASSERT_LE((rkt - rkt_gt).norm(), 1e-8);

    Kt = std::get<0>(res_Kd);
    dt = std::get<1>(res_Kd);

    Kt_gt = m_io.load_csv("data/Kt.csv");
    dt_gt = m_io.load_csv("data/dt.csv");

    ASSERT_LE((Kt - Kt_gt).norm(), 1e-10);
    ASSERT_LE((dt - dt_gt).norm(), 1e-10);
}
