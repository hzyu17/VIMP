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
#include "../robots/DoubleIntegrator.h"

using namespace Eigen;
using namespace vimp;

TEST(TestPGCS, linearization){
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

    std::tuple<MatrixXd, MatrixXd, MatrixXd, MatrixXd> res;
    res = dyn.linearize(zt, 5.0, At, St);

    hAt = std::get<0>(res);
    Bt = std::get<1>(res);
    hat = std::get<2>(res);
    nTr = std::get<3>(res);
    
    ASSERT_LE((hAt-hAt_gt).norm(), 1e-10);
    ASSERT_LE((Bt - Bt_gt).norm(), 1e-10);
    ASSERT_LE((hat - hat_gt).norm(), 1e-10);
    ASSERT_LE((nTr - nTr_gt).norm(), 1e-10);

}