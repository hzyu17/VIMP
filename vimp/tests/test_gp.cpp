/**
 * @file test_gp.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test Gaussian prior classes
 * @version 0.1
 * @date 2022-07-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <gtest/gtest.h>
#include "../instances/PlanarPRFactor.h"
#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gtsam/inference/Symbol.h>
#include <gpmp2/gp/GPutils.h>


using namespace vimp;

TEST(TESTFIXEDGP, cost){
    int dim = 2;

    VectorXd mu{VectorXd::Ones(dim)};
    MatrixXd Qc{MatrixXd::Identity(dim, dim)};
    
    FixedPriorGP gp{Qc, mu};

    double thres = 1e-7;

    VectorXd x(dim);
    x << 1.2, 2.1;

    double cost_expected = 1.25;

    ASSERT_LE(abs(gp.cost(x) - cost_expected), thres);
}


TEST(TESTGPAccGP, creation){
    int dim = 2;
    double dt = 0.01;
    MatrixXd Qc{MatrixXd::Identity(dim, dim)};
    MinimumAccGP gp{Qc, dt};

    double thres = 1e-7;
    
    /// Qc
    ASSERT_LE((Qc - gp.Qc()).norm(), thres);
}


TEST(TESTGPAccGP, computeQPhi){
    int dim = 2;
    double dt = 0.01;
    MatrixXd Qc{MatrixXd::Identity(dim, dim)};
    MinimumAccGP gp{Qc, dt};
    
    double thres = 1e-7;

    /// calculation of Q
    MatrixXd Q_expected{(MatrixXd(2*dim, 2*dim) << 
                  3.333333333333334e-07, 0, 5e-05, 0, 
                  0, 3.333333333333334e-07, 0, 5e-05,
                  5e-05, 0, 0.01, 0,
                  0, 5e-05, 0, 0.01).finished()};

    MatrixXd Phi_expected{(MatrixXd(2*dim, 2*dim) <<
                    1, 0, 0.01, 0,
                    0, 1, 0, 0.01,
                    0, 0, 1, 0,
                    0, 0, 0, 1).finished()};

    ASSERT_LE((Q_expected - gp.Q()).norm(), thres);
    ASSERT_LE((Phi_expected - gp.Phi()).norm(), thres);

}


TEST(TESTGPAccGP, invQ){
    int dim = 2;
    double dt = 0.01;
    MatrixXd Qc{MatrixXd::Identity(dim, dim)};
    MinimumAccGP gp{Qc, dt};

    double thres = 1e-7;

    /// calculation of Q
    MatrixXd Q_expected{(MatrixXd(2*dim, 2*dim) << 
                  3.333333333333334e-07, 0, 5e-05, 0, 
                  0, 3.333333333333334e-07, 0, 5e-05,
                  5e-05, 0, 0.01, 0,
                  0, 5e-05, 0, 0.01).finished()};

    /// invQ
    MatrixXd invQ_expected = Q_expected.inverse();
    ASSERT_LE((gp.invQ() - invQ_expected).norm(), thres);

}


TEST(TESTGPAccGP, cost){
    int dim = 2;
    double dt = 0.01;
    MatrixXd Qc{MatrixXd::Identity(dim, dim)};
    MinimumAccGP gp{Qc, dt};

    double thres = 1e-7;

    VectorXd x1{(VectorXd(2*dim)<<1.5, 1.0, 0.1, 0.1).finished()};
    VectorXd x2{(VectorXd(2*dim)<<1.1, 1.2, 0.3, 0.2).finished()};
    
    /// cost function, -logprob
    double cost = gp.cost(x1, x2);
    double cost_expected = 1.206039999999998e+06;
    ASSERT_LE(abs(cost-cost_expected), thres);

}


TEST(TESTGPAccGP, cost_combined_states){
    int dim = 2;
    double dt = 0.01;
    MatrixXd Qc{MatrixXd::Identity(dim, dim)};
    MinimumAccGP gp{Qc, dt};

    double thres = 1e-7;

    VectorXd x1{(VectorXd(2*dim)<<1.5, 1.0, 0.1, 0.1).finished()};
    VectorXd x2{(VectorXd(2*dim)<<1.1, 1.2, 0.3, 0.2).finished()};

    /// combined states
    VectorXd state_cmb(4*dim);
    state_cmb.segment(0, 2*dim) = x1;
    state_cmb.segment(2*dim, 2*dim) = x2;

    /// cost function, -logprob
    double cost = cost_linear_gp(state_cmb, gp);
    double cost_expected = 1.206039999999998e+06;
    ASSERT_LE(abs(cost-cost_expected), thres);

}

using namespace gpmp2;
using namespace gtsam;

TEST(TESTGPAccGP, gpmp2_compare){
    /// --------------------------------- vimp ---------------------------------
    int dim = 2;
    double dt = 0.01;
    MatrixXd Qc{MatrixXd::Identity(dim, dim)};
    MinimumAccGP gp{Qc, dt};

    double thres = 1e-7;

    VectorXd x1{(VectorXd(2*dim)<<1.5, 1.0, 0.1, 0.1).finished()};
    VectorXd x2{(VectorXd(2*dim)<<1.1, 1.2, 0.3, 0.2).finished()};

    /// combined states
    VectorXd state_cmb(4*dim);
    state_cmb.segment(0, 2*dim) = x1;
    state_cmb.segment(2*dim, 2*dim) = x2;

    /// cost function, -logprob
    double cost = cost_linear_gp(state_cmb, gp);

    /// ------------------------------ gpmp2 -----------------------------------
    SharedNoiseModel Qc_gpmp2 = noiseModel::Isotropic::Sigma(dim, 1.0);

    GaussianProcessPriorLinear prior_gpmp2{gtsam::symbol('x', 0), 
                                            gtsam::symbol('v', 1), 
                                            gtsam::symbol('x', 0), 
                                            gtsam::symbol('v', 1), dt, Qc_gpmp2};

    MatrixXd invQ_gpmp2 = calcQ_inv(Qc, dt);
    MatrixXd Phi_gpmp2 = calcPhi(dim, dt);

    /// ------------------------------ compare -----------------------------------
    ASSERT_LE((gp.invQ() - invQ_gpmp2).norm(), thres);
    ASSERT_LE((gp.Phi() - Phi_gpmp2).norm(), thres);

}