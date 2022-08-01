/**
 * @file test_GH.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief use known integrations to test the Gausse-Hermite approximated integrations.
 * @version 0.1
 * @date 2022-05-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../helpers/GaussHermite.h"
#include <functional>
#include <gtest/gtest.h>

using namespace Eigen;
using namespace vimp;

/// integrands used for testing
MatrixXd gx_1d(const VectorXd& x){
    return MatrixXd{MatrixXd::Constant(1, 1, x.norm()*x.norm())};
}

MatrixXd gx_2d(const VectorXd& x){
    MatrixXd res(2, 1);
    res << 3*x(0)*x(0), 2*x(0)*x(1);
    return MatrixXd{res};
}

MatrixXd gx_3d(const VectorXd& x){
    return MatrixXd{x*x.transpose().eval()};
}


/**
 * @brief Test the permute function helper with replacement.
 */
TEST(GaussHermite, permute){
    int dim = 1;
    VectorXd m = VectorXd::Ones(dim);
    MatrixXd P = MatrixXd::Identity(dim, dim);

    GaussHermite<std::function<MatrixXd(const VectorXd&)>> gausshermite(10, dim, m, P, gx_1d);

    int expected_0[dim] = {3,2,1,0};
    int expected_N[dim] = {0,1,2,3};
    
    int deg = 3;
    std::vector<int> vec = {0,1,2};
    std::vector<int> res = {0};
    std::vector<std::vector<int>> v_res;

    gausshermite.permute_replacing(vec, dim, res, 0, v_res);

    for (std::vector<int>& i_res: v_res){
        for (int& j:i_res){
            cout << "--j--" << endl << j << endl;
        }
    }
}


TEST(GaussHermite, one_dim){
    int dim = 1;
    VectorXd m = VectorXd::Ones(dim);
    MatrixXd P = MatrixXd::Identity(dim, dim);
    GaussHermite<std::function<MatrixXd(const VectorXd&)>> gausshermite(10, dim, m, P, gx_1d);

    MatrixXd integral1{gausshermite.Integrate()};
    MatrixXd integral_expected{MatrixXd::Constant(1, 1, 2.0)};

    ASSERT_LE((integral1 - integral_expected).norm(), 1e-10);

}

TEST(GaussHermite, two_dim){
    int dim = 2;
    VectorXd m = VectorXd::Ones(dim);
    MatrixXd prec(dim, dim);
    prec << 1.0,-0.74,-0.74,1.0;
    MatrixXd cov{prec.inverse()};

    GaussHermite<std::function<MatrixXd(const VectorXd&)>> gausshermite(10, dim, m, cov, gx_1d);
    MatrixXd integral1{gausshermite.Integrate()};

    MatrixXd integral_expected{MatrixXd::Constant(1, 1, 6.42087)};

    gausshermite.update_integrand(gx_2d);
    MatrixXd integral2{gausshermite.Integrate()};

    MatrixXd integral2_expected(2, 1);
    integral2_expected << 9.6313, 5.27144;

    ASSERT_LE((integral2 - integral2_expected).norm(), 1e-5);

}

TEST(GaussHermite, three_dim){
    int dim = 3;
    VectorXd m = VectorXd::Ones(dim);
    MatrixXd P = MatrixXd::Identity(dim, dim);

    GaussHermite<std::function<MatrixXd(const VectorXd&)>> gausshermite(10, dim, m, P, gx_1d);

    MatrixXd integral1{gausshermite.Integrate()};
    
    MatrixXd integral_expected{MatrixXd::Constant(1, 1, 6)};

    ASSERT_LE((integral1 - integral_expected).norm(), 1e-10);

}



