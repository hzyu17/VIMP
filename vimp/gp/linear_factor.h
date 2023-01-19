/**
 * @file linear_factor.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The linear Gaussian factor. -log(p(x|z)) = C*||Ax - B\mu_t||_{\Sigma_t^{-1}}.
 * @version 0.1
 * @date 2023-01-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <Eigen/Dense>
using namespace Eigen;

namespace vimp{
class LinearFactor{
public:
    LinearFactor(){};

    virtual VectorXd get_mean(){};
    virtual MatrixXd get_covariance(){};
    virtual MatrixXd get_precision(){};

    virtual MatrixXd get_A(){};
    virtual MatrixXd get_B(){};
    virtual double get_C(){}; // get the constant
};
}
