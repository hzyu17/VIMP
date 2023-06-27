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
    virtual ~LinearFactor(){} 
    LinearFactor(){}

    // virtual VectorXd get_mean() = 0;
    inline virtual VectorXd get_mu() = 0;
    inline virtual MatrixXd get_covariance() const = 0;
    inline virtual MatrixXd get_precision() const = 0;

    inline virtual MatrixXd get_Lambda() const = 0;
    inline virtual MatrixXd get_Psi() const = 0;
    inline virtual double get_C() const = 0; // get the constant
};
}
