/**
 * @file GaussHermite.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Class to calculate the approximated integrations using Gauss-Hermite quadrature
 * @version 0.1
 * @date 2022-05-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <boost/math/special_functions/factorials.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
using namespace Eigen;
using namespace std;

namespace vimp{
    template <typename Function>
class GaussHermite{
public:
    /**
     * @brief Default constructor.
     */
    GaussHermite(){}

    /**
     * @brief Constructor
     * 
     * @param deg degree of GH polynomial
     * @param dim dimension of the integrand
     * @param mean mean 
     * @param P covariance matrix
     * @param func the integrand function
     */
    GaussHermite(
        const int& deg, 
        const int& dim, 
        const VectorXd& mean, 
        const MatrixXd& P, 
        const Function& func):
            _deg{deg},
            _dim{dim},
            _mean{mean},
            _P{P},
            _f{boost::make_shared<Function>(func)},
            _W{VectorXd::Zero(_deg)},
            _sigmapts{VectorXd::Zero(_deg)}{}


    /**
     * @brief Compute the Sigma Pts
     */
    void computeSigmaPts();

    /**
     * @brief Define the Hermite polynomial of degree deg, evaluate at x.
     * @param deg the degree to evaluate
     * @param x input
     * @return double function value
     */
    double HermitePolynomial(const int& deg, const double& x) const;

    /**
     * @brief Compute the weights in the Gauss-Hermite cubature method.
     * 
     * @return VectorXd Weights
     */
    void computeWeights();

    /**
     * @brief Compute the approximated integration using Gauss-Hermite.
     * 
     * @return MatrixXd 
     */
    MatrixXd Integrate();

    /**
     * Update member variables
     * */
    inline void update_mean(const VectorXd& mean){ assert(_mean.size()== mean.size()); _mean = mean; }

    inline void update_P(const MatrixXd& P){ assert(_P.size()==P.size()); _P = P; }

    inline void set_polynomial_deg(const int& p){ _deg = p; }

    inline void update_integrand(const Function& fun){ _f = boost::make_shared<Function>(fun); }

    inline void update_dimension(const int& dim){ _dim = dim; }

    inline VectorXd mean() const{ return _mean; }

    inline MatrixXd cov() const{ return _P; }

    inline MatrixXd f(const VectorXd& x){return (*_f)(x);}

private:
    int _deg;
    int _dim;
    VectorXd _mean;
    MatrixXd _P;
    boost::shared_ptr<Function> _f;
    VectorXd _W;
    VectorXd _sigmapts;
};

}

#include <vimp/helpers/GaussHermite-impl.h>