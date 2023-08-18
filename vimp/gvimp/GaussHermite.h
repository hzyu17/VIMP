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

#include "helpers/EigenWrapper.h"
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
     * @param dim dimension of the integrand input
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
            _f{func},
            _W{VectorXd::Zero(_deg)},
            _sigmapts{VectorXd::Zero(_deg)}{}

    /**
     * @brief A helper function to compute all possible permutations given a dimension and a degree.
     *  for computing the integration using sigmapoints and weights. Returns all vector of length dimension,
     *  collected from the number degree. It is a permutation with replacement.
     * @param dimension 
     * @return std::vector<double>
     */
    void permute_replacing(const std::vector<int>& vec, 
                            const int& dimension, 
                            std::vector<int>& res, 
                            int index, 
                            std::vector<std::vector<int>>& v_res);

    /**
     * @brief Compute the Sigma Pts
     */
    void computeSigmaPts();

    /**
     * @brief Define the Hermite polynomial of degree deg, evaluate at x.
     * @param deg the degree to evaluate
     * @param x input
     */
    double HermitePolynomial(const int& deg, const double& x) const;

    /**
     * @brief Compute the weights in the Gauss-Hermite cubature method.
     */
    void computeWeights();

    /**
     * @brief Compute the approximated integration using Gauss-Hermite.
     */
    MatrixXd Integrate(const Function& function);

    void update_integrand(const Function& function);

    MatrixXd Integrate();

    /**
     * Update member variables
     * */
    inline void update_mean(const VectorXd& mean){ _mean = mean; }

    inline void update_P(const MatrixXd& P){ _P = P; }

    inline void set_polynomial_deg(const int& deg){ _deg = deg; }

    inline void update_dimension(const int& dim){ _dim = dim; }

    inline VectorXd mean() const{ return _mean; }

    inline MatrixXd cov() const{ return _P; }

    inline VectorXd weights() { computeWeights(); return _W;}

    inline VectorXd sigmapts() { computeSigmaPts(); return _sigmapts;}

protected:
    int _deg;
    int _dim;
    VectorXd _mean;
    MatrixXd _P;
    VectorXd _W;
    VectorXd _sigmapts;
    EigenWrapper _ei;
    Function _f;
};

}

#include "GaussHermite-impl.h"