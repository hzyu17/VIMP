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

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <boost/math/special_functions/factorials.hpp>
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
     * @param p degree of GH polynomial
     * @param dim dimension of the integrand
     * @param mean mean 
     * @param P covariance matrix
     * @param func the integrand function
     */
    GaussHermite(const int& p, const int& dim, const VectorXd& mean, const MatrixXd& P, const Function& func):
        _p{p},
        _dim{dim},
        _mean{mean},
        _P{P},
        _f{func},
        _W{VectorXd::Zero(_p)},
        _sigmapts{VectorXd::Zero(_p)}{}


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
    inline void update_mean(const VectorXd& mean){ _mean = mean; }

    inline void update_P(const MatrixXd& P){ _P = P; }

    inline void set_polynomial_deg(const int& p){ _p = p; }

    inline void update_integrand(const Function& fun){ _f = fun; }

    inline void update_dimension(const int& dim){ _dim = dim; }

private:
    int _p;
    int _dim;
    VectorXd _mean;
    MatrixXd _P;
    Function _f;
    VectorXd _W;
    VectorXd _sigmapts;
};

}
