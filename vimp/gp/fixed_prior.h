/**
 * @file fixed_prior.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Fixed Gaussian prior
 * @version 0.1
 * @date 2022-07-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include<Eigen/Dense>

using namespace Eigen;

namespace vimp{
    class FixedPriorGP{
        public:
            FixedPriorGP(){}
            FixedPriorGP(const MatrixXd& K, const VectorXd& mu):
            _mu{mu}, 
            _dim{mu.size()}, 
            _K{K}, 
            _invK{K.inverse()}{}

        private:
            MatrixXd _K;
            MatrixXd _invK;
            int _dim;
            VectorXd _mu;

        public:
            double cost(const VectorXd& x) const{
                return (x-_mu).transpose().eval() * _invK * (x-_mu);
            }

    };
}// namespace vimp
