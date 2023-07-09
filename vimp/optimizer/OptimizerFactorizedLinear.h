/**
 * @file OptimizerFactorizedLinearGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Factorized optimization steps for linear gaussian factors 
 * -log(p(x|z)) = ||Ax - B\mu_t||_{\Sigma_t^{-1}},
 *  which has closed-form expression in computing gradients wrt variables.
 * @version 0.1
 * @date 2023-01-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "optimizer/OptimizerFactorizedGHBase.h"
#include "gp/linear_factor.h"

namespace vimp{
    template <typename CostClass>
    class VIMPFactorizedLinear : public VIMPOptimizerFactorizedBase{
        using Base = VIMPOptimizerFactorizedBase;
        using CostFunction = std::function<double(const VectorXd&, const CostClass&)>;
    public:
        VIMPFactorizedLinear(const int& dimension,
                            int dim_state, 
                            const CostFunction& function, 
                            const CostClass& linear_factor,
                            int num_states,
                            int start_indx):
            Base(dimension, dim_state, num_states, start_indx, true),
            _linear_factor{linear_factor}            
            {
                Base::_func_phi = [this, function, linear_factor](const VectorXd& x){return MatrixXd::Constant(1, 1, function(x, linear_factor));};
                Base::_func_Vmu = [this, function, linear_factor](const VectorXd& x){return (x-Base::_mu) * function(x, linear_factor);};
                Base::_func_Vmumu = [this, function, linear_factor](const VectorXd& x){return MatrixXd{(x-Base::_mu) * (x-Base::_mu).transpose() * function(x, linear_factor)};};
                Base::_gauss_hermite = GaussHermite<GHFunction>{6, dimension, Base::_mu, Base::_covariance, Base::_func_phi};

                _target_mean = linear_factor.get_mean();
                _target_precision = linear_factor.get_precision();
                _A = linear_factor.get_A();
                _B = linear_factor.get_B();
                _const_multiplier = linear_factor.get_C();
            }

    private:
        CostClass _linear_factor;

        MatrixXd _target_mean, _target_precision, _A, _B;

        double _const_multiplier = 1.0;

    public:
        /*Calculating phi * (partial V) / (partial mu), and 
         * phi * (partial V^2) / (partial mu * partial mu^T) for Gaussian posterior: closed-form expression:
         * (partial V) / (partial mu) = Sigma_t{-1} * (mu_k - mu_t)
         * (partial V^2) / (partial mu)(partial mu^T): higher order moments of a Gaussian.
        */
        void calculate_partial_V(){
            // helper vectors
            MatrixXd tmp{MatrixXd::Zero(_dim, _dim)};

            // partial V / partial mu           
            _Vdmu = _const_multiplier * (2 * _A.transpose() * _target_precision * (_A*_mu - _B*_target_mean));
            
            MatrixXd AT_precision_A = _A.transpose() * _target_precision * _A;

            // partial V^2 / partial mu*mu^T
            // update tmp matrix
            for (int i=0; i<(_dim); i++){
                for (int j=0; j<(_dim); j++) {
                    for (int k=0; k<(_dim); k++){
                        for (int l=0; l<(_dim); l++){
                            tmp(i, j) += (_covariance(i, j) * (_covariance(k, l)) + _covariance(i,k) * (_covariance(j,l)) + _covariance(i,l)*_covariance(j,k))*AT_precision_A(k,l);
                        }
                    }
                }
            }

            _Vddmu = _const_multiplier * (_precision * tmp * _precision - _precision * (AT_precision_A*_covariance).trace());
        }

        double fact_cost_value() {
            return _const_multiplier * ((_A.transpose()*_target_precision*_A * _covariance).trace() + 
                    (_A*_mu - _B*_target_mean).transpose() * _target_precision * (_A*_mu - _B*_target_mean));
        }

        double fact_cost_value(const VectorXd& x, const MatrixXd& Cov) {
            return _const_multiplier * ((_A.transpose()*_target_precision*_A * Cov).trace() + 
                    (_A*x - _B*_target_mean).transpose() * _target_precision * (_A*x - _B*_target_mean));
        }

    };
}