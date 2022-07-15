/**
 * @file OptimizerFactorizedTwoFactorsGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Optimizer with two cost classes. Inheritance from the one cost class optimizer class.
 * @version 0.1
 * @date 2022-07-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/OptimizerFactorizedGH.h"

namespace MPVI{
    template <typename Function, typename CostClass, typename CostClass1, typename... Args>
    using Base = VIMPOptimizerFactorizedGaussHermite<Function, CostClass>;
    /// Decription: The marginal optimizer using Gauss-Hermite quadrature to calculate the expectations
    class VIMPOptimizerFactorizedTwoClassGH: public Base{
    public:
        ///@param dimension The dimension of the state
        ///@param function_ Template function class which calculate the cost
        ///@param cost_class_ CostClass
        ///@param cost_class_1 CostClass1
        VIMPOptimizerFactorizedTwoClassGH(const int& dimension, 
                                          const Function& function_, 
                                          const CostClass& cost_class_,
                                          const CostClass1& cost_class1_):
                Base(dimension, function_, cost_class_),
                _cost_class1{cost_class1_},
                _func_phi{[this](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, _cost_function(x, Base::_cost_class, _cost_class1))};}},
                _func_Vmu{[this](const VectorXd& x){return (x-Base::_mu) * _cost_function(x, Base::_cost_class, _cost_class1);}},
                _func_Vmumu{[this](const VectorXd& x){return MatrixXd{(x-Base::_mu) * (x-Base::_mu).transpose().eval() * _cost_function(x, Base::_cost_class, _cost_class1)};}},
                _gauss_hermite{10, Base::_dim, Base::_mu, Base::_covariance, _func_phi}
                {}
    protected:
        
        /// Class of cost sources
        CostClass1 _cost_class1;

        /// Intermediate functions for Gauss-Hermite quadratures
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;
        GHFunction _func_phi;
        GHFunction _func_Vmu;
        GHFunction _func_Vmumu;

        /// G-H quadrature class
        GaussHermite<GHFunction> _gauss_hermite;
    };
}