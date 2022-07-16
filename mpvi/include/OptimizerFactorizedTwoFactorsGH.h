/**
 * @file OptimizerFactorizedTwoFactorsGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Optimizer. cost function takes two cost classes.
 * @version 0.1
 * @date 2022-07-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/OptimizerFactorizedGHBase.h"

namespace MPVI{
    template <typename Function, typename CostClass, typename CostClass1>
    /// Decription: The marginal optimizer using Gauss-Hermite quadrature to calculate the expectations
    class VIMPOptimizerFactorizedTwoClassGH: public VIMPOptimizerFactorizedBase<Function>{
    using Base = VIMPOptimizerFactorizedBase<Function>;
    using GHFunction = std::function<MatrixXd(const VectorXd&)>;
    public:
        ///@param dimension The dimension of the state
        ///@param function_ Template function class which calculate the cost
        ///@param cost_class_ CostClass
        ///@param cost_class_1 CostClass1
        VIMPOptimizerFactorizedTwoClassGH(const int& dimension, 
                                          const Function& function_, 
                                          const CostClass& cost_class_,
                                          const CostClass1& cost_class1_,
                                          const MatrixXd& Pk_):
                Base(dimension, function_, Pk_),
                _cost_class{cost_class_},
                _cost_class1{cost_class1_},
                _cost_function{function_}{
                Base::_func_phi = [this](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, _cost_function(x, _cost_class, _cost_class1))};};
                Base::_func_Vmu = [this](const VectorXd& x){return (x-Base::_mu) * _cost_function(x, _cost_class, _cost_class1);};
                Base::_func_Vmumu = [this](const VectorXd& x){return MatrixXd{(x-Base::_mu) * (x-Base::_mu).transpose().eval() * _cost_function(x, _cost_class, _cost_class1)};};
                Base::_gauss_hermite = GaussHermite<GHFunction>{10, Base::_dim, Base::_mu, Base::_covariance, Base::_func_phi};
                }
                
    private:
        
        /// Class of cost sources
        CostClass _cost_class;
        CostClass1 _cost_class1;

        /// cost function
        Function _cost_function;
    };
}