/**
 * @file GVIFactorizedThreeFactorsGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Optimizer. cost function takes three cost classes.
 * @version 0.1
 * @date 2022-07-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "GVIFactorizedGHBase.h"

namespace vimp{
    template <typename Function, typename CostClass, typename CostClass1, typename CostClass2>
    /// Decription: The marginal optimizer using Gauss-Hermite quadrature to calculate the expectations
    class GVIFactorizedThreeClassGH: public GVIFactorizedBase{
    using Base = GVIFactorizedBase;
    using GHFunction = std::function<MatrixXd(const VectorXd&)>;
    public:
        /// Default constructor
        GVIFactorizedThreeClassGH(){};

        /// Constructor
        ///@param dimension The dimension of the state
        ///@param function_ Template function class which calculate the cost
        ///@param cost_class_ CostClass
        ///@param cost_class_1 CostClass1
        GVIFactorizedThreeClassGH(const int& dimension, 
                                          const Function& function_, 
                                          const CostClass& cost_class_,
                                          const CostClass1& cost_class1_,
                                          const CostClass2& cost_class2_,
                                          const MatrixXd& Pk_):
                Base(dimension, Pk_){
                Base::_func_phi = [this, function_, cost_class_, cost_class1_, cost_class2_](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, function_(x, cost_class_, cost_class1_, cost_class2_))};};
                Base::_func_Vmu = [this, function_, cost_class_, cost_class1_, cost_class2_](const VectorXd& x){return (x-Base::_mu) * function_(x, cost_class_, cost_class1_, cost_class2_);};
                Base::_func_Vmumu = [this, function_, cost_class_, cost_class1_, cost_class2_](const VectorXd& x){return MatrixXd{(x-Base::_mu) * (x-Base::_mu).transpose().eval() * function_(x, cost_class_, cost_class1_, cost_class2_)};};
                Base::_gh = GaussHermite<GHFunction>{6, Base::_dim, Base::_mu, Base::_covariance, Base::_func_phi};
                }
    };
}