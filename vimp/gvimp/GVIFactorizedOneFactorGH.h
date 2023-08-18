/**
 * @file GVIFactorizedOneFactorGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief factorized optimizer which only takes one cost class. (templated)
 * @version 0.1
 * @date 2022-08-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "gvimp/GVIFactorizedBase.h"

namespace vimp{
    template <typename CostClass>
    class GVIFactorizedOneCost : public GVIFactorizedBase{
        using Base = GVIFactorizedBase;
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;
        using CostFunction = std::function<double(const VectorXd&, const CostClass&)>;
        using GH = GaussHermite<GHFunction>;
        public:
            GVIFactorizedOneCost(const int& dimension,
                                    int dim_state, 
                                    const CostFunction& function, 
                                    const CostClass& cost_class,
                                    int num_states,
                                    int start_indx):
                Base(dimension, dim_state, num_states, start_indx){
                Base::_func_phi = [this, function, cost_class](const VectorXd& x){return MatrixXd::Constant(1, 1, function(x, cost_class));};
                Base::_func_Vmu = [this, function, cost_class](const VectorXd& x){return (x-Base::_mu) * function(x, cost_class);};
                Base::_func_Vmumu = [this, function, cost_class](const VectorXd& x){return MatrixXd{(x-Base::_mu) * (x-Base::_mu).transpose().eval() * function(x, cost_class)};};
                Base::_gh = std::make_shared<GH>(GH{6, dimension, Base::_mu, Base::_covariance, Base::_func_phi});
            }

    };
}