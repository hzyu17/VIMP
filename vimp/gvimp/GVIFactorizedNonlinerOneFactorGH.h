/**
 * @file GVIFactorizedNonlinerOneFactorGH.h
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
    class GVIFactorizedNonlinerOneFactorGH : public GVIFactorizedBase{
        using Base = GVIFactorizedBase;
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;
        using CostFunction = std::function<double(const VectorXd&, const CostClass&)>;
        public:
            GVIFactorizedNonlinerOneFactorGH(int dimension,
                                            int dim_state,
                                            const CostFunction& function, 
                                            const CostClass& cost_class,
                                            int num_states,
                                            int start_indx,
                                            double temperature, 
                                            double high_temperature):
                Base(dimension, dim_state, num_states, start_indx, temperature, high_temperature){

                Base::_func_phi = std::make_shared<GHFunction>([this, function, cost_class](const VectorXd& x){return MatrixXd::Constant(1, 1, function(x, cost_class));});
                Base::_func_Vmu = std::make_shared<GHFunction>([this, function, cost_class](const VectorXd& x){return VectorXd::Ones(this->_dim) * function(x, cost_class);});
                Base::_func_Vmumu = std::make_shared<GHFunction>([this, function, cost_class](const VectorXd& x){return MatrixXd{(x-Base::_mu) * (x-Base::_mu).transpose().eval() * function(x, cost_class)};});
                
                // Base::construct_function_T();

                using GH = GaussHermite<GHFunction>;
                Base::_gh = std::make_shared<GH>(GH{6, dimension, Base::_mu, Base::_covariance});
                
            }

    };
}