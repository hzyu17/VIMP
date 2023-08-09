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

                Base::_func_phi = [this, function, cost_class, temperature](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, function(x, cost_class) / temperature)};};
                Base::_func_Vmu = [this, function, cost_class, temperature](const VectorXd& x){return (x-Base::_mu) * function(x, cost_class) / temperature ;};
                Base::_func_Vmumu = [this, function, cost_class, temperature](const VectorXd& x){return MatrixXd{(x-Base::_mu) * (x-Base::_mu).transpose().eval() * function(x, cost_class) / temperature};};
                
                // Base::_func_phi_highT = [this, function, cost_class, high_temperature](const VectorXd& x){return MatrixXd::Constant(1, 1, function(x, cost_class) / high_temperature );};
                // Base::_func_Vmu_highT = [this, function, cost_class, high_temperature](const VectorXd& x){return (x-Base::_mu) * function(x, cost_class) / high_temperature;};
                // Base::_func_Vmumu_highT = [this, function, cost_class, high_temperature](const VectorXd& x){return MatrixXd{(x-Base::_mu) * (x-Base::_mu).transpose() * function(x, cost_class) / high_temperature};};
                
                // Base::construct_function_T();

                using GH = GaussHermite<GHFunction>;
                Base::_gh = std::make_shared<GH>(GH{6, dimension, Base::_mu, Base::_covariance, Base::_func_phi});
                
            }

    };
}