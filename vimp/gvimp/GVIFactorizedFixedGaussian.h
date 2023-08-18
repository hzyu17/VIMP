/**
 * @file GVIFactorizedFixedGaussian.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Factorized optimization steps for fixed linear gaussian factors 
 * -log(p(x|z)) = ||X - \mu_t||_{\Sigma_t^{-1}},
 *  which has closed-form expression in computing gradients wrt variables.
 * The difference from the linear dynamics prior is that for this factor, 
 * temperature and high temperature are both 1.0;
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "gvimp/GVIFactorizedLinear.h"
#include "gp/linear_factor.h"

namespace vimp{
    template <typename LinearFactor>
    class GVIFactorizedFixedGaussian : public GVIFactorizedLinear<LinearFactor>{
        using Base = GVIFactorizedLinear<LinearFactor>;
        using CostFunction = std::function<double(const VectorXd&, const LinearFactor&)>;
    public:
        GVIFactorizedFixedGaussian(const int& dimension,
                                    int dim_state,
                                    const CostFunction& function, 
                                    const LinearFactor& linear_factor,
                                    int num_states,
                                    int start_indx,
                                    double temperature,
                                    double high_temperature):
            Base(dimension, dim_state, function, linear_factor, num_states, start_indx, temperature, high_temperature)
            { }    
    };
}