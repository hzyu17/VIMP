/**
 * @file PriorColPlanar.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Class definition for planar robots and obstacles, templated on robot type.
 * @version 0.1
 * @date 2022-08-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// #pragma once

#include "../gp/fixed_prior.h"
#include "../gp/minimum_acc_prior.h"
#include "../optimizer/OptimizerFactorizedOneFactorGH.h"
#include "../optimizer/OptimizerGH.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>

using namespace Eigen;


namespace vimp{

    /**
     * @brief Fixed cost with a covariance
     * @param x input
     * @param fixed_gp 
     * @return double cost
     */
    double cost_fixed_gp(const VectorXd& x, const FixedPriorGP& fixed_gp){
        return fixed_gp.cost(x);
    }

    /**
     * @brief cost for the linear gaussian process.
     * @param pose : the combined two secutive states. [x1; v1; x2; v2]
     * @param gp_minacc : the linear gp object
     * @return double, the cost (-logporb)
     */
    double cost_linear_gp(const VectorXd& pose_cmb, const MinimumAccGP& gp_minacc){
        int dim = gp_minacc.dim_posvel();
        VectorXd x1 = pose_cmb.segment(0, dim);
        VectorXd x2 = pose_cmb.segment(dim, dim);

        return gp_minacc.cost(x1, x2);
    }

    /**
     * Obstacle factor
     * */
    template <typename ROBOT>
    double cost_obstacle(const VectorXd& pose, 
                        const gpmp2::ObstaclePlanarSDFFactor<ROBOT>& obs_factor){
        
        VectorXd vec_err = obs_factor.evaluateError(pose);

        // MatrixXd precision_obs;
        MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
        precision_obs = precision_obs / obs_factor.get_noiseModel()->sigmas()[0];

        double cost = vec_err.transpose().eval() * precision_obs * vec_err;
        
        return cost*800;

    }

    typedef VIMPFactorizedOneCost<FixedPriorGP> FixedGpPrior;
    typedef VIMPFactorizedOneCost<MinimumAccGP> LinearGpPrior;
    template <typename ROBOT>
    using OptPlanarSDFFactor = VIMPFactorizedOneCost<gpmp2::ObstaclePlanarSDFFactor<ROBOT>> ;

}

