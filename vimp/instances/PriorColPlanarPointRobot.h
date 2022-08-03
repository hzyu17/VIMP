/**
 * @file PriorColPlanarPointRobot.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Joint optimizer for planar point robot with prior and collision factors.
 * @version 0.1
 * @date 2022-08-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// #pragma once

#include "../gp/fixed_prior.h"
#include "../gp/minimum_acc_prior.h"
#include "../optimizer/OptimizerFactorizedOneFactorGH.h"
#include "../optimizer/OptimizerGH.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

using namespace Eigen;


namespace vimp{

    double cost_fixed_gp(const VectorXd& x, const FixedPriorGP& fixed_gp){
        return fixed_gp.cost(x);
    }

    /**
     * @brief cost for the linear gaussian process.
     * 
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
    double cost_obstacle(const VectorXd& pose, 
                        const gpmp2::ObstaclePlanarSDFFactorPointRobot& obs_factor){
        
        VectorXd vec_err = obs_factor.evaluateError(pose);

        // MatrixXd precision_obs;
        MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
        precision_obs = precision_obs / obs_factor.get_noiseModel()->sigmas()[0];

        double cost = vec_err.transpose().eval() * precision_obs * vec_err;
        
        return cost*600;

    }

    typedef VIMPFactorizedOneCost<FixedPriorGP> FixedGpPrior;
    typedef VIMPFactorizedOneCost<MinimumAccGP> LinearGpPrior;
    typedef VIMPFactorizedOneCost<gpmp2::ObstaclePlanarSDFFactorPointRobot> PlanarPRColFactor;

}

