/**
 * @file OptimizerFactorizedPriorCollisionPointRobot.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/// Define the optimizer for planar point robot with one prior and obstacle, and without interpolations.
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>
#include "GaussianPriorLinearGetQc.h"
#include "GaussianPriorUnaryTranslation.h"
#include "../include/OptimizerFactorizedPriorCollisionGH.h"

using namespace MPVI;
using namespace gpmp2;
using namespace Eigen;

using UnaryFactorTranslation2D = UnaryFactorTranslation<gtsam::Vector2>;

/// Function calculating two cost factors: prior and the collision factor. 
inline double errorWrapperSinglePrior(const gtsam::Vector& theta, const UnaryFactorTranslation2D& prior_factor,
                                      const ObstaclePlanarSDFFactorPointRobot& obstacle_factor) {

    /**
     * Prior factor
     * */
    gtsam::Vector2 position;
    position = theta.segment<2>(0);
    VectorXd vec_prior_err = prior_factor.evaluateError(position);
    MatrixXd K = prior_factor.get_Qc();
    double prior_err = vec_prior_err.transpose() * K.inverse() * vec_prior_err;

    /**
     * Obstacle factor
     * */
    VectorXd vec_err = obstacle_factor.evaluateError(theta);

    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    double sig_obs = 1.0;
    precision_obs = precision_obs / sig_obs;

    double collision_cost = vec_err.transpose() * precision_obs * vec_err;

    return prior_err + collision_cost;
}

/// The instantiation of the template defined in the calss OptimizerFactorizedPriorColGH. The
typedef OptimizerFactorizedPriorColGH<std::function<double(const gtsam::Vector&, 
                                                            const UnaryFactorTranslation2D&, 
                                                            const ObstaclePlanarSDFFactorPointRobot&)>,
        const gtsam::Vector&,
        const UnaryFactorTranslation2D&,
        const ObstaclePlanarSDFFactorPointRobot&>
        OptimizerFactorPriorColPointRobotGH;
