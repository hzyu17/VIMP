/**
 * @file OptimizerFactorizedPriorPointRobot.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/// Define the optimizer for planar point robot with one prior and obstacle, and without interpolations.
#include "GaussianPriorLinearGetQc.h"
#include "GaussianPriorUnaryTranslation.h"
#include "../include/OptimizerFactorizedGH.h"

using namespace MPVI;
using namespace gpmp2;
using namespace Eigen;

using UnaryFactorTranslation2D = UnaryFactorTranslation<gtsam::Vector2>;

/// Function calculating two cost factors: prior and the collision factor. 
inline double errorWrapperSinglePrior(const gtsam::Vector& theta, const UnaryFactorTranslation2D& prior_factor) {

    /**
     * Prior factor
     * */
    gtsam::Vector2 position;
    position = theta.segment<2>(0);
    VectorXd vec_prior_err = prior_factor.evaluateError(position);
    MatrixXd K = prior_factor.get_Qc();
    return vec_prior_err.transpose() * K.inverse() * vec_prior_err;

}

/// The instantiation of the template defined in the calss OptimizerFactorizedPriorColGH. The
typedef VIMPOptimizerFactorizedGaussHermite<std::function<double(const gtsam::Vector&, 
                                                            const UnaryFactorTranslation2D&)>,
        const UnaryFactorTranslation2D&,
        const gtsam::Vector&>
        OptimizerFactorPriorPointRobotGH;
