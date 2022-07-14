/**
 * Optimizer for the cost functions on a single config, including the prior and the collision.
 * For Point Robot.
 * Author: Hongzhe Yu
 * Date: 5/26/22
 */

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>
#include "GaussianPriorUnaryTranslation.h"
#include "../include/OptimizerFactorizedPriorCollisionGH.h"

using namespace MPVI;
using namespace gpmp2;

using UnaryFactorTranslation2D = UnaryFactorTranslation<gtsam::Vector2>;

typedef VIMPOptimizerFactorizedGaussHermite<std::function<double(const gtsam::Vector&, const UnaryFactorTranslation2D&, const ObstaclePlanarSDFFactorPointRobot&)>,
        const gtsam::Vector&,
        const UnaryFactorTranslation2D&,
        const ObstaclePlanarSDFFactorPointRobot&>
        OptimizerFactorizedPriorColPlanarPointRobot;
