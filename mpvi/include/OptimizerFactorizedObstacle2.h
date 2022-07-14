//
// Created by hongzhe on 4/7/22.
//

#ifndef MPVI_OPTIMIZERFACTORIZEDOBSTACLEINTERPOLATION_H
#define MPVI_OPTIMIZERFACTORIZEDOBSTACLEINTERPOLATION_H
#include "OptimizerFactorizedTwoFactors.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h>

using namespace gpmp2;

namespace MPVI {
    using FunctionCollisionInterp = std::function<double(const gtsam::Vector&, const ObstaclePlanarSDFFactorGPPointRobot&)>;

    using OptimizerCollision2 = VIMPOptimizerFactorizedTwoFactors<FunctionCollisionInterp,
            ObstaclePlanarSDFFactorGPPointRobot,
            gtsam::Vector>;
}
#endif //MPVI_OPTIMIZERFACTORIZEDOBSTACLEINTERPOLATION_H
