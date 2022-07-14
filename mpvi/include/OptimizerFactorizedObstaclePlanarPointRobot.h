//
// Created by hongzhe on 4/6/22.
//

#ifndef MPVI_OPTIMIZERFACTORIZEDOBSTACLEPLANARPOINTROBOT_H
#define MPVI_OPTIMIZERFACTORIZEDOBSTACLEPLANARPOINTROBOT_H
#include "OptimizerFactorizedOneFactorPointRobot.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>
using namespace gpmp2;

namespace MPVI {
    using Function_collision = std::function<double(const gtsam::Vector&, const ObstaclePlanarSDFFactorPointRobot&)>;

    using OptimizerOneCollision = VIMPOptimizerFactorizedOneFactor<Function_collision,
            ObstaclePlanarSDFFactorPointRobot,
            gtsam::Vector>;
}

#endif //MPVI_OPTIMIZERFACTORIZEDOBSTACLEPLANARPOINTROBOT_H
