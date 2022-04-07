//
// Created by hongzhe on 4/6/22.
//

#ifndef MPVI_OPTIMIZERFACTORIZEDPRIORANDOBSTACLEGPPOINTROBOT_H
#define MPVI_OPTIMIZERFACTORIZEDPRIORANDOBSTACLEGPPOINTROBOT_H

#include "../include/OptimizerFactorizedTwoFactors.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h>
#include "../include/GaussianPriorLinearPlus.h"
using namespace gpmp2;

namespace MPVI {
    using Function_interp = std::function<double(const gtsam::Vector&, GaussianPriorLinearPlus,
                                                 const ObstaclePlanarSDFFactorGPPointRobot&)>;

    using OptimizerInterp = VIMPOptimizerFactorizedTwoFactors<Function_interp,
            GaussianPriorLinearPlus,
            ObstaclePlanarSDFFactorGPPointRobot,
            gtsam::Vector>;
}
#endif //MPVI_OPTIMIZERFACTORIZEDPRIORANDOBSTACLEGPPOINTROBOT_H
