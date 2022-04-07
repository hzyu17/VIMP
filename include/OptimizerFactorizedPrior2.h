//
// Created by hongzhe on 4/6/22.
//

#ifndef MPVI_OPTIMIZERFACTORIZEDPPRIORTWOFACTORS_H
#define MPVI_OPTIMIZERFACTORIZEDPPRIORTWOFACTORS_H

#include "../include/OptimizerFactorizedTwoFactors.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h>
#include "../include/GaussianPriorLinearPlus.h"
using namespace gpmp2;

namespace MPVI {
    using FunctionPriorTwo = std::function<double(const gtsam::Vector&, const GaussianPriorLinearPlus&)>;

    using OptimizerPrior2 = VIMPOptimizerFactorizedTwoFactors<FunctionPriorTwo,
            GaussianPriorLinearPlus,
            gtsam::Vector>;
}
#endif //MPVI_OPTIMIZERFACTORIZEDPPRIORTWOFACTORS_H
