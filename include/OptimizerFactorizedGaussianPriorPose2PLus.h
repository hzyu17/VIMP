//
// Created by hongzhe on 4/6/22.
//

#ifndef MPVI_OPTIMIZERFACTORIZEDGAUSSIANPRIORPOSE2PLUS_H
#define MPVI_OPTIMIZERFACTORIZEDGAUSSIANPRIORPOSE2PLUS_H
#include "../include/OptimizerFactorizedOneFactor.h"
#include "../include/GaussianPriorPose2Plus.h"
using namespace gtsam;

namespace MPVI {
    using Function_prior = std::function<double(const gtsam::Vector&, const UnaryFactorVector2&)>;

    using OptimizerOnePrior = VIMPOptimizerFactorizedOneFactor<Function_prior, UnaryFactorVector2, gtsam::Vector>;
}

#endif //MPVI_OPTIMIZERFACTORIZEDGAUSSIANPRIORPOSE2PLUS_H
