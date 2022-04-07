//
// Created by hongzhe on 4/6/22.
//

#ifndef MPVI_OPTIMIZERFACTORIZEDGAUSSIANPRIORPOSVEL_H
#define MPVI_OPTIMIZERFACTORIZEDGAUSSIANPRIORPOSVEL_H
#include "../include/OptimizerFactorizedOneFactor.h"
#include "../include/GaussianPriorUnaryTranslation.h"
using namespace gtsam;

namespace MPVI {
    using Function_prior = std::function<double(const gtsam::Vector&, const UnaryFactorTranslation<gtsam::Vector2>&)>;

    using OptimizerOnePrior = VIMPOptimizerFactorizedOneFactor<Function_prior, UnaryFactorTranslation<gtsam::Vector2>, gtsam::Vector>;
}

#endif //MPVI_OPTIMIZERFACTORIZEDGAUSSIANPRIORPOSVEL_H
