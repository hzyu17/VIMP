/**
 * @file PriorColPlanar.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Class definition for planar robots and obstacles, templated on robot type.
 * @version 0.1
 * @date 2022-08-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// #pragma once

#include "CostFunctions.h"
#include "optimizer/OptimizerFactorizedNonlinerOneFactorGH.h"
#include "optimizer/OptimizerGH.h"
#include "optimizer/OptimizerFactorizedLinear.h"

using namespace Eigen;


namespace vimp{

    // typedef VIMPFactorizedOneCost<FixedPriorGP> FixedGpPriorOneCost;
    typedef VIMPFactorizedLinear<FixedPriorGP> FixedGpPrior;
    typedef VIMPFactorizedLinear<MinimumAccGP> LinearGpPrior;
    template <typename ROBOT>
    using OptSDFNonlinearFactor = OptimizerFactorizedNonlinerOneFactorGH<gpmp2::ObstacleSDFFactor<ROBOT>> ;

}

