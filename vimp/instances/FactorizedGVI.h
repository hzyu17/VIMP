/**
 * @file Factors.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Class definition for planar robots and obstacles, templated on robot type.
 * @version 0.1
 * @date 2022-08-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// #pragma once

#include "instances/CostFunctions.h"
#include "GaussianVI/GVIFactorizedNonlinerGH.h"
#include "GaussianVI/GVI-GH.h"
#include "GaussianVI/GVIFactorizedLinear.h"
#include "GaussianVI/GVIFactorizedFixedGaussian.h"

using namespace Eigen;


namespace vimp{

    // typedef VIMPFactorizedOneCost<FixedPriorGP> FixedGpPriorOneCost;
    typedef GVIFactorizedFixedGaussian<FixedPriorGP> FixedGpPrior;
    typedef GVIFactorizedLinear<MinimumAccGP> LinearGpPrior;
    template <typename ROBOT>
    using GVIFactorizedSDF = GVIFactorizedNonlinerGH<gpmp2::ObstacleSDFFactor<ROBOT>> ;

}

