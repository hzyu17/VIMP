/**
 * @file FactorizedGVIPlanarNGD.h
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
#include "GaussianVI/ngd/NGDFactorizedNonlinerGH.h"
#include "GaussianVI/ngd/NGD-GH.h"
#include "GaussianVI/ngd/NGDFactorizedLinear.h"
#include "GaussianVI/ngd/NGDFactorizedFixedGaussian.h"

using namespace Eigen;


namespace vimp{

    typedef NGDFactorizedFixedGaussian<FixedPriorGP> FixedGpPrior;
    typedef NGDFactorizedLinear<MinimumAccGP> LinearGpPrior;
    template <typename ROBOT>
    using NGDFactorizedPlanarSDF = NGDFactorizedNonlinerGH<gpmp2::ObstaclePlanarSDFFactor<ROBOT>>;

}

