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

#pragma once
#include "GaussianVI/gp/factorized_opts_linear.h"
#include "instances/CostFunctions.h"
#include "GaussianVI/ngd/NGDFactorizedNonlinerGH.h"
#include "GaussianVI/ngd/NGD-GH.h"

namespace vimp{
    template <typename ROBOT>
    using NGDFactorizedPlanarSDF = gvi::NGDFactorizedNonlinerGH<gpmp2::ObstaclePlanarSDFFactor<ROBOT>>;

}

