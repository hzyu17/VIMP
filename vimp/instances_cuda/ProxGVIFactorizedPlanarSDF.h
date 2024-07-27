/**
 * @file ProxGVIFactorizedPlanarSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Class definition for planar robots and obstacles, templated on robot type.
 * @version 0.1
 * @date 2024-05-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

// #pragma once
#include "GaussianVI/gp/factorized_opts_linear_prox.h"
#include "instances/CostFunctions.h"
#include "GaussianVI/proxgd/ProxGVIFactorizedBaseGH.h"
#include "GaussianVI/proxgd/ProxGVI-GH.h"

namespace vimp{
    template <typename ROBOT>
    using ProxGVIFactorizedPlanarSDF = gvi::ProxGVIFactorizedBaseGH<gpmp2::ObstaclePlanarSDFFactor<ROBOT>>;

}

