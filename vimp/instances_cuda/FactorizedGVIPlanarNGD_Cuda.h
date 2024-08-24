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
#include "GaussianVI/gp/factorized_opts_linear_Cuda.h"
#include "instances_cuda/CostFunctions.h"
#include "GaussianVI/ngd/NGDFactorizedBaseGH_Cuda.h"
#include "GaussianVI/ngd/NGD-GH.h"
#include <gpmp2/kinematics/PointRobotModel.h>

namespace vimp{
    template <typename ROBOT>
    using NGDFactorizedPlanarSDF_Cuda = gvi::NGDFactorizedBaseGH_Cuda<gpmp2::ObstaclePlanarSDFFactor<ROBOT>>;
}

