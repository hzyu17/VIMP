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
#include "GaussianVI/ngd/NGDFactorizedNonlinerGH.h"
#include "GaussianVI/gp/factorized_opts_linear.h"
#include "GaussianVI/ngd/NGD-GH.h"

using namespace Eigen;


namespace vimp{
    template <typename ROBOT>
    using GVIFactorizedSDF = gvi::NGDFactorizedNonlinerGH<gpmp2::ObstacleSDFFactor<ROBOT>> ;

}

