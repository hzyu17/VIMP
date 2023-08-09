/**
 * @file PlanarFactor.h
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
#include "gvimp/GVIFactorizedNonlinerOneFactorGH.h"
#include "gvimp/GVI-GH.h"
#include "gvimp/GVIFactorizedLinear.h"
#include "gvimp/GVIFactorizedFixedGaussian.h"

using namespace Eigen;


namespace vimp{

    // typedef VIMPFactorizedOneCost<FixedPriorGP> FixedGpPriorOneCost;
    typedef GVIFactorizedFixedGaussian<FixedPriorGP> FixedGpPrior;
    typedef VIMPFactorizedLinear<MinimumAccGP> LinearGpPrior;
    template <typename ROBOT>
    using OptSDFNonlinearFactor = GVIFactorizedNonlinerOneFactorGH<gpmp2::ObstacleSDFFactor<ROBOT>> ;

}

