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

#include "../instances/CostFunctions.h"
#include "../gvimp/GVIFactorizedNonlinerOneFactorGH.h"
#include "../gvimp/GVI-GH.h"
#include "../gvimp/GVIFactorizedLinear.h"

using namespace Eigen;


namespace vimp{

    // typedef VIMPFactorizedOneCost<FixedPriorGP> FixedGpPriorOneCost;
    typedef GVIFactorizedLinear<FixedPriorGP> FixedGpPrior;
    typedef GVIFactorizedLinear<MinimumAccGP> LinearGpPrior;
    template <typename ROBOT>
    using OptPlanarSDFFactor = GVIFactorizedNonlinerOneFactorGH<gpmp2::ObstaclePlanarSDFFactor<ROBOT>> ;

}

