/**
 * @file PlanarArmFactor.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Planar case for Arm
 * @version 0.1
 * @date 2022-08-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "instances/FactorizedGVIPlanarNGD.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorArm.h>

namespace vimp{

    auto cost_sdf_Arm = cost_obstacle_planar<gpmp2::ArmModel>;
    using OptPlanarSDFFactorArm = NGDFactorizedPlanarSDF<gpmp2::ArmModel>;
}// namespace 
