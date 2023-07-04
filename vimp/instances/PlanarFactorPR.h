/**
 * @file PlanarPRFactor.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Joint optimizer for planar point robot with prior and collision factors.
 * @version 0.1
 * @date 2022-08-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// #pragma once

#include "PlanarFactor.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

namespace vimp{
    auto cost_sdf_pR = cost_obstacle_planar<gpmp2::PointRobotModel>;
    using PlanarSDFFactorPR = OptPlanarSDFFactor<gpmp2::PointRobotModel>;
}// namespace 
