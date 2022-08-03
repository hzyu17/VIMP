/**
 * @file PriorColPlanarPointRobot.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Joint optimizer for planar point robot with prior and collision factors.
 * @version 0.1
 * @date 2022-08-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// // #pragma once

#include "PriorColPlanar.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

namespace vimp{

    auto cost_sdf_pR = cost_obstacle<gpmp2::PointRobotModel>;
    using OptPlanarSDFFactorPointRobot = OptPlanarSDFFactor<gpmp2::PointRobotModel>;
}// namespace 
