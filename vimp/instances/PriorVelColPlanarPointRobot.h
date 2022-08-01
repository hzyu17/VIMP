/**
 * @file PriorVelColPlanarPointRobot.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Joint optimizer for planar point robot with prior and collision factors.
 * @version 0.1
 * @date 2022-07-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// #pragma once

#include "../instances/PriorColThreeFactors.h"
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

using namespace Eigen;

namespace vimp{
    typedef UnaryFactorTranslation<gtsam::Vector2> UnaryFactorTranslation2D;
    typedef OptFactPriVelColGH<gtsam::Vector2, gpmp2::PointRobotModel> OptFactPriVelColPlanarPRGH;

}   

