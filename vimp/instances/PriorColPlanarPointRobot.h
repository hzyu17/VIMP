/**
 * @file PriorColPlanarPointRobot.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Joint optimizer for planar point robot with prior and collision factors.
 * @version 0.1
 * @date 2022-07-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// #pragma once

#include <vimp/instances/PriorCol.h>
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

using namespace Eigen;

namespace vimp{
    typedef UnaryFactorTranslation<gtsam::Vector2> UnaryFactorTranslation2D;
    typedef OptFactPriColGH<gtsam::Vector2, gpmp2::PointRobotModel> OptFactPriColPlanarPRGH;

    typedef OptFactPriColGHInstance<gtsam::Vector2, gpmp2::PointRobotModel> OptFactPriColGHInstancePointRobot;
    typedef VIMPOptimizerGH<OptFactPriColGHInstancePointRobot> VIMPOptimizerPriColPR;
}   

