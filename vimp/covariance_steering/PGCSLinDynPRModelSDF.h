/**
 * @file PGCSLinDynPRModelPlanarSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient algorithm for nonlinear covariance steering with plannar obstacles, linear dynamics. 
 * @version 0.1
 * @date 2023-03-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "PGCSLinDynRobotSDF.h"
#include "robots/PointRobotSDF3D_pgcs.h"

namespace vimp{
    using PGCSLinDynPRModelSDF = PGCSLinDynRobotSDF<PointRobot3DSDFExample>;
}