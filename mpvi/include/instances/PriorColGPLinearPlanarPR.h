/**
 * @file PriorColGPLinearPlanarPR.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Joint optimizer for planar point robot with GP interpolated 
 * prior and collision factors.
 * @version 0.1
 * @date 2022-07-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "PriorColGPInter.h"
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>

namespace vimp{

    using ObstacleFactorGPInterLinPR= gpmp2::ObstacleSDFFactorGP<gpmp2::PointRobotModel, gpmp2::GaussianProcessInterpolatorLinear>;

    using OptFactPriColGHGPInterLinPR = OptFactPriColGHGPInter<gpmp2::PointRobotModel, gpmp2::GaussianProcessInterpolatorLinear>;

}
