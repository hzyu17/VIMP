/**
 * @file GVIMPPlanarPRSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The optimizer for planar robots at the joint level.
 * @version 0.1
 * @date 2023-06-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "instances/gvimp/GVIMP3DRobotSDF.h"
#include "robots/PointRobotSDF3D_pgcs.h"

namespace vimp{
    using GVIMP3DPRSDF = GVIMP3DRobotSDF<gpmp2::PointRobotModel, PointRobot3DSDFExample>;

}