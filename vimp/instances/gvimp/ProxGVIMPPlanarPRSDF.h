/**
 * @file GVIMPPlanarPRSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The optimizer for planar robots at the joint level.
 * @version 0.1
 * @date 2024-05-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "instances/gvimp/ProxGVIMPPlanarRobotSDF.h"
#include "robots/PlanarPointRobotSDF_pgcs.h"

namespace vimp{
    using ProxGVIMPPlanarPRSDF = ProxGVIMPPlanarRobotSDF<gpmp2::PointRobotModel, PlanarPRSDFExample>;

}