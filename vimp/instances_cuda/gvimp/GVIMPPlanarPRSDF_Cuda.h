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

#include "instances_cuda/gvimp/GVIMPPlanarRobotSDF_Cuda.h"
#include "robots/PlanarPointRobotSDF_pgcs.h"

namespace vimp{
    using GVIMPPlanarPRSDF_Cuda = GVIMPPlanarRobotSDF_Cuda<gpmp2::PointRobotModel, PlanarPRSDFExample>;

}