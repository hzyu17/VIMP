/**
 * @file GVIMPArm2SDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The optimizer for a 2-link arm at the joint level.
 * @version 0.1
 * @date 2023-06-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "instances/gvimp/GVIMPPlanarRobotSDF.h"
#include "robots/PlanarArm2SDF_pgcs.h"

namespace vimp{
    using GVIMPArm2SDF = GVIMPPlanarRobotSDF<gpmp2::ArmModel, PlanarArm2SDFExample>;

}