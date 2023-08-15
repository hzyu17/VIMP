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

#include "instances/gvimp/GVIMPRobotSDF.h"
#include "robots/WamArmSDFExample.h"

namespace vimp{
    using GVIMPWAMArm = GVIMPRobotSDF<gpmp2::ArmModel, WamArmSDFExample>;
}