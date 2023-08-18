/**
 * @file ArmFactor.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Define prior and collision factors for Arm Robot.
 * @version 0.1
 * @date 2023-01-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "PriorCol.h"
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>

namespace vimp{

    auto cost_sdf_Arm = cost_obstacle<gpmp2::ArmModel>;
    using OptNonlinearSDFFactorArm = OptSDFNonlinearFactor<gpmp2::ArmModel>;
}// namespace 
