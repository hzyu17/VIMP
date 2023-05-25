/**
 * @file PGCSLinDynArmModelPlanarSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient algorithm for nonlinear covariance steering with plannar obstacles, arm robot model. 
 * @version 0.1
 * @date 2023-03-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include "PGCSLinDynRobotSDF.h"
#include "../robots/WamArmSDFExample.h"

namespace vimp{
    using PGCSLinDynArmSDF = PGCSLinDynRobotSDF<WamArmSDFExample>;
}
