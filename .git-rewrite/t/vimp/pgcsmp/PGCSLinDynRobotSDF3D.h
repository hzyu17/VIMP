/**
 * @file PGCSLinDynRobotSDF3D.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief PGCS-MP for Planar SDF.
 * @version 0.1
 * @date 2023-07-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "pgcsmp/PGCSLinDynRobotSDF.h"

namespace vimp{
    template<typename RobotSDF>
    using PGCSLinDynRobotSDF3D = PGCSLinDynRobotSDF<RobotSDF, Eigen::Vector3d> ;
}
