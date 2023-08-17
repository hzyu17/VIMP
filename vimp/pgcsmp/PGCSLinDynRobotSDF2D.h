/**
 * @file PGCSLinDynRobotSDF2D.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief PGCS-MP for 3D SDF.
 * @version 0.1
 * @date 2023-07-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "PGCSLinDynRobotSDF.h"

namespace vimp{
    template<typename RobotSDF>
    using PGCSLinDynRobotSDF2D = PGCSLinDynRobotSDF<RobotSDF, Eigen::Vector2d> ;
}
