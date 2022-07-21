/**
 * @file PriorColPlanarPR.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Joint optimizer for planar point robot with prior and collision factors.
 * @version 0.1
 * @date 2022-07-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "PriorCol.h"
#include <gpmp2/kinematics/PointRobotModel.h>

namespace vimp{
    using UnaryFactorTranslation2D = UnaryFactorTranslation<gtsam::Vector2>;
    using OptFactPriColPlanarPRGH = OptFactPriColGH<gtsam::Vector2, gpmp2::PointRobotModel>;
}

