/**
 * @file OptimizerPriorColPlanarPR.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The declaration of joint optimizer for planar point robot with prior and collision factors.
 * @version 0.1
 * @date 2022-07-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/OptimizerGH.h"
#include "../include/OptimizerFactorizedTwoFactorsGH.h"
#include "../include/GaussianPriorUnaryTranslation.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

using namespace MPVI;

using UnaryFactorTranslation2D = UnaryFactorTranslation<gtsam::Vector2>;

/**
 * @brief Declaration of the factorized optimizer.
 * 
 */
using FunctionPriorCol = std::function<double(const VectorXd&, const UnaryFactorTranslation2D&, const gpmp2::ObstaclePlanarSDFFactorPointRobot&)>;

using OptFactPriColPRGH = VIMPOptimizerFactorizedTwoClassGH<FunctionPriorCol, UnaryFactorTranslation2D, gpmp2::ObstaclePlanarSDFFactorPointRobot>;
