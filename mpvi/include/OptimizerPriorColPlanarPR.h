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

#include "../include/OptimizerFactorizedTwoFactorsGH.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>
#include "../include/Optimizer.h"


/**
 * @brief Declaration of the factorized optimizer.
 * 
 */
using PriorClass = UnaryFactorTranslation2D;
using CollisionClass = ObstaclePlanarSDFFactorPointRobot;
using FunctionPriorCol = std::function<double(const VectorXd&, const PriorClass&, const CollisionClass&)>;

using OptFactPriColPRGH = VIMPOptimizerFactorizedTwoClassGH<FunctionPriorCol, PriorClass, CollisionClass, VectorXd>;
