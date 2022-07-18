/**
 * @file PriorColGPInter.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Joint optimizer with prior and collision factors using Gaussian
 * process interpolation, templated for robot model.
 * @version 0.1
 * @date 2022-07-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gpmp2/gp/GPutils.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGP.h>
#include "../optimizer/OptimizerFactorizedTwoFactorsGH.h"

using namespace Eigen;

namespace vimp{
    template<class ROBOT, class GPINTER>
    using FunctionPriorColGPInter = std::function<double(const VectorXd&, const gpmp2::GaussianProcessPriorLinear&, 
                                     const gpmp2::ObstacleSDFFactorGP<ROBOT, GPINTER>&)>;

    template<class ROBOT, class GPINTER>
    using OptFactPriColGHGPInter = VIMPOptimizerFactorizedTwoClassGH<FunctionPriorColGPInter<ROBOT, GPINTER>, 
                                                                gpmp2::GaussianProcessPriorLinear, 
                                                                gpmp2::ObstacleSDFFactorGP<ROBOT, GPINTER>>;

}