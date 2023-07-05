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
#include "gvimp/GVIFactorizedTwoFactorsGH.h"

using namespace Eigen;

namespace vimp{

    class GPInterLinearExtend:public gpmp2::GaussianProcessPriorLinear{
        public: 
            GPInterLinearExtend(gtsam::Key poseKey1, gtsam::Key velKey1,
                                gtsam::Key poseKey2, gtsam::Key velKey2,
                                double delta_t, const gtsam::SharedNoiseModel Qc_model):
                                inv_Qc_{gpmp2::calcQ_inv(Qc_model, delta_t)},
                gpmp2::GaussianProcessPriorLinear(poseKey1, velKey1, poseKey2, velKey2, delta_t, Qc_model)
            {}
            MatrixXd inv_Qc(){
                return inv_Qc_;
            }

        private:
            MatrixXd inv_Qc_;
    };

    template<class ROBOT, class GPINTER>
    using FunctionPriorColGPInter = std::function<double(const VectorXd&, GPInterLinearExtend&, 
                                     const gpmp2::ObstacleSDFFactorGP<ROBOT, GPINTER>&)>;

    template<class ROBOT, class GPINTER>
    using OptFactPriColGHGPInter = VIMPOptimizerFactorizedTwoClassGH<FunctionPriorColGPInter<ROBOT, GPINTER>, 
                                                                GPInterLinearExtend, 
                                                                gpmp2::ObstacleSDFFactorGP<ROBOT, GPINTER>>;

}