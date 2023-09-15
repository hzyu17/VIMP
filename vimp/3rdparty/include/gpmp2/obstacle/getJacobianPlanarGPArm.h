//
// Created by hongzhe on 2/13/22.
//

#pragma once

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/obstacle/getJacobianPlanarGP.h>

namespace gpmp2 {

// template uses PointRobotModel as robot type
    typedef GetJacobianGP<ArmModel, GaussianProcessInterpolatorLinear>
            GetJacobianGPArm;

}