/**
 * @file PlanarOptimizerRobotSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The optimizer for planar robots at the joint level.
 * @version 0.1
 * @date 2023-06-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../helpers/ExperimentParams.h"

namespace vimp{

template <typename Robot, typename RobotSDF>
class PlanarOptimizerRobotSDF{
public:
PlanarOptimizerRobotSDF(){}
PlanarOptimizerRobotSDF(GVIMPExperimentParams& params):

{

}

};

protected: 
    RobotSDF _robot_sdf;
    double _eps_sdf;
    double _Sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    CostHelper _cost_helper;

}