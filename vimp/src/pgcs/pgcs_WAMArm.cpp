/**
 * @file pgcs_pR_RobotModel.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief pgcs with plannar obstacles and arm robot, 
 * using Robot Model which has a vector of balls to check collisions.
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

// #include "dynamics/LinearDynamics.h"
#include "covariance_steering/PGCSLinDynArmSDF.h"
#include "helpers/experiment_runner_7D.h"

using namespace Eigen;
using namespace vimp;

int main(){
    
    int num_exp = 3;
    std::string config_file{"/home/hongzhe/git/VIMP/vimp/configs/pgcs/wam_arm.xml"};
    ExperimentRunner7D<PGCSLinDynArmSDF> runner(num_exp, config_file);

    runner.read_config_file();

    runner.run();

}