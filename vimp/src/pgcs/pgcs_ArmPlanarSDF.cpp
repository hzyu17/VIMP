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

#include "covariance_steering/PGCSLinDynArmPlanarSDF.h"
#include "helpers/experiment_runner.h"

using namespace Eigen;
using namespace vimp;

int main(){
    int nx=4, nu=2, num_exp=2;
    std::string config_file{"/home/hongzhe/git/VIMP/vimp/configs/pgcs/planar_2link_arm_map2.xml"};
    PGCSRunner<PGCSLinArmPlanarSDF> runner(nx, nu, num_exp, config_file);

    runner.read_config_file();

    runner.run();

}