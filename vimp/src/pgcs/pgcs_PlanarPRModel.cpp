/**
 * @file pgcs_PRModel.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief pgcs with plannar obstacles and point robot, 
 * using Robot Model which has a vector of balls to check collisions.
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "covariance_steering/PGCSLinDynPRModelPlanarSDF.h"
#include "helpers/experiment_runner.h"

using namespace Eigen;
using namespace vimp;

int main(){
    int nx=4, nu=2;
    int num_exp = 4;
    std::string config_file{"/home/hongzhe/git/VIMP/vimp/configs/pgcs/planar_pR_map2.xml"};
    ExperimentRunner<PGCSLinDynPRModelPlanarSDF> runner(nx, nu, num_exp, config_file);

    runner.read_config_file();

    runner.run();
}