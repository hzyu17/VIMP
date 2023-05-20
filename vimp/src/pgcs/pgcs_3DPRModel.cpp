/**
 * @file pgcs_3DPRModel.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief pgcs with plannar obstacles and point robot, 
 * using Robot Model which has a vector of balls to check collisions.
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "covariance_steering/PGCSLinDynPRModelSDF.h"
#include "helpers/experiment_runner_3D.h"

using namespace Eigen;
using namespace vimp;

int main(){
    int num_exp = 4;
    
    std::string config_file{"/home/hongzhe/git/VIMP/vimp/configs/pgcs/pR3D_map2.xml"};
    ExperimentRunner3D<PGCSLinDynPRModelSDF> runner(num_exp, config_file);

    runner.read_config_file();
    runner.run();
}