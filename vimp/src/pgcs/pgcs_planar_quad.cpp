/**
 * @file pgcs_planar_quad.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief pgcs with planar quadrotor dynamics (plannar sdf).
 * @version 0.1
 * @date 2023-05-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "covariance_steering/PGCSPlanarQuadSDF.h"
#include "helpers/ExperimentRunner.h"

using namespace Eigen;
using namespace vimp;

int main(){

    int nx=6, nu=2;
    int num_exp = 1;
    std::string config_file{"/home/hzyu/git/VIMP/vimp/configs/pgcs/planar_quad_map2.xml"};
    PGCSRunnerNonLinear<PGCSPlanarQuadSDF> runner(nx, nu, num_exp, config_file);

    runner.run();

    return 0;
    
}