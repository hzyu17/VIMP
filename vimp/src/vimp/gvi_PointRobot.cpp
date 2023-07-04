/**
 * @file conv_prior_posvel_col_pR.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the convergence of the algorithm with prior (pos + vel) + collision cost 
 * only on supported states, for a planar robot.
 * @version 0.1
 * @date 2022-07-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "../../helpers/ExperimentRunner.h"
#include "../../instances/GVIMPPlanarPRSDF.h"
using namespace vimp;

int main(){
    std::string config_file{"/home/hongzhe/git/VIMP/vimp/configs/vimp/planar_pR_map2_new.xml"};
    int nx = 4, nu = 2, num_exp = 4;
    GVIMPRunner<GVIMPPlanarPRSDF> runner(nx, nu, num_exp, config_file);
    GVIMPExperimentParams params;
    runner.read_config_file(params);
    runner.run();
    
    return 0;
}