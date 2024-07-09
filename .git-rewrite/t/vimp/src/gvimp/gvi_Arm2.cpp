/**
 * @file prior_pos_vel_col_Arm.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Experiment for arm robot model
 * @version 0.1
 * @date 2022-08-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "helpers/ExperimentRunner.h"
#include "instances/gvimp/GVIMPArm2SDF.h"
using namespace vimp;


int main(){
    std::string source_root{XSTRING(SOURCE_ROOT)};
    int nx = 4, nu = 2, num_exp = 1;
    GVIMPParams params;
    // no experiment argument, run the default scripts
    std::string config_file{source_root+"/configs/vimp/planar_2link_arm_new.xml"};
    GVIMPRunner<GVIMPArm2SDF> runner(nx, nu, num_exp, config_file);
    runner.read_config(params);
    runner.run();

    return 0;
}