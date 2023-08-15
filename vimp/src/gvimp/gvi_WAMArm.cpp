/**
 * @file gvi_WAMArm.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Experiments for a 7-DOF WAM arm robot.
 * @version 0.1
 * @date 2023-08-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "helpers/ExperimentRunner.h"
#include "instances/gvimp/GVIMPWAMArm.h"
using namespace vimp;


int main(){
    std::string source_root{XSTRING(SOURCE_ROOT)};
    int nx = 14, nu = 7, num_exp = 1;
    GVIMPParams params;
    // no experiment argument, run the default scripts
    std::string config_file{source_root+"/configs/vimp/wam_map1_new.xml"};
    GVIMPRunner7D<GVIMPWAMArm> runner(num_exp, config_file);
    runner.read_config(params);
    runner.run();

    return 0;
}