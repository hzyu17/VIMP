/**
 * @file test_onestep_arm2.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test one step of the GVI-MP algorithm for a known arm 2 experiment result. 
 * Any modifications of the algorithm should pass this test.
 * @version 0.1
 * @date 2023-08-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "helpers/ExperimentRunner.h"
#include "instances/gvimp/GVIMPPlanarPRSDF.h"
#include <gtest/gtest.h>

using namespace gpmp2;
using namespace Eigen;
using namespace vimp;
using namespace std;

#define STRING(x) #x
#define XSTRING(x) STRING(x)

using SDFPR = gpmp2::ObstaclePlanarSDFFactor<gpmp2::ArmModel>;
using GVIFactorizedPlanarSDFPR = GVIFactorizedPlanarSDF<gpmp2::ArmModel>;

std::string config_file{source_root+"/configs/vimp/planar_pR_map1_new.xml"};
int nx = 4, nu = 2, num_exp = 2;
GVIMPRunner<GVIMPPlanarPRSDF> runner(nx, nu, num_exp, config_file);
GVIMPParams params;