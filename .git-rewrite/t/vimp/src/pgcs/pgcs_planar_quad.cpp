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

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "pgcsmp/PGCSPlanarQuadSDF.h"
#include "helpers/ExperimentRunner.h"

using namespace Eigen;
using namespace vimp;

int main(){

    int nx=6, nu=2;
    int num_exp = 1;
    std::string source_root{XSTRING(SOURCE_ROOT)};
    std::string config_file{source_root+"/configs/pgcs/planar_quad_map2.xml"};
    PGCSRunnerNonLinear<PGCSPlanarQuadSDF, PlanarQuadDynamics> runner(nx, nu, num_exp, config_file);

    runner.run();

    return 0;
    
}