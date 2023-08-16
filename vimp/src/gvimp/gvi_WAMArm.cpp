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


int main(int argc, char* argv[]){
    std::string source_root{XSTRING(SOURCE_ROOT)};
    int nx = 14, nu = 7, num_exp = 1;
    GVIMPParams params;
    if (argc == 1){
        // no experiment argument, run the default scripts
        std::string config_file{source_root+"/configs/vimp/wam_map1_new.xml"};
        GVIMPRunner7D<GVIMPWAMArm> runner(num_exp, config_file);
        runner.read_config(params);
        runner.run();
    }
    else if (argc == 2){
        num_exp = 1;
        std::string config_relative = static_cast<std::string>(argv[1]);
        std::string config_file{source_root+"/"+config_relative};
        GVIMPRunner7D<GVIMPWAMArm> runner(num_exp, config_file);
        // Read configurations
        runner.read_config(params);
        for (int i=0; i<num_exp; i++){
            runner.read_boundary_conditions(config_file, i, params);

            // Collect results
            std::tuple<Eigen::VectorXd, SpMat> mean_precision;
            mean_precision = runner.run_one_exp_return(params);
            VectorXd mean = std::get<0>(mean_precision);
            SpMat precision = std::get<1>(mean_precision);
            
            EigenWrapper ei;
            ei.print_matrix(mean, "optimized mean");
            ei.print_matrix(precision, "optimized precision");

        }
    }
    else{
        std::runtime_error("Wrong number of arguments!");
    }
    return 0;

    return 0;
}