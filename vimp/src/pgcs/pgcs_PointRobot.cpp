/**
 * @file pgcs_PRModel.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief pgcs with plannar obstacles and point robot. 
 * We use runner class defined under helpers/ to run the experiments. 
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "pgcsmp/PGCSLinDynPRModelPlanarSDF.h"
#include "helpers/ExperimentRunner.h"
#include <chrono>

using namespace Eigen;
using namespace vimp;

int main(int argc, char* argv[]){
    int nx=4, nu=2;
    int num_exp = 4;
    std::string file_name = "planar_pR_map2_BRM_demo";

    // arguments: num_exp, file_name
    if (argc == 3){
        num_exp = std::stoi(argv[1]);
        file_name = static_cast<std::string>(argv[2]);
    }

    std::string source_root{XSTRING(SOURCE_ROOT)};
    std::string config_file{source_root+"/configs/pgcs/" + file_name + ".xml"};
    std::cout<<"config file loaded from: "<<config_file<<std::endl;

    PGCSRunner<PGCSLinDynPRModelPlanarSDF> runner(nx, nu, num_exp, config_file);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // no experiment argument, run the default scripts
    if (argc == 1 || argc == 3){
        std::cout << "run " << std::endl;
        runner.run();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Run Time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()/1e6 << "s" << std::endl;
        return 0;
    }
    // arguments: i_exp, params:(i_exp, eps, eps_sdf, speed, nt, sig0, sigT, eta, stop_err, max_iter, cost_sig)
    else if (argc == 16){

        int i_exp = std::stoi(argv[1]);
        double eps = std::stof(argv[2]);
        double eps_sdf = std::stof(argv[3]);
        double radius = std::stof(argv[4]);
        double speed = std::stof(argv[5]);
        int nt = std::stof(argv[6]);
        double sig0 = std::stof(argv[7]);
        double sigT = std::stof(argv[8]);
        double eta = std::stof(argv[9]);
        double stop_err = std::stof(argv[10]);
        int max_iter = std::stof(argv[11]);
        double sig_obs = std::stof(argv[12]);
        double backtrack_ratio = std::stof(argv[13]);
        int backtrack_iterations = std::stoi(argv[14]);
        std::string saving_prefix = static_cast<std::string>(argv[15]);

        PGCSParams params(nx, nu, eps_sdf, radius, eps, speed, 
                                   nt, sig0, sigT, eta, stop_err, sig_obs, 
                                   max_iter, backtrack_ratio, backtrack_iterations);
        params.set_saving_prefix(saving_prefix);
        runner.run_one_exp(i_exp, params);
        return 0;
    }
    else{
        std::runtime_error("Wrong number of arguments!");
    }

    
}