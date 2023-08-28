/**
 * @file pgcs_pR_RobotModel.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief pgcs with plannar obstacles and arm robot, 
 * using Robot Model which has a vector of balls to check collisions.
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "pgcsmp/PGCSLinDynArmSDF.h"
#include "helpers/ExperimentRunner.h"

using namespace vimp;

int main(int argc, char* argv[]){
    
    int nx = 14, nu=7;

    int num_exp = 3;
    std::string source_root{XSTRING(SOURCE_ROOT)};
    std::string config_file{source_root+"/configs/pgcs/wam_arm.xml"};
    PGCSRunner7D<PGCSLinDynArmSDF> runner(num_exp, config_file);
    // no experiment argument, run the default scripts
    if (argc == 1){
        runner.run();
        return 0;
    }
    // arguments: i_exp, params:(i_exp, eps, eps_sdf, radius, total_time, nt, sig0, sigT, step_size, stop_err, max_iter, sig_obs, backtrack_ratio, backtrack_iterations)
    else if (argc == 17){
        std::cout << "========= start running one exp GVI-MP =========" << std::endl;
        int i_exp = std::stoi(argv[1]);
        double eps = std::stof(argv[2]);
        double eps_sdf = std::stof(argv[3]);
        double radius = std::stof(argv[4]);
        double total_time = std::stof(argv[5]);
        int nt = std::stof(argv[6]);
        double sig0 = std::stof(argv[7]);
        double sigT = std::stof(argv[8]);
        double step_size = std::stof(argv[9]);
        double stop_err = std::stof(argv[10]);
        int max_iter = std::stof(argv[11]);
        double sig_obs = std::stof(argv[12]);
        double backtrack_ratio = std::stof(argv[13]);
        int backtrack_iterations = std::stoi(argv[14]);
        std::string map_name{argv[15]};
        std::string sdf_file{argv[16]};

        PGCSParams params(nx, nu, eps_sdf, radius, eps, total_time, 
                          nt, sig0, sigT, step_size, stop_err, sig_obs, 
                          max_iter, backtrack_ratio, backtrack_iterations, map_name, sdf_file);
        runner.run_one_exp(i_exp, params, true);
        return 0;
    }
    else{
        std::cout << "Wrong number of arguments!" << std::endl;
        return 0;
    }
    

}