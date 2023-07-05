/**
 * @file pgcs_pR_RobotModel.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief pgcs with plannar obstacles and point robot, 
 * using Robot Model which has a vector of balls to check collisions.
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "covariance_steering/PGCSLinDynPRModelPlanarSDF.h"
#include "helpers/ExperimentRunner.h"


using namespace Eigen;
using namespace vimp;

int main(int argc, char* argv[]){

    std::string config_file{"/home/hongzhe/git/VIMP/vimp/configs/pgcs/planar_pR_longrange.xml"};
    int nx=4, nu=2;
    int num_exp = 4;

    PGCSRunner<PGCSLinDynPRModelPlanarSDF> runner(nx, nu, num_exp, config_file);

    // no experiment argument, run the default scripts
    if (argc == 1){
        runner.run();
        return 0;
    }
    // arguments: i_exp, params:(i_exp, eps, eps_sdf, speed, nt, sig0, sigT, eta, stop_err, max_iter, cost_sig)
    else if (argc == 15){

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

        PGCSExperimentParams params(nx, nu, eps_sdf, radius, eps, speed, 
                                   nt, sig0, sigT, eta, stop_err, sig_obs, 
                                   max_iter, backtrack_ratio, backtrack_iterations);
        runner.run_one_exp(i_exp, params);
        return 0;
    }
    else{
        std::runtime_error("Wrong number of arguments!");
    }

    // int nx=4, nu=2;
    // int num_exp = 4;
    // std::string config_file{"/home/hongzhe/git/VIMP/vimp/configs/pgcs/planar_pR_longrange.xml"};
    // PGCSRunner<PGCSLinDynPRModelPlanarSDF> runner(nx, nu, num_exp, config_file);

    // runner.read_config_file();

    // runner.run();

}