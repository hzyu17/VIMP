/**
 * @file gvi_FrankaArm_spgh_cuda.cpp
 * @author Zinuo Chang (zchang40@gatech.edu)
 * @brief Testing GPU computation of obstacle collision costs on a 3D robot arm
 * @version 0.1
 * @date 2025-05-01
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "helpers/ExperimentRunner.h"
#include "instances_cuda/gvimp/GVIMPFrankaSDF.h"

using namespace vimp;

int main(int argc, char* argv[]){
    std::string source_root{XSTRING(SOURCE_ROOT)};
    int nx = 14, nu = 7, num_exp = 1;
    GVIMPParams params;
    // no experiment argument, run the default scripts
    if (argc == 1){
        std::string config_file{source_root+"/configs/vimp/sparse_gh/franka.xml"};
        GVIMPRunner7D<GVIMPFrankaSDF> runner(num_exp, config_file);
        runner.read_config(params);
        runner.run();
        return 0;
    }
    else if (argc == 2){
        num_exp = 1;
        std::string config_relative = static_cast<std::string>(argv[1]);
        std::string config_file{source_root+"/"+config_relative};
        GVIMPRunner7D<GVIMPFrankaSDF> runner(num_exp, config_file);
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
        return 0;
    }
    else if (argc == 18) {
        std::string config_file{source_root+"/configs/vimp/wam_map1_new.xml"};
        GVIMPRunner7D<GVIMPFrankaSDF> runner(num_exp, config_file);

        int i_exp = std::stoi(argv[1]);
        double total_time = std::stof(argv[2]);
        int n_states = std::stoi(argv[3]);
        double coeff_Qc = std::stof(argv[4]);
        double sig_obs = std::stof(argv[5]);
        double eps_sdf = std::stof(argv[6]);
        double radius = std::stof(argv[7]);
        double step_size = std::stof(argv[8]);
        double init_precision_factor = std::stof(argv[9]);
        double boundary_penalties = std::stof(argv[10]);
        double temperature = std::stof(argv[11]);
        double high_temperature = std::stof(argv[12]);
        int low_temp_iterations = std::stoi(argv[13]);
        double stop_err = std::stof(argv[14]);
        int num_iter = std::stoi(argv[15]);
        int max_n_backtracking = std::stoi(argv[16]);
        std::string sdf_file{argv[17]};
        int GH_deg = 3;
        int nx = 14, nu = 7;

        params = GVIMPParams(nx, nu, total_time, n_states, coeff_Qc, GH_deg, sig_obs, 
                            eps_sdf, radius, step_size, num_iter, init_precision_factor, 
                            boundary_penalties, temperature, high_temperature, low_temp_iterations, 
                            stop_err, max_n_backtracking, "map_bookshelf", sdf_file);

        double time_elapsed = 0.0;
        for (int i=0; i<5; i++){
            std::cout << "repeated experiment: " << i << std::endl;
            time_elapsed += runner.run_one_exp(i_exp, params, true);
        }
        
        std::cout << "time elapsed: " << time_elapsed << std::endl;

        return 0;
    }
    else{
        std::runtime_error("Wrong number of arguments!");
    }
    return 0;
}