#include "helpers/ExperimentRunner.h"
#include "instances/gvimp/GVIMPWAMArm.h"
#include "GaussianVI/ngd/NGD-GH.h"

using namespace vimp;

int main(int argc, char* argv[]){

    int n_repeat_exp = 50;

    if (argc == 2){
        n_repeat_exp = std::stoi(argv[1]);
    }

    // GVI-MP 7D Arm Robot
    int nx = 14, nu = 7, num_exp = 2;
    GVIMPParams params;

    std::string config_file{source_root+"/configs/vimp/sparse_gh/wam_map1_new.xml"};
    GVIMPRunner7D<GVIMPWAMArm> runner(num_exp, config_file);
    runner.read_config(params);

    for (int i_exp=1; i_exp<3; i_exp++){
        double time_elapsed = 0.0;
        for (int i=0; i<n_repeat_exp; i++){
            std::cout << "repeated experiment: " << i << std::endl;
            time_elapsed += runner.run_one_exp(i_exp, params, false);
        }
        
        std::cout << "time elapsed: " << time_elapsed / n_repeat_exp << " for experiment " << i_exp << std::endl;
    }
        
    return 0;
}