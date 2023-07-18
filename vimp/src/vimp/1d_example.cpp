/**
 * @file conv_1d_example.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Recover the results of 1d estimation example in the paper.
 * @version 0.1
 * @date 2022-07-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "gvimp/GVIFactorizedSimpleGH.h"
#include "gvimp/GVI-GH.h"

using namespace Eigen;
using namespace vimp;

double cost_function(const VectorXd& vec_x){
    double x = vec_x(0);
    double mu_p = 20, f = 400, b = 0.1, sig_r_sq = 0.09;
    double sig_p_sq = 9;

    // y should be sampled. for single trial just give it a value.
    double y = f*b/mu_p - 0.8;

    return ((x - mu_p)*(x - mu_p) / sig_p_sq / 2 + (y - f*b/x)*(y - f*b/x) / sig_r_sq / 2); 

}


int main(){

    EigenWrapper _ei;
    typedef std::function<double(const VectorXd&)> Function;
    typedef GVIFactorizedSimpleGH<Function> OptFact;
    
    int dim_state = 1;
    int num_states = 1;
    int dim_factor = 1;

    std::vector<std::shared_ptr<OptFact>> vec_opt_fact;
    std::shared_ptr<OptFact> p_opt_fac{new OptFact(dim_factor, dim_state, num_states, 0, cost_function)};
    VectorXd init_mu{VectorXd::Constant(1, 20.0)};
    SpMat init_prec(1, 1);
    init_prec.setZero();
    init_prec.coeffRef(0, 0) = 1.0/9.0;

    vec_opt_fact.emplace_back(p_opt_fac);
    GVIGH<OptFact> opt{vec_opt_fact, dim_state, num_states};

    std::string source_root{XSTRING(SOURCE_ROOT)};
    std::string prefix{source_root+"/data/vimp/1d/"};

    opt.update_file_names(prefix);
    std::string costmap_file{source_root+"/data/vimp/1d/costmap.csv"};
    opt.save_costmap(costmap_file);

    opt.set_GH_degree(20);
    opt.set_initial_values(init_mu, init_prec);
    opt.set_step_size_base(0.75);
    cout << "opt.mu " << endl << opt.mean() << endl;
    opt.optimize();
    
    return 0;
}