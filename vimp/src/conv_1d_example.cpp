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


#include "../optimizer/OptimizerFactorizedSimpleGH.h"
#include "../optimizer/OptimizerGH.h"

using namespace Eigen;
using namespace vimp;

double cost_function(const VectorXd& vec_x){
    double x = vec_x(0);
    double mu_p = 20, f = 400, b = 0.1, sig_r_sq = 0.09;
    double sig_p_sq = 9;

    // y should be sampled. for single trial just give it a value.
    double y = f*b/mu_p - 1;

    return (x - mu_p)*(x - mu_p) / sig_p_sq / 2 + (y - f*b/x)*(y - f*b/x) / sig_r_sq / 2; 

}


int main(){

    typedef std::function<double(const VectorXd&)> Function;
    typedef VIMPOptimizerFactorizedSimpleGH<Function> OptFact;
    
    MatrixXd Pk{MatrixXd::Constant(1, 1, 1)}; 

    std::vector<std::shared_ptr<OptFact>> vec_opt_fact;

    std::shared_ptr<OptFact> p_opt_fac{new OptFact(1, cost_function, Pk)};
    
    VectorXd init_mu{VectorXd::Constant(1, 20.0)};
    MatrixXd init_prec(MatrixXd::Constant(1, 1, 1.0/9.0));

    vec_opt_fact.emplace_back(p_opt_fac);

    VIMPOptimizerGH<OptFact> opt{vec_opt_fact, 5};

    opt.save_costmap();
    opt.set_initial_values(init_mu, init_prec);
    cout << "opt.mu " << endl << opt.mean() << endl;
    opt.optimize();
    
    return 0;
}