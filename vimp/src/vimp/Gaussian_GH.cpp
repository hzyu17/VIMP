/**
 * @file test_conv_GH.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-05-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */


/// Test the convergence of algorithm using a known Gaussian posterior.

#include "../../gvimp/GVI-GH.h"
#include "../../gvimp/GVIFactorizedGH.h"
#include "../../helpers/MVGsampler.h"
#include <iostream>
#include <random>

using namespace std;
using namespace vimp;

inline double cost_function(const VectorXd& sample, const vimp::Gaussian_distribution& target_distr){
    return -target_distr.log_prob(sample);
}

using FactorizedOptimizer = VIMPOptimizerFactorizedGaussHermite<std::function<double(const VectorXd&, const Gaussian_distribution&)>, Gaussian_distribution>;

void test_coupled(){
    const int ndim = 4;

    // construct block matrices Sigma_k, P_k, and factored cost functions
    // vector<std::function<MatrixXd(const VectorXd&, const Gaussian_distribution&)>> vec_cost_func;
    // vector<Gaussian_distribution> vec_cost_class;
    vector<FactorizedOptimizer::shared_ptr> vec_factorized_opt;

    // vector<MatrixXd> vec_Pks;
    for (int i=0; i<ndim-1; i++){
        // Pks
        MatrixXd Pk{MatrixXd::Zero(2, ndim)};
        Pk(0, i) = 1;
        Pk(1, i+1) = 1;

        // known target posteria Gaussian
        VectorXd mean_t(2);
        MatrixXd precision_t(2, 2);
        precision_t = MatrixXd::Identity(2, 2);
        precision_t(0, 1) = 0.25;
        precision_t(1, 0) = 0.25;
        mean_t = VectorXd::Ones(2);

        Gaussian_distribution target_distr(mean_t, precision_t.inverse());

        /// try with unique pointer
        FactorizedOptimizer::shared_ptr pFactOptimizer(new FactorizedOptimizer{2, cost_function, target_distr, Pk});
        vec_factorized_opt.emplace_back(pFactOptimizer);

    }

    VIMPOptimizerGH<FactorizedOptimizer> optimizer(vec_factorized_opt);

    const int num_iter = 20;

    optimizer.set_niterations(num_iter);

    // initial values
    VectorXd init_mean{VectorXd::Zero(ndim)};
    MatrixXd init_precision{MatrixXd::Identity(ndim, ndim)};
    optimizer.set_initial_values(init_mean, init_precision);
    optimizer.set_step_size(0.6);
    optimizer.optimize();

    cout << "mean " << endl << optimizer.mean() << endl;
    cout << "joint precision matrix " << endl << optimizer.precision() << endl;
    
}

int main(){
    test_coupled();

};