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

#include "../include/optimizer/OptimizerGH-impl.h"
#include "../include/optimizer/OptimizerFactorizedGH.h"
#include <iostream>
#include <random>

using namespace std;
using namespace GaussianSampler;
using namespace vimp;

typedef SparseMatrix<double> SpMatrix;
typedef Triplet<double> T;

inline double cost_function(const VectorXd& sample, const Gaussian_distribution& target_distr){
    return -target_distr.log_prob(sample);
}

using FactorizedOptimizer = VIMPOptimizerFactorizedGaussHermite<std::function<double(const VectorXd&, const Gaussian_distribution&)>, Gaussian_distribution>;

void test_coupled(){
    const int ndim = 15;

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
        precision_t(0, 1) = -0.74;
        precision_t(1, 0) = -0.74;
        mean_t = VectorXd::Ones(2);

        Gaussian_distribution target_distr(mean_t, precision_t.inverse());

        /// try with unique pointer
        FactorizedOptimizer::shared_ptr pFactOptimizer(new FactorizedOptimizer{2, cost_function, target_distr, Pk});
        vec_factorized_opt.emplace_back(pFactOptimizer);

    }

    VIMPOptimizerGH<FactorizedOptimizer> optimizer(vec_factorized_opt);

    const int num_iter = 10;

    optimizer.set_niterations(num_iter);
    optimizer.optimize();

    cout << "mean " << endl << optimizer.mean() << endl;
    cout << "joint precision matrix " << endl << optimizer.precision() << endl;
    
}

int main(){
    test_coupled();

};