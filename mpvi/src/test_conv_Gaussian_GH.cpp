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

#include "../include/OptimizerGH.h"
#include "../include/OptimizerFactorizedGH.h"
#include <gtsam/base/Matrix.h>
#include <iostream>
#include <random>

using namespace gtsam;
using namespace std;
using namespace GaussianSampler;
using namespace MPVI;

typedef SparseMatrix<double> SpMatrix;
typedef Triplet<double> T;

inline double cost_function(const VectorXd& sample, const Gaussian_distribution& target_distr){
    return -target_distr.log_prob(sample);
}

using FactorizedOptimizer = VIMPOptimizerFactorizedGaussHermite<std::function<double(const VectorXd&, const Gaussian_distribution&)>, Gaussian_distribution, gtsam::Vector>;

void test_coupled(){
    const int ndim = 15;

    // construct block matrices Sigma_k, P_k, and factored cost functions
    vector<std::function<MatrixXd(const gtsam::Vector&, const Gaussian_distribution&)>> vec_cost_func;
    vector<Gaussian_distribution> vec_cost_class;
    vector<std::shared_ptr<FactorizedOptimizer>> vec_factorized_opt;

    vector<gtsam::Matrix> vec_Pks;
    for (int i=0; i<ndim-1; i++){
        // Pks
        MatrixXd Pk{MatrixXd::Zero(2, ndim)};
        Pk(0, i) = 1;
        Pk(1, i+1) = 1;
        vec_Pks.emplace_back(Pk);

        // known target posteria Gaussian
        gtsam::Vector mean_t(2);
        gtsam::Matrix precision_t(2, 2);
        precision_t = gtsam::Matrix::Identity(2, 2);
        precision_t(0, 1) = -0.74;
        precision_t(1, 0) = -0.74;
        mean_t = gtsam::Vector::Ones(2);

        Gaussian_distribution target_distr(mean_t, precision_t.inverse());

        /// try with unique pointer
        std::shared_ptr<FactorizedOptimizer> pFactOptimizer(new FactorizedOptimizer{2, cost_function, target_distr});
        vec_factorized_opt.emplace_back(pFactOptimizer);

    }

    VIMPOptimizerGH<FactorizedOptimizer> optimizer(vec_Pks, vec_factorized_opt);

    const int num_iter = 10;
    double step_size = 0.9;

    for (int i=0; i<num_iter; i++) {
        step_size = step_size / pow((i + 1), 1 / 3);
        optimizer.set_step_size(step_size, step_size);

        cout << "==== iteration " << i << " ====" << endl
             << "mean " << endl << optimizer.get_mean().format(CleanFmt) << endl
             << "precision matrix " << endl << optimizer.get_precision().format(CleanFmt) << endl;

        // get the derivatives from samples
         optimizer.step();

    }
}

int main(){
    test_coupled();

};