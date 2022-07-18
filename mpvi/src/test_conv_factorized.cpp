//
// Created by hongzhe on 3/7/22.
//

// Test the factorized optimizer convergence

#include "../include/optimizer/OptimizerFactorized.h"
#include <gtsam/base/Matrix.h>
#include <iostream>
#include <random>
#include "../include/optimizer/SparseInverseMatrix.h"

using namespace gtsam;
using namespace std;
using namespace GaussianSampler;

typedef SparseMatrix<double> SpMatrix;
typedef Triplet<double> T;

double target_cost(const gtsam::Vector& sample,
                   const Gaussian_distribution& target_gaussian){
    return -target_gaussian.log_prob(sample);
}


int main(){
    const int ndim = 2;

    // known target posteria Gaussian
    gtsam::Vector mean_t(ndim);
    gtsam::Matrix precision_t(ndim, ndim);
    precision_t = gtsam::Matrix::Identity(ndim, ndim);
    precision_t(0, 1) = -0.74;
    precision_t(1, 0) = -0.74;
    mean_t = gtsam::Vector::Ones(ndim);

    Gaussian_distribution target_distr(mean_t, precision_t.inverse());

    cout << "==== target distribution ====" << endl << "mean " << endl << target_distr.get_mean().format(CleanFmt) << endl
         << "precision matrix " << endl << target_distr.get_precision().format(CleanFmt) << endl << "covariance matrix " << endl
         << target_distr.get_covariance().format(CleanFmt) << endl;

    // optimizer
    VIMPOptimizerFactorized<std::function<double(const gtsam::Vector&, const Gaussian_distribution&)>,
                                           Gaussian_distribution, gtsam::Vector> optimizer(ndim, target_cost, target_distr);

    // inverser
    using namespace SparseInverse;
    dense_inverser inverser_(MatrixXd::Identity(ndim, ndim));

    int n_iter = 30;
    double step_size = 0.9;
    optimizer.set_step_size(step_size, step_size);

    cout << "=== start iteration ===" << endl;
    gtsam::Vector l_mean = optimizer.get_mean();
    gtsam::Matrix l_precision = optimizer.get_precision();

    for (int i=0; i<n_iter; i++){
        step_size = step_size / pow((i+1), 1/3);
        optimizer.set_step_size(step_size, step_size);

        bool decrease = optimizer.step();

        /// Update the sampler parameters
        // use direct inverse
//        optimizer.updateMean();
//        optimizer.updateCovarianceMatrix();

        // use the sparse inverse algorithm
        optimizer.updateSamplerMean();
        optimizer.updateSamplerCovarianceMatrix(inverser_.inverse(optimizer.get_precision()));

        cout << "==== iteration " << i << " ====" << endl
             << "mean " << endl << optimizer.get_mean().format(CleanFmt) << endl
             << "precision matrix " << endl <<optimizer.get_precision().format(CleanFmt) << endl;

        if (not decrease){
            cout << "end of iteration " << endl;
            break;
        }

    }
    l_precision = optimizer.get_precision();
    l_mean = optimizer.get_mean();
    cout << "==== result ====" << endl
         << "mean " << endl << optimizer.get_mean().format(CleanFmt) << endl
         << "precision matrix " << endl <<optimizer.get_precision().format(CleanFmt) << endl;
};

