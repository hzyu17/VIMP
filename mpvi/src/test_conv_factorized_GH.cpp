/**
 * Test the factorized optimizer convergence with GH integrator
 * Author: Hongzhe Yu
 * Date: 05/11/22
 * */

#include "../include/OptimizerFactorizedGH.h"
#include <gtsam/base/Matrix.h>
#include <iostream>
#include <random>
#include "../include/SparseInverseMatrix.h"

using namespace GaussianSampler;
using namespace gtsam;
using namespace std;
using namespace MPVI;

typedef SparseMatrix<double> SpMatrix;
typedef Triplet<double> T;

inline double target_cost(const gtsam::Vector& sample, const Gaussian_distribution& target_distr){
    
    return -target_distr.log_prob(sample);
}


int main(){
    const int ndim = 2;

    // known target posteria Gaussian
    gtsam::Vector mean_t(2);
    gtsam::Matrix precision_t(2, 2);
    precision_t = gtsam::Matrix::Identity(2, 2);
    precision_t(0, 1) = -0.74;
    precision_t(1, 0) = -0.74;
    mean_t = gtsam::Vector::Ones(2);
    Gaussian_distribution target_distr(mean_t, precision_t.inverse());

    /// optimizer 
    VIMPOptimizerFactorizedGaussHermite<std::function<double(const gtsam::Vector&, const Gaussian_distribution&)>,
                                           Gaussian_distribution, gtsam::Vector> optimizer(ndim, target_cost, target_distr, MatrixXd::Identity(ndim, ndim));

    /// inverser
    using namespace SparseInverse;
    dense_inverser inverser_(MatrixXd::Identity(ndim, ndim));

    int n_iter = 10;
    double step_size = 0.9;
    optimizer.set_step_size(step_size, step_size);

    /// Main iteration code block
    cout << "=== start iteration ===" << endl;
    gtsam::Vector l_mean = optimizer.get_mean();
    gtsam::Matrix l_precision = optimizer.get_precision();

    for (int i=0; i<n_iter; i++){
        step_size = step_size / pow((i+1), 1/3);
        optimizer.set_step_size(step_size, step_size);

        bool decrease = optimizer.step();

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

