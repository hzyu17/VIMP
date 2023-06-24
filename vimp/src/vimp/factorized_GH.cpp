/**
 * Test the factorized optimizer convergence with GH integrator
 * Author: Hongzhe Yu
 * Date: 05/11/22
 * */

#include <iostream>
#include <random>
#include <vimp/helpers/SparseInverseMatrix.h>
#include <vimp/helpers/MVGsampler.h>
#include <vimp/gvimp/GVIFactorizedGH.h>

using namespace std;
using namespace vimp;

typedef SparseMatrix<double> SpMatrix;
typedef Triplet<double> T;

inline double phi(const VectorXd& sample, const vimp::Gaussian_distribution& gaussian){
    
    return -gaussian.log_prob(sample);
}


int main(){
    const int ndim = 2;

    // known target posteria Gaussian
    VectorXd mean_t(2);
    MatrixXd precision_t(2, 2);
    precision_t = MatrixXd::Identity(2, 2);
    precision_t(0, 1) = -0.74;
    precision_t(1, 0) = -0.74;
    mean_t = VectorXd::Ones(2);
    Gaussian_distribution gaussian(mean_t, precision_t.inverse());

    using Function = std::function<double(const VectorXd&, const Gaussian_distribution&)>;
    using Optimizer = VIMPOptimizerFactorizedGaussHermite<Function, Gaussian_distribution>;
    /// optimizer 
    Optimizer::shared_ptr p_optimizer(new Optimizer{ndim, phi, gaussian, MatrixXd{MatrixXd::Identity(ndim, ndim)}});

    int n_iter = 10;
    double step_size = 0.9;
    p_optimizer->set_step_size(step_size, step_size);

    /// Main iteration code block
    cout << "=== start iteration ===" << endl;
    VectorXd l_mean = p_optimizer->mean();
    MatrixXd l_precision = p_optimizer->precision();

    for (int i=0; i<n_iter; i++){
        step_size = step_size / pow((i+1), 1/3);
        p_optimizer->set_step_size(step_size, step_size);

        bool decrease = p_optimizer->step();

        cout << "==== iteration " << i << " ====" << endl
             << "mean " << endl << p_optimizer->mean().format(CleanFmt) << endl
             << "precision matrix " << endl <<p_optimizer->precision().format(CleanFmt) << endl;

        if (not decrease){
            cout << "end of iteration " << endl;
            break;
        }

    }
    l_precision = p_optimizer->precision();
    l_mean = p_optimizer->mean();
    cout << "==== result ====" << endl
         << "mean " << endl << p_optimizer->mean().format(CleanFmt) << endl
         << "precision matrix " << endl << p_optimizer->precision().format(CleanFmt) << endl;
};
