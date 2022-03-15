//
// Created by hongzhe on 3/6/22.
//

// Test the convergence of the natural gradient discent algorithm using a known posteria

#include "../include/Optimizer.h"
#include <gtsam/base/Matrix.h>
#include <iostream>
#include <random>

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

    // known posteria Gaussian
    gtsam::Vector mean_t(ndim);
    gtsam::Matrix precision_t(ndim, ndim);
    precision_t = gtsam::Matrix::Zero(ndim, ndim);
    mean_t = gtsam::Vector::Ones(ndim);

    const int nnz = 3*ndim-2; //number of nonzeros in the tri-diag precision matrix
    precision_t(0, 0) = 1;
    precision_t(0, 1) = 0.4;
    for (int i=1; i<ndim-1; i++){
        precision_t(i, i-1) = 0.4;
        precision_t(i, i) = 1.0;
        precision_t(i, i+1) = 0.4;
    }
    precision_t(ndim-1, ndim-1) = 1;
    precision_t(ndim-1, ndim-2) = 0.4;
    cout << "precision_t" << endl << precision_t << endl;
    Gaussian_distribution target_distr(mean_t, precision_t);

    // construct block matrices Sigma_k, P_k
    vector<gtsam::Matrix> vec_Pks;
    for (int i=0; i<ndim-1; i++){
        gtsam::Matrix Pk{gtsam::Matrix::Zero(2, ndim)};
        Pk(0, i) = 1;
        Pk(1, i+1) = 1;
        cout << "i th mean " << endl << Pk * mean_t << endl;
        cout << "i the precision matrix " << endl << Pk * precision_t * Pk.transpose() << endl;
        vec_Pks.emplace_back(Pk);
    }

    VariationalIferenceMPOptimizer<std::function<double(const gtsam::Vector&, const Gaussian_distribution&)>, Gaussian_distribution, VectorXd> optimizer(ndim,
            target_cost, target_distr, vec_Pks);

    const int num_iter = 10;
    for (int i=0; i<num_iter; i++){
        optimizer.step();
    }

};