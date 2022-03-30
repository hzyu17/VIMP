//
// Created by hongzhe on 3/6/22.
//

// Test the convergence of algorithm using a known Gaussian posterior

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
                   const Gaussian_distribution& cost_distribution){
    return -cost_distribution.log_prob(sample);
}

void test_coupled(){
    const int ndim = 15;

    // known posteria Gaussian
    gtsam::Vector mean_t(ndim);
    gtsam::Matrix precision_t(ndim, ndim);
    precision_t = gtsam::Matrix::Zero(ndim, ndim);
    mean_t = gtsam::Vector::Ones(ndim);

    const int nnz = 3*ndim-2; //number of nonzeros in the tri-diag precision matrix
    precision_t(0, 0) = 1;
    precision_t(0, 1) = 0.24;
    for (int i=1; i<ndim-1; i++){
        precision_t(i, i-1) = 0.24;
        precision_t(i, i) = 1.0;
        precision_t(i, i+1) = 0.24;
    }
    precision_t(ndim-1, ndim-1) = 1;
    precision_t(ndim-1, ndim-2) = 0.24;
    MatrixXd covariance_t{precision_t.inverse()};
    cout << "precision target" << endl << precision_t << endl;

    // construct block matrices Sigma_k, P_k, and factored cost functions
    vector<std::function<double(const gtsam::Vector&, const Gaussian_distribution&)>> vec_target_costs;
    vector<Gaussian_distribution> vec_target_classes;
    vector<Gaussian_distribution> vec_cost_class;

    cout << "whole precision " << endl << precision_t << endl;
    vector<gtsam::Matrix> vec_Pks;
    for (int i=0; i<ndim-1; i++){
        MatrixXd Pk{MatrixXd::Zero(2, ndim)};
        Pk(0, i) = 1;
        Pk(1, i+1) = 1;
        vec_Pks.emplace_back(Pk);

        vec_target_costs.emplace_back(target_cost);

        vec_target_classes.emplace_back(Gaussian_distribution{Pk*mean_t, Pk*covariance_t*Pk.transpose()});

        MatrixXd cost_precision;
        cost_precision = Pk*precision_t*Pk.transpose();
        if (i==0) {cost_precision(1, 1) = cost_precision.eval()(1, 1) / 2.0;}
        if (i==ndim-2) {cost_precision(0, 0) = cost_precision.eval()(0, 0) / 2.0;}
        if (i > 0 && i < ndim-2) {
            cost_precision(0, 0) = cost_precision.eval()(0, 0) / 2.0;
            cost_precision(1, 1) = cost_precision.eval()(1, 1) / 2.0;
        }
        vec_cost_class.emplace_back(Gaussian_distribution{Pk*mean_t, cost_precision.inverse()});

        cout << "Pk" << endl << Pk << endl;
        cout << "cost_precision " << endl << cost_precision << endl;

    }

    VariationalInferenceMPOptimizer<std::function<double(const gtsam::Vector&, const Gaussian_distribution&)>,
            Gaussian_distribution,
            gtsam::Vector> optimizer(ndim, 2, vec_target_costs, vec_cost_class, vec_Pks);

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

        // test for a known Gaussian posterior
//        optimizer.step_closed_form();
    }
}

void test_decoupled(){
    const int ndim = 4;

    // known posteria Gaussian
    gtsam::Vector mean_t(ndim);
    gtsam::Matrix precision_t(ndim, ndim);
    precision_t = gtsam::Matrix::Zero(ndim, ndim);
    mean_t = gtsam::Vector::Ones(ndim);

    const int nnz = 3*ndim-2; //number of nonzeros in the tri-diag precision matrix
    precision_t(0, 0) = 1;
    precision_t(0, 1) = 0.24;
    for (int i=1; i<ndim-1; i++){
        precision_t(i, i-1) = 0.24;
        precision_t(i, i) = 1.0;
        precision_t(i, i+1) = 0.24;
    }
    precision_t(ndim-1, ndim-1) = 1;
    precision_t(ndim-1, ndim-2) = 0.24;
    MatrixXd covariance_t{precision_t.inverse()};
    cout << "precision target" << endl << precision_t << endl;

    // construct block matrices Sigma_k, P_k, and factored cost functions
    vector<std::function<double(const gtsam::Vector&, const Gaussian_distribution&)>> vec_target_costs;
    vector<Gaussian_distribution> vec_target_classes;
    vector<Gaussian_distribution> vec_cost_class;

    cout << "whole precision " << endl << precision_t << endl;
    vector<gtsam::Matrix> vec_Pks;
    MatrixXd P1{MatrixXd::Zero(2, ndim)};
    MatrixXd P2{MatrixXd::Zero(2, ndim)};
    P1(0, 0) = 1;
    P1(1, 1) = 1;
    P2(0, 2) = 1;
    P2(1, 3) = 1;

    cout << "P1 " << endl << P1 << endl;
    cout << "P2 " << endl << P2 << endl;

    vec_Pks.emplace_back(P1);
    vec_Pks.emplace_back(P2);

    for (int i=0; i<ndim-2; i++){

        MatrixXd& Pk = vec_Pks[i];
        vec_target_costs.emplace_back(target_cost);
        vec_target_costs.emplace_back(target_cost);

        vec_target_classes.emplace_back(Gaussian_distribution{Pk*mean_t, Pk*covariance_t*Pk.transpose()});

        MatrixXd cost_precision;
        cost_precision = Pk*precision_t*Pk.transpose();

        vec_cost_class.emplace_back(Gaussian_distribution{Pk*mean_t, cost_precision.inverse()});

    }

    VariationalInferenceMPOptimizer<std::function<double(const gtsam::Vector&, const Gaussian_distribution&)>,
            Gaussian_distribution,
            gtsam::Vector> optimizer(ndim, 2, vec_target_costs, vec_cost_class, vec_Pks);

    const int num_iter = 10;
    double step_size = 0.9;

    for (int i=0; i<num_iter; i++) {
        step_size = step_size / pow((i + 1), 1 / 6);
        optimizer.set_step_size(step_size, step_size);

        optimizer.step();

        if (i%5==0 && i>0){
            cout << "==== iteration " << i << " ====" << endl
                 << "mean " << endl << optimizer.get_mean().format(CleanFmt) << endl
                 << "precision matrix " << endl << optimizer.get_precision().format(CleanFmt) << endl;
        }
    }
}


int main(){

//    test_decoupled();
    test_coupled();

};