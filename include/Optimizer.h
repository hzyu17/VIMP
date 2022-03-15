//
// Created by hongzhe on 3/7/22.
//

#ifndef MPVI_OPTIMIZER_H
#define MPVI_OPTIMIZER_H

#endif //MPVI_OPTIMIZER_H

#include <gtsam/base/Matrix.h>
#include <iostream>
#include <random>
#include <utility>
#include "SparseInverseMatrix.h"
#include "OptimizerTwoByTwo.h"
#include <vector>

using namespace GaussianSampler;
using namespace std;
using namespace SparseInverse;

// template function and classes to calculate the costs
template <typename Function, typename costClass, typename... Args>
class VariationalIferenceMPOptimizer{
public:
    VariationalIferenceMPOptimizer(const int& dimension, Function _function,
                                   const costClass& _cost_class, const vector<MatrixXd>& _vec_Pks):
                                   dim{dimension},
                                   num_sub_vars{_vec_Pks.size()},
                                   precision_{SpMatrix(dim, dim)},
                                   d_precision{SpMatrix(dim, dim)},
                                   cost_function_{std::forward<Function>(_function)},
                                   cost_class_{_cost_class},
                                   vec_Pks_{_vec_Pks},
                                   mu_{VectorXd::Zero(dimension)},
                                   d_mu{VectorXd::Zero(dimension)},
                                   inverser_{precision_},
                                   vec_sub_precisions_{vector<MatrixXd>(num_sub_vars)},
                                   vec_sub_mus{vector<VectorXd>(num_sub_vars)}{

        /// initialize sparse precision matrix and the inverse  helper
        precision_.reserve(3*dimension-2);
        d_precision.reserve(3*dimension-2);
        inverser_.update_sparse_precision(precision_);

        /// initialize the factors
        for (int i=0; i<num_sub_vars; i++){
            VectorXd sub_mu_{VectorXd::Zero(2)};
            MatrixXd sub_precision_{MatrixXd::Identity(2, 2)};

            vec_sub_mus.emplace_back(sub_mu_);
            vec_sub_precisions_.emplace_back(sub_precision_);
            vec_factor_optimizers_.emplace_back(VariationalIferenceMPOptimizerTwoByTwo<Function, costClass>{2, _function, _cost_class});
        }
    }
protected:
    // optimization variables
    int dim;
    int num_sub_vars;
    int num_samples = 10000;
    gtsam::Vector mu_, d_mu;
    SpMatrix precision_, d_precision;

    vector<MatrixXd> vec_sub_precisions_;
    vector<VectorXd> vec_sub_mus;


    // step sizes
    double step_size_mu = 0.05;
    double step_size_Sigma = 0.05;

    // sampler
    vector<VariationalIferenceMPOptimizerTwoByTwo<Function, costClass>> vec_factor_optimizers_;

    // cost functional. Input: samples vector; Output: cost
    Function cost_function_;
    costClass cost_class_;
    const vector<gtsam::Matrix> vec_Pks_;

    // Sparse matrix inverse helper
    sparse_inverser inverser_;

public:
    auto cost_function(Args... args){
        return cost_function_(args..., cost_class_);
    }

    void set_step_size(double ss_mean, double ss_precision){
        step_size_mu = ss_mean;
        step_size_Sigma = ss_precision;
    }

    bool step(){
        MatrixXd Sigma = inverser_.inverse();
        for (int k=0; k<num_sub_vars; k++){
            MatrixXd Pk = vec_Pks_[k];

            cout << "Pk" << endl << Pk << endl;

            VectorXd sub_mu_k = Pk * mu_;
            MatrixXd sub_Sigma_k = Pk * Sigma * Pk.transpose();

            auto optimizer_k = vec_factor_optimizers_[k];
            optimizer_k.updateMean(sub_mu_k);
            optimizer_k.updateCovarianceMatrix(sub_Sigma_k);

            optimizer_k.step();

        }
    }

    gtsam::Vector get_mean(){
        return mu_;
    }

    gtsam::Matrix get_precision(){
        return precision_;
    }

    gtsam::Matrix get_covariance(){
        return inverser_.inverse();
    }

};