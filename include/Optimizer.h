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
#include <boost/scoped_ptr.hpp>

using namespace GaussianSampler;
using namespace std;
using namespace SparseInverse;
typedef Triplet<double> T;

template <typename Function, typename costClass, typename... Args>

// template function and classes to calculate the costs
class VariationalIferenceMPOptimizer{
    using FactorizedOptimizer = VariationalIferenceMPOptimizerTwoByTwo<Function, costClass, Args...>;

public:
    VariationalIferenceMPOptimizer(const int& dimension, const vector<Function>& _vec_function,
                                   const vector<costClass>& _vec_cost_class, const vector<MatrixXd>& _vec_Pks):
                                   dim{dimension},
                                   num_sub_vars{_vec_Pks.size()},
                                   precision_{MatrixXd::Identity(dim, dim).sparseView()},
                                   vec_cost_function_{_vec_function},
                                   vec_cost_class_{_vec_cost_class},
                                   vec_Pks_{_vec_Pks},
                                   mu_{VectorXd::Zero(dimension)},
                                   inverser_{MatrixXd::Identity(dim, dim)}{

        /// initialize sparse precision matrix and the inverse helper
//        precision_.reserve(3*dimension-2);
//        d_precision.reserve(3*dimension-2);

//        inverser_.update_sparse_precision(precision_);

        /// initialize the factors
        for (int i=0; i<num_sub_vars; i++){
            FactorizedOptimizer optimizer_k{2, _function, _cost_class};
            vec_factor_optimizers_.emplace_back(optimizer_k);
        }
    }
protected:
    // optimization variables
    int dim;
    int num_sub_vars;

    gtsam::Vector mu_;
    MatrixXd precision_;

    // sampler
    vector<FactorizedOptimizer> vec_factor_optimizers_;

    // cost functional. Input: samples vector; Output: cost
    const vector<Function> vec_cost_function_;
    const vector<costClass> vec_cost_class_;
    const vector<MatrixXd> vec_Pks_;

    // Sparse matrix inverse helper
//    sparse_inverser inverser_;
    dense_inverser inverser_;

public:

    bool step(){
        MatrixXd Sigma{inverser_.inverse(precision_)};

        MatrixXd new_precision{MatrixXd::Zero(dim, dim)};
        VectorXd new_mu{VectorXd::Zero(dim)};

        for (int k=0; k<num_sub_vars; k++){
            MatrixXd Pk = vec_Pks_[k];

            VectorXd sub_mu_k{Pk * mu_};
            MatrixXd sub_Sigma_k{Pk * Sigma * Pk.transpose()};

            auto &optimizer_k = vec_factor_optimizers_[k];
            optimizer_k.updateMean(sub_mu_k);
            optimizer_k.updateCovarianceMatrix(sub_Sigma_k);

            optimizer_k.step();

            cout << "new_precision " << endl << new_precision << endl;

            new_precision = new_precision + Pk.transpose() * optimizer_k.get_precision() * Pk;
            new_mu = new_mu + Pk.transpose() * optimizer_k.get_mean();

            cout << "new_precision " << endl << new_precision << endl;
        }

        precision_ = new_precision;
        mu_ = new_mu;

        cout << "mu_ " << endl << mu_ << endl;
        cout << "new precision " << endl << precision_ << endl;

        return true;
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

    void set_step_size(double ss_mean, double ss_precision){
        for (auto & k_optimizer:vec_factor_optimizers_)
            k_optimizer.set_step_size(ss_mean, ss_precision);
    }

};
