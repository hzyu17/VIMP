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

using namespace GaussianSampler;
using namespace std;
using namespace SparseInverse;
typedef Triplet<double> T;

// template function and classes to calculate the costs
template <typename Function, typename costClass, typename... Args>
class VariationalIferenceMPOptimizer{
public:
    VariationalIferenceMPOptimizer(const int& dimension, Function _function,
                                   const costClass& _cost_class, const vector<MatrixXd>& _vec_Pks):
                                   dim{dimension},
                                   num_sub_vars{_vec_Pks.size()},
                                   precision_{MatrixXd::Identity(dim, dim).sparseView()},
                                   cost_function_{_function},
                                   cost_class_{_cost_class},
                                   vec_Pks_{_vec_Pks},
                                   mu_{VectorXd::Zero(dimension)},
                                   inverser_{MatrixXd::Identity(dim, dim)}{

        /// initialize sparse precision matrix and the inverse helper
//        precision_.reserve(3*dimension-2);
//        d_precision.reserve(3*dimension-2);

//        inverser_.update_sparse_precision(precision_);

        /// initialize the factors
        for (int i=0; i<num_sub_vars; i++){
            VariationalIferenceMPOptimizerTwoByTwo<Function, costClass, Args...> optimizer_k{2, _function, _cost_class};
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
    vector<VariationalIferenceMPOptimizerTwoByTwo<Function, costClass, Args...>> vec_factor_optimizers_;

    // cost functional. Input: samples vector; Output: cost
    Function cost_function_;
    costClass cost_class_;
    const vector<MatrixXd> vec_Pks_;

    // Sparse matrix inverse helper
//    sparse_inverser inverser_;
    dense_inverser inverser_;

public:

    bool step(){
        MatrixXd Sigma{inverser_.inverse(precision_)};

        MatrixXd new_precision(dim, dim);
        VectorXd new_mu{VectorXd::Zero(dim)};

        auto optimizer_k = vec_factor_optimizers_[0];
        // use the sparse inverse algorithm
        optimizer_k.updateMean();
        optimizer_k.updateCovarianceMatrix(Sigma);

        optimizer_k.step();
//        for (int k=0; k<num_sub_vars; k++){
//            MatrixXd Pk = vec_Pks_[k];
//
////            VectorXd sub_mu_k = Pk * mu_;
////            MatrixXd sub_Sigma_k = Pk * Sigma * Pk.transpose();
//
//            VectorXd sub_mu_k{mu_};
//            MatrixXd sub_Sigma_k{Sigma};
//
////            cout << "sigma k" << endl << sub_Sigma_k << endl;
//
//            auto optimizer_k = vec_factor_optimizers_[k];
//            optimizer_k.updateMean(sub_mu_k);
//            optimizer_k.updateCovarianceMatrix(sub_Sigma_k);
//
//            optimizer_k.step();
//
////            cout << "new_precision " << endl << new_precision << endl;
////            cout << "Pk.transpose() * optimizer_k.get_precision() * Pk " << endl << Pk.transpose() * optimizer_k.get_precision() * Pk << endl;
//
////            new_precision = new_precision + Pk.transpose() * optimizer_k.get_precision() * Pk;
////            new_mu = new_mu + Pk.transpose() * optimizer_k.get_mean();
////            new_precision = optimizer_k.get_precision().sparseView();
//            new_precision = optimizer_k.get_precision();
//            new_mu = optimizer_k.get_mean();
//
////            cout << "new_precision " << endl << new_precision << endl;
//
//        }

//        precision_ = new_precision;
//        mu_ = new_mu;

        precision_ = optimizer_k.get_precision();
        mu_ = optimizer_k.get_mean();

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