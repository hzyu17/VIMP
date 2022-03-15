//
// Created by hongzhe on 3/7/22.
//

#ifndef MPVI_OPTIMIZER_H
#define MPVI_OPTIMIZER_H

#endif //MPVI_OPTIMIZER_H

#include "SparseMatrixHelper.h"
#include "MVGsampler.h"
#include <gtsam/base/Matrix.h>
#include <iostream>
#include <random>
#include <utility>

using namespace GaussianSampler;
using namespace std;

IOFormat CleanFmt(4, 0, ", ", "\n");


template <typename Function, typename costClass, typename... Args>
class VariationalIferenceMPOptimizerTwoByTwo{
public:
    VariationalIferenceMPOptimizerTwoByTwo(const int& dimension, Function _function, const costClass& _cost_class):
    cost_function_{std::forward<Function>(_function)},
    cost_class_{_cost_class},
    dim{dimension},
    d_mu{gtsam::Vector::Zero(dimension)},
    mu_{gtsam::Vector::Zero(dimension)},
    precision_{gtsam::Matrix::Identity(dimension, dimension)},
    d_precision{gtsam::Matrix::Zero(dimension, dimension)},
    sampler_{normal_random_variable(mu_, precision_.inverse())}{}
protected:
    // optimization variables
    int dim;
    int num_samples = 20000;
    gtsam::Vector mu_, d_mu;
    gtsam::Matrix precision_, d_precision;

    gtsam::Vector Vdmu = gtsam::Vector::Zero(dim);
    gtsam::Matrix Vddmu = gtsam::Matrix::Zero(dim, dim);

    // step sizes
    double step_size_mu = 0.05;
    double step_size_Sigma = 0.05;

    // sampler
    normal_random_variable sampler_;

    // cost functional. Input: samples vector; Output: cost
    Function cost_function_;
    costClass cost_class_;

public:

    auto cost_function(Args... args){
        return cost_function_(args..., cost_class_);
    }

    void set_step_size(double ss_mean, double ss_precision){
        step_size_mu = ss_mean;
        step_size_Sigma = ss_precision;
    }

    bool updateCovarianceMatrix(const MatrixXd& new_cov){
        sampler_.updateCovariance(new_cov);
        return true;
    }

    bool updateMean(const VectorXd& new_mu){
        sampler_.updateMean(new_mu);
        return true;
    }

    bool step(){
        // Zero grad
        d_mu.setZero();
        d_precision.setZero();
        Vdmu.setZero();
        Vddmu.setZero();

        gtsam::Matrix samples = sampler_(num_samples);
        int B = 0;

        // see sample mean and covariance
        VectorXd mean_sample {samples.rowwise().sum() / num_samples};
        MatrixXd cov_samples{((samples.colwise() - mean_sample) * (samples.colwise() - mean_sample).transpose()) / (num_samples-1)};

        auto colwise = samples.colwise();
        double accum_phi = 0;
        std::for_each(colwise.begin(), colwise.end(), [&](auto const &sample) {
            double phi = cost_function(sample);

            accum_phi += phi;

            Vdmu = Vdmu.eval() + (sample - mu_) * phi;
            Vddmu = Vddmu.eval() + (sample - mu_) * (sample - mu_).transpose().eval() * phi;
        });

        Vdmu = precision_ * Vdmu.eval() / double(num_samples);
        Vddmu.triangularView<Upper>() = precision_ * Vddmu.selfadjointView<Upper>() * precision_;
        Vddmu.triangularView<StrictlyLower>() = Vddmu.triangularView<StrictlyUpper>().transpose();

        Vddmu = Vddmu.eval() / double(num_samples);

        double avg_phi = accum_phi / double(num_samples);

        gtsam::Matrix tmp{precision_ * avg_phi};

        Vddmu.triangularView<Upper>() = Vddmu - precision_ * avg_phi;
        Vddmu.triangularView<StrictlyLower>() = Vddmu.triangularView<StrictlyUpper>().transpose();

        d_precision = -precision_ + Vddmu;

        /// without backtracking
        precision_ = precision_ + step_size_Sigma * d_precision;
        d_mu = precision_.colPivHouseholderQr().solve(-Vdmu);

        mu_ = mu_ + step_size_mu * d_mu;

        // Update the sampler parameters
        updateMean(mu_);
        updateCovarianceMatrix(precision_.inverse());

        return true;

        /// backtracking
//        // last-step value cost function evaluation
//        float l_V = accum_phi / num_samples + log(precision_.determinant()) / 2.0;
//        cout << "l_V" << l_V << endl;
//        MatrixXd l_precision_{precision_};
//        VectorXd l_mu{mu_};
//        precision_.setZero();
//        mu_.setZero();
//        for (int i_ls=0; i_ls<10; i_ls++){
//            B = i_ls;
//
//            precision_ = l_precision_ + pow(step_size_Sigma, B) * d_precision;
//            d_mu = precision_.colPivHouseholderQr().solve(-Vdmu);
//
//            mu_ = l_mu + pow(step_size_mu, B) * d_mu;
//
//            // Update the sampler parameters
//            sampler_.updateMean(mu_);
//            sampler_.updatePrecisionMatrix(precision_);
//
//            // Evaluate the cost function
//            gtsam::Matrix samples_ls{sampler_(num_samples)};
//            auto colwise_ls = samples_ls.colwise();
//            double accum_phi_ls = 0;
//            std::for_each(colwise_ls.begin(), colwise_ls.end(), [&](auto const &sample) {
//                double phi_ls = cost_function(sample);
//                accum_phi_ls += phi_ls;
//            });
//            double new_V = accum_phi_ls/ double(num_samples) + log(precision_.determinant())/2.0;
//
//            if (new_V < l_V){
//                cout << "Lower value function" << endl << "precision_ " << endl << precision_.format(CleanFmt) << endl;
//                return true;
//            }
//
//            if (isnan(new_V)){
//                cout << "nan new cost function" << endl << "precision matrix" << endl
//                     << precision_.format(CleanFmt) << endl << "determinate" << precision_.determinant() << endl;
//                break;
//            }
//        }

//        // Update the sampler parameters
//        sampler_.updateMean(l_mu);
//        sampler_.updatePrecisionMatrix(l_precision_);

//        cout << "no lower value function" << endl << "precision_ " << endl << precision_.format(CleanFmt) << endl;
//        return false;

    }

    gtsam::Vector get_mean(){
        return mu_;
    }

    gtsam::Matrix get_precision(){
        return precision_;
    }

    gtsam::Matrix get_covariance(){
        return precision_.inverse();
    }

};