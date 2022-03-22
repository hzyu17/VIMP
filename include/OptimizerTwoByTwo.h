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
            dim{dimension},
            cost_function_{std::forward<Function>(_function)},
            cost_class_{_cost_class},
            d_mu{VectorXd::Zero(dimension)},
            mu_{VectorXd::Zero(dimension)},
            Vdmu{VectorXd::Zero(dimension)},
            Vddmu{MatrixXd::Zero(dimension, dimension)},
            precision_{gtsam::Matrix::Identity(dimension, dimension)},
            d_precision{gtsam::Matrix::Zero(dimension, dimension)},
            sampler_{normal_random_variable(mu_, precision_.inverse())}{}
protected:
    // optimization variables
    int dim;
    int num_samples = 50000;
    VectorXd mu_, d_mu;
    MatrixXd precision_, d_precision;

    VectorXd Vdmu;
    MatrixXd Vddmu;

    // step sizes
    double step_size_mu = 0.9;
    double step_size_Sigma = 0.9;

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

    void updateSamplerCovarianceMatrix(const MatrixXd& new_cov){
        sampler_.updateCovariance(new_cov);
    }

    void updateSamplerCovarianceMatrix(){
        sampler_.updateCovariance(precision_.inverse());
    }

    void updateSamplerMean(){
        sampler_.updateMean(mu_);
    }

    void updateSamplerMean(const VectorXd& new_mu){
        sampler_.updateMean(new_mu);
    }

    void update_mu(const VectorXd& new_mu){
        mu_ = new_mu;
    }

    void update_precision(const MatrixXd& new_precision){
        precision_ = new_precision;
    }

    void calculate_partial_V(){
        Vdmu.setZero();
        Vddmu.setZero();

        MatrixXd samples{sampler_(num_samples)};

        auto colwise = samples.colwise();
        double accum_phi = 0;
        std::for_each(colwise.begin(), colwise.end(), [&](auto const &sample) {
            double phi = cost_function(sample);

            accum_phi += phi;

            Vdmu = Vdmu + (sample - mu_) * phi;
            Vddmu = Vddmu + (sample - mu_) * (sample - mu_).transpose().eval() * phi;
        });

        Vdmu = precision_ * Vdmu.eval() / double(num_samples);
        Vddmu.triangularView<Upper>() = (precision_ * Vddmu * precision_).triangularView<Upper>();
        Vddmu.triangularView<StrictlyLower>() = Vddmu.triangularView<StrictlyUpper>().transpose();

        Vddmu = Vddmu.eval() / double(num_samples);

        double avg_phi = accum_phi / double(num_samples);

        gtsam::Matrix tmp{precision_ * avg_phi};

        Vddmu.triangularView<Upper>() = (Vddmu - precision_ * avg_phi).triangularView<Upper>();
        Vddmu.triangularView<StrictlyLower>() = Vddmu.triangularView<StrictlyUpper>().transpose();
    }

    void calculate_exact_partial_V(){
        // In the case of a Gaussian posterior, the expectations can be calculated exactly


    }

    MatrixXd get_Vddmu(){
        return Vddmu;
    }

    VectorXd get_Vdmu(){
        return Vdmu;
    }

    bool step(){
        // Zero grad
        d_mu.setZero();
        d_precision.setZero();

        calculate_partial_V();

        d_precision = -precision_ + Vddmu;

        /// without backtracking
        precision_ = precision_ + step_size_Sigma * d_precision;
        d_mu = precision_.colPivHouseholderQr().solve(-Vdmu);

        mu_ = mu_ + step_size_mu * d_mu;

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

    VectorXd get_mean(){
        return mu_;
    }

    MatrixXd get_precision(){
        return precision_;
    }

    MatrixXd get_covariance(){
        return precision_.inverse();
    }

};