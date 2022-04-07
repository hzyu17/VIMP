//
// Created by hongzhe on 4/2/22.
//

/**
 * Factorized optimizer for Variational GPMP, cost including Gaussian prior and collisions.
 */

#ifndef MPVI_OPTIMIZERTWOFACTORS_H
#define MPVI_OPTIMIZERTWOFACTORS_H

//#include "SparseMatrixHelper.h"
#include "MVGsampler.h"
#include <gtsam/base/Matrix.h>
#include <iostream>
#include <random>
#include <utility>
#include <boost/optional.hpp>
#include <optional>

using namespace GaussianSampler;
using namespace std;
using namespace Eigen;

IOFormat CleanFmt(4, 0, ", ", "\n");
namespace MPVI {
    template<typename Function, typename CostClass, typename... Args>
    class VIMPOptimizerFactorizedTwoFactors {
    public:

        VIMPOptimizerFactorizedTwoFactors(
                const int &dimension,
                Function _function,
                const CostClass &_cost_class,
                const MatrixXd &Pk) :
                dim_{dimension},
                cost_function_{std::forward<Function>(_function)},
                cost_class_{_cost_class},
                d_mu{VectorXd::Zero(dim_)},
                mu_{VectorXd::Zero(dim_)},
                Vdmu{VectorXd::Zero(dim_)},
                Vddmu{MatrixXd::Zero(dim_, dim_)},
                precision_{MatrixXd::Identity(dim_, dim_)},
                d_precision{MatrixXd::Zero(dim_, dim_)},
                covariance_{precision_.inverse()},
                sampler_{normal_random_variable(mu_, precision_.inverse())},
                Pk_{Pk} {}

    protected:
        // optimization variables
        int dim_;
        int num_samples = 50000;
        VectorXd mu_, d_mu;
        MatrixXd precision_, d_precision, covariance_;

        MatrixXd Pk_;

        VectorXd Vdmu;
        MatrixXd Vddmu;

        // step sizes
        double step_size_mu = 0.9;
        double step_size_Sigma = 0.9;

        // sampler
        normal_random_variable sampler_;

        // cost functional. Input: samples vector; Output: cost
        Function cost_function_;
        CostClass cost_class_;

    public:

        MatrixXd Pk() {
            return Pk_;
        }

        auto cost_function(Args... args) {
            return cost_function_(args..., cost_class_);
        }

        void set_step_size(double ss_mean, double ss_precision) {
            step_size_mu = ss_mean;
            step_size_Sigma = ss_precision;
        }

        void updateSamplerCovarianceMatrix(const MatrixXd &new_cov) {
            sampler_.updateCovariance(new_cov);
        }

        void updateSamplerCovarianceMatrix() {
            sampler_.updateCovariance(precision_.inverse());
        }

        void updateSamplerMean() {
            sampler_.updateMean(mu_);
        }

        void updateSamplerMean(const VectorXd &new_mu) {
            sampler_.updateMean(new_mu);
        }

        void update_mu(const VectorXd &new_mu) {
            mu_ = new_mu;
        }

        void update_precision(const MatrixXd &new_precision) {
            precision_ = new_precision;
        }

        void update_covariance() {
            covariance_ = precision_.inverse();
        }

        void calculate_partial_V() {
            Vdmu.setZero();
            Vddmu.setZero();

            MatrixXd samples{sampler_(num_samples)};

            auto colwise = samples.colwise();
            double accum_phi = 0;

            std::for_each(colwise.begin(), colwise.end(), [&](auto const &sample) {
                double phi = cost_function(sample);
//            double phi = 1;

                accum_phi += phi;

                Vdmu = Vdmu + (sample - mu_) * phi;
                Vddmu = Vddmu + (sample - mu_) * (sample - mu_).transpose().eval() * phi;
            });

            Vdmu = precision_ * Vdmu.eval() / double(num_samples);
            Vddmu.triangularView<Upper>() = (precision_ * Vddmu * precision_).triangularView<Upper>();
            Vddmu.triangularView<StrictlyLower>() = Vddmu.triangularView<StrictlyUpper>().transpose();

            Vddmu = Vddmu.eval() / double(num_samples);

//        cout << "mu" << endl << mu_ << endl;
//        cout << "covariance " << endl << covariance_ << endl;
//
//        cout << "sampler precision matrix" << endl << sampler_.get_precision() << endl;
//
//        cout << "error" << endl << (sampler_.get_precision() - Vddmu).norm() << endl;
//
//        cout << "Vdmu " << endl << Vdmu << endl;
//        cout << "Vddmu " << endl << Vddmu << endl;

            double avg_phi = accum_phi / double(num_samples);

            gtsam::Matrix tmp{precision_ * avg_phi};

            Vddmu.triangularView<Upper>() = (Vddmu - precision_ * avg_phi).triangularView<Upper>();
            Vddmu.triangularView<StrictlyLower>() = Vddmu.triangularView<StrictlyUpper>().transpose();
        }

        /// Gaussian posterior: closed-form expression
        void calculate_exact_partial_V(VectorXd mu_t, MatrixXd covariance_t) {
            Vdmu.setZero();
            Vddmu.setZero();
            update_covariance();

            // helper vectors
            VectorXd eps{mu_ - mu_t};
            MatrixXd tmp{MatrixXd::Zero(dim_, dim_)};
            MatrixXd precision_t{covariance_t.inverse()};

            // partial V / partial mu
            Vdmu = precision_t * eps;

            // partial V^2 / partial mu*mu^T
            // update tmp matrix
            for (int i = 0; i < dim_; i++) {
                for (int j = 0; j < dim_; j++) {
                    for (int k = 0; k < dim_; k++) {
                        for (int l = 0; l < dim_; l++) {
                            tmp(i, j) +=
                                    (covariance_(i, j) * covariance_(k, l) + covariance_(i, k) * covariance_(j, l) +
                                     covariance_(i, l) * covariance_(j, k)) * precision_t(k, l);
                        }
                    }
                }
            }

            Vddmu = precision_ * tmp * precision_ - precision_ * (precision_t * covariance_).trace();
            Vddmu = Vddmu / 2;

        }

        MatrixXd get_Vddmu() {
            return Vddmu;
        }

        VectorXd get_Vdmu() {
            return Vdmu;
        }

        bool step() {
            // Zero grad
            d_mu.setZero();
            d_precision.setZero();

            calculate_partial_V();
//        calculate_exact_partial_V(collision_factor_class_.get_mean(), collision_factor_class_.get_covariance());

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

        VectorXd get_mean() {
            return mu_;
        }

        MatrixXd get_precision() {
            return precision_;
        }

        MatrixXd get_covariance() {
            return precision_.inverse();
        }

    };
}
#endif //MPVI_OPTIMIZERTWOFACTORS_H