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
#include "../include/GaussianHermite.h"

using namespace GaussianSampler;
using namespace std;
using namespace Eigen;

IOFormat CleanFmt(4, 0, ", ", "\n");

template <typename Function, typename costClass, typename... Args>
class VariationalInferenceMPOptimizerFactorizedGaussHermite{
public:
    VariationalInferenceMPOptimizerFactorizedGaussHermite(const int& dimension, Function _function, const costClass& _cost_class):
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
            func_Vmu{[&](const VectorXd& x){return MatrixXd{(x-mu_)*cost_function_(x, cost_class_)};}},
            func_Vmumu{[&](const VectorXd& x){return MatrixXd{(x - mu_) * (x - mu_).transpose().eval()*cost_function_(x, cost_class_)};}},
            func_phi{[&](const VectorXd& x){return Matrix<double, 1, 1>{cost_function_(x, cost_class_)};}},
            gauss_hermite_Vmu_{10, dim_, mu_, covariance_, func_Vmu}{}
protected:
    // optimization variables
    int dim_;
    VectorXd mu_, d_mu;
    MatrixXd precision_, d_precision, covariance_;

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

    std::function<MatrixXd(const VectorXd&)> func_Vmu;
    std::function<MatrixXd(const VectorXd&)> func_Vmumu;
    std::function<MatrixXd(const VectorXd&)> func_phi;


    GaussHermite<std::function<VectorXd(const VectorXd&)>> gauss_hermite_Vmu_;

public:
    /**
     * Function used in the GH approximated integration for partial_V/partial_mu
     * */
//    VectorXd func_Vmu(const VectorXd& x){
//        return (x - mu_) * cost_function(x, cost_class_);
//    }

    /**
     * update the GH approximator
     * */
     void updateGH(){
        gauss_hermite_Vmu_.update_mean(mu_);
        gauss_hermite_Vmu_.update_P(covariance_);
     }

     void updateGHfunc(const std::function<MatrixXd(const VectorXd&)>& func){
         gauss_hermite_Vmu_.update_integrand(func);
     }

    /**
     * Caller of the cost function
     * */
    auto cost_function(Args... args){
        return cost_function_(args..., cost_class_);
    }

    /**
     * Update the step size
     * */
    void set_step_size(double ss_mean, double ss_precision){
        step_size_mu = ss_mean;
        step_size_Sigma = ss_precision;
    }

    /**
     * Update the parameters: mean, covariance, and precision matrix
     * */
    void update_mu(const VectorXd& new_mu){
        mu_ = new_mu;
    }

    void update_precision(const MatrixXd& new_precision){
        precision_ = new_precision;
    }

    void update_covariance(){
        covariance_ = precision_.inverse();
    }

    /**
     * Update the mean and covariance in the sampler
     * */
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

    /**
     * Update the mean and variance in the Gauss-Hermite approximator
     * */
    void update_GH_mean(){
        gauss_hermite_Vmu_.update_mean(mu_);
    }

    void update_GH_covariance(){
        gauss_hermite_Vmu_.update_P(covariance_);
    }

    /**
     * Main code: calculate phi * (partial V) / (partial mu), and phi * (partial V^2) / (partial mu * partial mu^T)
     * */
    void calculate_partial_V(){
        Vdmu.setZero();
        Vddmu.setZero();

        // GH approximation
        updateGH();
        /** Integrate for Vdmu **/
        gauss_hermite_Vmu_.update_integrand(func_Vmu);
        MatrixXd Vdmu = gauss_hermite_Vmu_.Integrate();
        Vdmu = precision_ * Vdmu;

        /** Integrate for Vddmu **/
        gauss_hermite_Vmu_.update_integrand(func_Vmumu);
        Vddmu = gauss_hermite_Vmu_.Integrate();

        /** Integrate for phi(x) **/
        gauss_hermite_Vmu_.update_integrand(func_phi);
        double avg_phi = gauss_hermite_Vmu_.Integrate()(0, 0);

        Vddmu.triangularView<Upper>() = (precision_ * Vddmu * precision_).triangularView<Upper>();
        Vddmu.triangularView<StrictlyLower>() = Vddmu.triangularView<StrictlyUpper>().transpose();

        gtsam::Matrix tmp{precision_ * avg_phi};

        Vddmu.triangularView<Upper>() = (Vddmu - precision_ * avg_phi).triangularView<Upper>();
        Vddmu.triangularView<StrictlyLower>() = Vddmu.triangularView<StrictlyUpper>().transpose();
    }


    /**
     * Gaussian posterior: closed-form expression
     * */
    void calculate_exact_partial_V(VectorXd mu_t, MatrixXd covariance_t){
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
        for (int i=0; i<dim_; i++){
            for (int j=0; j<dim_; j++) {
                for (int k=0; k<dim_; k++){
                    for (int l=0; l<dim_; l++){
                        tmp(i, j) += (covariance_(i, j)*covariance_(k, l) + covariance_(i,k)*covariance_(j,l) + covariance_(i,l)*covariance_(j,k))*precision_t(k,l);
                    }
                }
            }
        }

        Vddmu = precision_ * tmp * precision_ - precision_ * (precision_t*covariance_).trace();
        Vddmu = Vddmu / 2;

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

//        calculate_partial_V();
        calculate_exact_partial_V(cost_class_.get_mean(), cost_class_.get_covariance());

        d_precision = -precision_ + Vddmu;

        /// without backtracking
        precision_ = precision_ + step_size_Sigma * d_precision;
        d_mu = precision_.colPivHouseholderQr().solve(-Vdmu);

        mu_ = mu_ + step_size_mu * d_mu;

        return true;

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