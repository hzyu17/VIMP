/**
 * @file OptimizerFactorizedGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-03-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "SparseMatrixHelper.h"
#include "MVGsampler.h"
#include <gtsam/base/Matrix.h>
#include <iostream>
#include <random>
#include <utility>
#include "GaussHermite.h"
#include "MVGsampler.h"
#include "GaussianPriorUnaryTranslation.h"


using namespace GaussianSampler;
using namespace std;
using namespace Eigen;

IOFormat CleanFmt(4, 0, ", ", "\n");

namespace MPVI{
    template <typename Function, typename CostClass, typename... Args>
    /// Decription: The marginal optimizer using Gauss-Hermite quadrature to calculate the expectations
    class VIMPOptimizerFactorizedGaussHermite{
    public:
        ///@param dimension The dimension of the state
        ///@param _function Template function class which calculate the cost
        VIMPOptimizerFactorizedGaussHermite(const int& dimension, const Function& _function, const CostClass& _cost_class):
                dim_{dimension},
                cost_function_{std::move(_function)},
                mu_{VectorXd::Zero(dim_)},
                d_mu{VectorXd::Zero(dim_)},
                precision_{MatrixXd::Identity(dim_, dim_)},
                d_precision{MatrixXd::Zero(dim_, dim_)},
                covariance_{precision_.inverse()},
                Vdmu_{VectorXd::Zero(dim_)},
                Vddmu_{MatrixXd::Zero(dim_, dim_)},
                cost_class_{std::move(_cost_class)},
                func_phi_{[this](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, cost_function_(x, cost_class_))};}},
                gauss_hermite_{10, dim_, mu_, covariance_, func_phi_}{}
    protected:
        /// dimension
        int dim_;

        /// cost function
        Function cost_function_;

        /// optimization variables
        VectorXd mu_, d_mu;
        MatrixXd precision_, d_precision, covariance_;

        /// intermediate variables in optimization steps
        VectorXd Vdmu_;
        MatrixXd Vddmu_;

        /// step sizes
        double step_size_mu = 0.9;
        double step_size_Sigma = 0.9;

        /// Class of cost sources
        CostClass cost_class_;

        /// Intermediate functions for Gauss-Hermite quadratures
        std::function<MatrixXd(const VectorXd&)> func_phi_;

        /// G-H quadrature class
        GaussHermite<std::function<MatrixXd(const VectorXd&)>, CostClass> gauss_hermite_;

    public:

        /// update the GH approximator
        inline void updateGH(){
            gauss_hermite_.update_mean(VectorXd{mu_});
            gauss_hermite_.update_P(MatrixXd{covariance_});
        }

        /// Update the cost function
        inline void updateGHfunc(const std::function<MatrixXd(const VectorXd&)>& func){
            gauss_hermite_.update_integrand(func);
        }

        /// Update the step size
        inline void set_step_size(double ss_mean, double ss_precision){
            step_size_mu = ss_mean;
            step_size_Sigma = ss_precision;
        }

        /// Update the parameters: mean, covariance, and precision matrix
        inline void update_mu(const VectorXd& new_mu){
            mu_ = new_mu;
        }

        inline void update_precision(const MatrixXd& new_precision){
            precision_ = new_precision;
        }

        inline void update_covariance(){
            covariance_ = precision_.inverse();
        }

        /// Update the mean and variance in the Gauss-Hermite approximator
        inline void update_GH_mean(){
            gauss_hermite_.update_mean(mu_);
        }

        inline void update_GH_covariance(){
            gauss_hermite_.update_P(covariance_);
        }
        
        /// Main function: calculate phi * (partial V) / (partial mu), and phi * (partial V^2) / (partial mu * partial mu^T)
        void calculate_partial_V(){

            // cost_class_.printKeys();

            // VectorXd test_vec = (VectorXd(4) << 1.0, 1.0, 0.0, 0.0).finished();
            // double temp {cost_function_(test_vec, cost_class_)};
            // cout << "temp GH" << endl << temp << endl;
            
            Vdmu_.setZero();
            Vddmu_.setZero();

            // GH approximation
            update_covariance();

            // cout << "temp GH" << endl << temp << endl;

            updateGH();

            /** Integrate for Vdmu_ **/
            std::function<MatrixXd(const VectorXd&)> func_Vmu_ = [this](const VectorXd& x){return (x-VectorXd{mu_}) * cost_function_(x, CostClass{cost_class_});};

            // cout << "temp GH" << endl << temp << endl;

            gauss_hermite_.update_integrand(func_Vmu_);
            Vdmu_ = gauss_hermite_.Integrate();
            Vdmu_ = precision_ * Vdmu_;

            /** Integrate for phi(x) **/
            std::function<MatrixXd(const VectorXd&)> func_phi_ = [this](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, cost_function_(x, CostClass{cost_class_}))};};
            gauss_hermite_.update_integrand(func_phi_);
            double avg_phi = gauss_hermite_.Integrate()(0, 0);

            /** Integrate for Vddmu_ **/
            std::function<MatrixXd(const VectorXd&)> func_Vmumu_ = [this](const VectorXd& x){return MatrixXd{(x-VectorXd{mu_}) * (x-VectorXd{mu_}).transpose().eval() * cost_function_(x, CostClass{cost_class_})};};
            gauss_hermite_.update_integrand(func_Vmumu_);
            Vddmu_ = gauss_hermite_.Integrate();

            Vddmu_.triangularView<Upper>() = (precision_ * Vddmu_ * precision_).triangularView<Upper>();
            Vddmu_.triangularView<StrictlyLower>() = Vddmu_.triangularView<StrictlyUpper>().transpose();

            // gtsam::Matrix tmp{precision_ * avg_phi};

            Vddmu_.triangularView<Upper>() = (Vddmu_ - precision_ * avg_phi).triangularView<Upper>();
            Vddmu_.triangularView<StrictlyLower>() = Vddmu_.triangularView<StrictlyUpper>().transpose();
        }

        /// Gaussian posterior: closed-form expression
        void calculate_exact_partial_V(VectorXd mu_t, MatrixXd covariance_t){
            Vdmu_.setZero();
            Vddmu_.setZero();
            update_covariance();

            // helper vectors
            VectorXd eps{mu_ - mu_t};
            MatrixXd tmp{MatrixXd::Zero(dim_, dim_)};
            MatrixXd precision_t{covariance_t.inverse()};

            // partial V / partial mu
            Vdmu_ = precision_t * eps;

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

            Vddmu_ = precision_ * tmp * precision_ - precision_ * (precision_t*covariance_).trace();
            Vddmu_ = Vddmu_ / 2;

            }

        inline MatrixXd get_Vddmu() const{
            return Vddmu_;
        }

        inline VectorXd get_Vdmu() const{
            return Vdmu_;
        }

        bool step(){
            // Zero grad
            d_mu.setZero();
            d_precision.setZero();

            calculate_partial_V();
    //        calculate_exact_partial_V(cost_class_.get_mean(), cost_class_.get_covariance());

            d_precision = -precision_ + Vddmu_;

            /// without backtracking
            precision_ = precision_ + step_size_Sigma * d_precision;

            d_mu = precision_.colPivHouseholderQr().solve(-Vdmu_);

            mu_ = mu_ + step_size_mu * d_mu;

            return true;

        }

        inline VectorXd get_mean() const{
            return mu_;
        }

        inline MatrixXd get_precision() const{
            return precision_;
        }

        inline MatrixXd get_covariance() const{
            return precision_.inverse();
        }

    };
}