/**
 * @file OptimizerFactorizedGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The marginal optimizer using Gauss-Hermite quadrature to calculate the expectations.
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


using namespace GaussianSampler;
using namespace std;
using namespace Eigen;

IOFormat CleanFmt(4, 0, ", ", "\n");

namespace MPVI{
    template <typename Function, typename CostClass>
    class VIMPOptimizerFactorizedGaussHermite{
    public:
        ///@param dimension The dimension of the state
        ///@param function_ Template function class which calculate the cost
        VIMPOptimizerFactorizedGaussHermite(const int& dimension, const Function& function_, const CostClass& cost_class_, const MatrixXd& Pk_):
                _dim{dimension},
                _cost_function{std::move(function_)},
                _mu{VectorXd::Zero(_dim)},
                _dmu{VectorXd::Zero(_dim)},
                _precision{MatrixXd::Identity(_dim, _dim)},
                _dprecision{MatrixXd::Zero(_dim, _dim)},
                _covariance{_precision.inverse()},
                _Vdmu{VectorXd::Zero(_dim)},
                _Vddmu{MatrixXd::Zero(_dim, _dim)},
                _cost_class{std::move(cost_class_)},
                _func_phi{[this](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, _cost_function(x, _cost_class))};}},
                _func_Vmu{[this](const VectorXd& x){return (x-_mu) * _cost_function(x, _cost_class);}},
                _func_Vmumu{[this](const VectorXd& x){return MatrixXd{(x-_mu) * (x-_mu).transpose().eval() * _cost_function(x, _cost_class)};}},
                _gauss_hermite{10, _dim, _mu, _covariance, _func_phi},
                _Pk{Pk_}
                {}
    protected:

        /// optimization variables
        VectorXd _dmu;
        MatrixXd _precision, _dprecision;

        /// intermediate variables in optimization steps
        VectorXd _Vdmu;
        MatrixXd _Vddmu;

        /// step sizes
        double _step_size_mu = 0.9;
        double _step_size_Sigma = 0.9;

        /// Intermediate functions for Gauss-Hermite quadratures
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;
        GHFunction _func_phi;
        GHFunction _func_Vmu;
        GHFunction _func_Vmumu;

        /// G-H quadrature class
        GaussHermite<GHFunction> _gauss_hermite;

        /// Mapping matrix to the joint distribution
        MatrixXd _Pk;
    
    /// Public members for the inherited classes access
    public:
        /// dimension
        int _dim;

        VectorXd _mu;
        MatrixXd _covariance;

        /// Class of cost sources
        CostClass _cost_class;

        /// cost function
        Function _cost_function;

    public:

        /// update the GH approximator
        inline void updateGH(){
            _gauss_hermite.update_mean(VectorXd{_mu});
            _gauss_hermite.update_P(MatrixXd{_covariance});
        }

        /// Update the cost function
        inline void updateGHfunc(const GHFunction& func){
            _gauss_hermite.update_integrand(func);
        }

        /// Update the step size
        inline void set_step_size(double ss_mean, double ss_precision){
            _step_size_mu = ss_mean;
            _step_size_Sigma = ss_precision;
        }

        /// Update the parameters: mean, covariance, and precision matrix
        inline void update_mu(const VectorXd& new_mu){
            _mu = _Pk * new_mu;
        }

        /**
         * @brief Update the marginal precision matrix using joint COVARIANCE matrix and 
         * mapping matrix Pk.
         * 
         * @param joint_covariance 
         */
        inline void update_precision_from_joint_covariance(const MatrixXd& joint_covariance){
            _precision = (_Pk * joint_covariance * _Pk.transpose()).inverse();
        }

        inline void update_covariance(){
            _covariance = _precision.inverse();
        }

        /// Update the mean and variance in the Gauss-Hermite approximator
        inline void update_GH_mean(){
            _gauss_hermite.update_mean(_mu);
        }

        inline void update_GH_covariance(){
            _gauss_hermite.update_P(_covariance);
        }
        
        /**
         * @brief Main function calculating phi * (partial V) / (partial mu), and 
         * phi * (partial V^2) / (partial mu * partial mu^T)
         * 
         * @return * void 
         */
        void calculate_partial_V(){
            
            _Vdmu.setZero();
            _Vddmu.setZero();

            // GH approximation
            update_covariance();
            updateGH();

            /// Integrate for _Vdmu 
            // std::function<MatrixXd(const VectorXd&)> _func_Vmu = [this](const VectorXd& x){return (x-_mu) * _cost_function(x, CostClass{_cost_class});};
            _gauss_hermite.update_integrand(_func_Vmu);

            _Vdmu = _gauss_hermite.Integrate();
            _Vdmu = _precision * _Vdmu;

            /// Integrate for phi(x)
            // std::function<MatrixXd(const VectorXd&)> _func_phi = [this](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, _cost_function(x, CostClass{_cost_class}))};};
            _gauss_hermite.update_integrand(_func_phi);
            double avg_phi = _gauss_hermite.Integrate()(0, 0);

            /// Integrate for partial V^2 / ddmu_ 
            // std::function<MatrixXd(const VectorXd&)> _func_Vmumu = [this](const VectorXd& x){return MatrixXd{(x-_mu) * (x-_mu).transpose().eval() * _cost_function(x, CostClass{_cost_class})};};
            _gauss_hermite.update_integrand(_func_Vmumu);
            _Vddmu = _gauss_hermite.Integrate();

            _Vddmu.triangularView<Upper>() = (_precision * _Vddmu * _precision).triangularView<Upper>();
            _Vddmu.triangularView<StrictlyLower>() = _Vddmu.triangularView<StrictlyUpper>().transpose();

            _Vddmu.triangularView<Upper>() = (_Vddmu - _precision * avg_phi).triangularView<Upper>();
            _Vddmu.triangularView<StrictlyLower>() = _Vddmu.triangularView<StrictlyUpper>().transpose();
        }

        /// Gaussian posterior: closed-form expression
        void calculate_exact_partial_V(VectorXd mu_t, MatrixXd covariance_t){
            _Vdmu.setZero();
            _Vddmu.setZero();
            update_covariance();

            // helper vectors
            VectorXd eps{_mu - mu_t};
            MatrixXd tmp{MatrixXd::Zero(_dim, _dim)};
            MatrixXd precision_t{covariance_t.inverse()};

            // partial V / partial mu
            _Vdmu = precision_t * eps;

            // partial V^2 / partial mu*mu^T
            // update tmp matrix
            for (int i=0; i<_dim; i++){
                for (int j=0; j<_dim; j++) {
                    for (int k=0; k<_dim; k++){
                        for (int l=0; l<_dim; l++){
                            tmp(i, j) += (_covariance(i, j)*_covariance(k, l) + _covariance(i,k)*_covariance(j,l) + _covariance(i,l)*_covariance(j,k))*precision_t(k,l);
                        }
                    }
                }
            }

            _Vddmu = _precision * tmp * _precision - _precision * (precision_t*_covariance).trace();
            _Vddmu = _Vddmu / 2;

        }

        inline MatrixXd get_Vddmu() const {
            return _Vddmu;
        }

        inline VectorXd get_Vdmu() const {
            return _Vdmu;
        }

        /// return the joint incremental mean after the mapping Pk from marginals.
        inline VectorXd get_joint_Vdmu() const {
            return _Pk.transpose() * _Vdmu;
        }

        inline MatrixXd get_joint_Vddmu() const {
            return _Pk.transpose().eval() * _Vddmu * _Pk;
        }

        inline MatrixXd Pk() const {
            return _Pk;
        }

        /**
         * @brief One step in the optimization.
         * 
         * @return true: success.
         */
        bool step(){
            // Zero grad
            _dmu.setZero();
            _dprecision.setZero();

            // test_GH_functions();

            calculate_partial_V();
        //    calculate_exact_partial_V(_cost_class.get_mean(), _cost_class.get_covariance());

            _dprecision = -_precision + _Vddmu;

            /// without backtracking
            _precision = _precision + _step_size_Sigma * _dprecision;

            _dmu = _precision.colPivHouseholderQr().solve(-_Vdmu);

            _mu = _mu + _step_size_mu * _dmu;

            return true;

        }

        inline VectorXd get_mean() const{
            return _mu;
        }

        inline MatrixXd get_precision() const{
            return _precision;
        }

        inline MatrixXd get_covariance() const{
            return _precision.inverse();
        }

    };
}