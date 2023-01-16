/**
 * @file OptimizerFactorizedGHBase.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The base class for marginal optimizer.
 * @version 0.1
 * @date 2022-03-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once


#include <iostream>
#include <random>
#include <utility>
#include <assert.h>
#include <random>

#include "../helpers/GaussHermite.h"
#include "../helpers/sparse_graph.h"


using namespace std;
using namespace Eigen;

IOFormat CleanFmt(4, 0, ", ", "\n");

namespace vimp{
    class VIMPOptimizerFactorizedBase{
    public:
        
        /**
         * @brief Default Constructor
         */
        VIMPOptimizerFactorizedBase(){}

        /**
         * @brief Construct a new VIMPOptimizerFactorizedBase object
         * 
         * @param dimension The dimension of the state
         * @param Pk_ Mapping matrix from marginal to joint
         */
        VIMPOptimizerFactorizedBase(int dimension, int state_dim, int num_states, int start_index):
                _dim{dimension},
                _state_dim{state_dim},
                _num_states{num_states},
                _mu{VectorXd::Zero(_dim)},
                _covariance{MatrixXd::Identity(_dim, _dim)},
                _precision{MatrixXd::Identity(_dim, _dim)},
                _dprecision{MatrixXd::Zero(_dim, _dim)},
                _Vdmu{VectorXd::Zero(_dim)},
                _Vddmu{MatrixXd::Zero(_dim, _dim)},
                // _Pk{Pk},
                _block{state_dim, num_states, start_index, dimension}
                {
                    _joint_size = _state_dim * _num_states;
                }
    
    
    /// Public members for the inherited classes access
    public:

        /// dimension
        int _dim, _state_dim, _num_states, _joint_size;

        VectorXd _mu;
        MatrixXd _covariance;
        
        /// Intermediate functions for Gauss-Hermite quadratures, default definition, needed to be overrided by the
        /// derived classes.
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;
        GHFunction _func_phi = GHFunction();
        GHFunction _func_Vmu = GHFunction();
        GHFunction _func_Vmumu = GHFunction();

        /// G-H quadrature class
        GaussHermite<GHFunction> _gauss_hermite = GaussHermite<GHFunction>();

    private:
        /// optimization variables
        MatrixXd _precision, _dprecision;

        /// intermediate variables in optimization steps
        VectorXd _Vdmu;
        MatrixXd _Vddmu;

        /// step sizes
        double _step_size_mu = 0.9;
        double _step_size_Sigma = 0.9;

        /// Mapping matrix to the joint distribution
        // MatrixXd _Pk;

        // sparse mapping
        TrajectoryBlock _block;

        // Sparse inverser and matrix helpers
        EigenWrapper _eigen_wrapper = EigenWrapper();

    /// public functions
    public:
        /// update the GH approximator
        void updateGH(){
            _gauss_hermite.update_mean(_mu);
            _gauss_hermite.update_P(_covariance); }

        /// update the GH approximator
        void updateGH(const VectorXd& x, const MatrixXd& P){
            _gauss_hermite.update_mean(x);
            _gauss_hermite.update_P(P); }

        /**
         * @brief Update the step size
         */
        inline void set_step_size(double ss_mean, double ss_precision){
            _step_size_mu = ss_mean;
            _step_size_Sigma = ss_precision; }


        /**
         * @brief Update mean
         */
        inline void update_mu(const VectorXd& new_mu){ 
            _mu = new_mu; }


        /**
         * @brief Update covariance matrix
         */
        inline void update_covariance(const MatrixXd& new_cov){ 
            _covariance = new_cov; 
            _precision = _covariance.inverse();
        }


        /**
         * @brief Update the marginal mean using JOINT mean and 
         * mapping matrix Pk.
         * 
         * @param joint_mean 
         */
        // inline void update_mu_from_joint_mean(const VectorXd& joint_mean){ 
        //     _mu = _Pk * joint_mean;}

        inline void update_mu_from_joint(const VectorXd & joint_mean){
            _mu = _block.extract_vector(joint_mean);
        }

        inline VectorXd extract_mu_from_joint(const VectorXd & joint_mean){
            VectorXd res(_dim);
            res = _block.extract_vector(joint_mean);
            return res;
        }

        inline MatrixXd extract_cov_from_joint(const MatrixXd& joint_covariance){
            SpMat j_covariance = joint_covariance.sparseView();
            MatrixXd covariance = _block.extract(j_covariance);
            return covariance;
        }

        /**
         * @brief Update the marginal precision matrix.
         */
        inline void update_precision_from_joint(const SpMat& joint_covariance){
            _covariance = _block.extract(joint_covariance);
            _precision = _covariance.inverse();
        }


        /**
         * @brief Main function calculating phi * (partial V) / (partial mu), and 
         * phi * (partial V^2) / (partial mu * partial mu^T)
         * 
         * @return * void 
         */
        void calculate_partial_V();


        /**
         * @brief Main function calculating phi * (partial V) / (partial mu), and 
         * phi * (partial V^2) / (partial mu * partial mu^T) for Gaussian posterior: closed-form expression
         */
        void calculate_exact_partial_V(VectorXd mu_t, MatrixXd covariance_t);


        /**
         * @brief One step in the optimization.
         */
        bool step();

        
        /**
         * @brief Compute the cost function. V(x) = E_q(\phi(x))
         */
        double cost_value(const VectorXd& x, const MatrixXd& Cov);


        /**
         * @brief Compute the cost function. V(x) = E_q(\phi(x)) using the current values.
         */
        double cost_value();


        /**
         * @brief Get the marginal intermediate variable (partial V^2 / par mu / par mu)
         */
        inline MatrixXd Vddmu() const { return _Vddmu; }


        /**
         * @brief Get the marginal intermediate variable partial V / dmu
         */
        inline VectorXd Vdmu() const { return _Vdmu; }


        /**
         * @brief Get the joint intermediate variable (partial V / partial mu).
         */
        // inline VectorXd joint_Vdmu() const { return _Pk.transpose() * _Vdmu; }

        inline VectorXd joint_Vdmu_sp() { 
            VectorXd res(_joint_size);
            _block.fill_vector(res, _Vdmu);
            return res;
            }

        /**
         * @brief Get the joint Pk.T * V^2 / dmu /dmu * Pk
         */
        // inline MatrixXd joint_Vddmu() const { return _Pk.transpose().eval() * _Vddmu * _Pk; }

        /**
         * @brief Get the joint Pk.T * V^2 / dmu /dmu * Pk using block insertion
         */
        inline MatrixXd joint_Vddmu_sp() { 
            
            SpMat res(_joint_size, _joint_size);
            res.setZero();
            _block.fill(_Vddmu, res);
            MatrixXd res_full{res};
            return res_full;
        }

        /**
         * @brief Get the mapping matrix Pk
         */
        // inline MatrixXd Pk() const { return _Pk; }


        inline TrajectoryBlock block() const {return _block;}

        /**
         * @brief Get the mean 
         */
        inline VectorXd mean() const{ return _mu; }


        /**
         * @brief Get the precision matrix
         */
        inline MatrixXd precision() const{ 
            assert((_precision - _covariance.inverse()).norm()==0); 
            return _precision; }


        /**
         * @brief Get the covariance matrix
         */
        inline MatrixXd covariance() const{ return _covariance;}

        /********************************************************/
        /// Function interfaces

        /**
         * @brief returns the Phi(x) 
         */
        inline MatrixXd Phi(const VectorXd& x) const{
            return _func_phi(x);
        }

        /**
         * @brief returns the (x-mu)*Phi(x) 
         */
        inline MatrixXd xMuPhi(const VectorXd& x) const{
            return _func_Vmu(x);
        }

        /**
         * @brief returns the (x-mu)*Phi(x) 
         */
        inline MatrixXd xMuxMuTPhi(const VectorXd& x) const{
            return _func_Vmumu(x);
        }

        /**
         * @brief returns the E_q{phi(x)} = E_q{-log(p(x,z))}
         */
        inline double E_Phi() {
            _gauss_hermite.update_integrand(_func_phi);
            return _gauss_hermite.Integrate()(0, 0);
        }

        inline MatrixXd E_xMuPhi(){
            _gauss_hermite.update_integrand(_func_Vmu);
            return _gauss_hermite.Integrate();
        }

        inline MatrixXd E_xMuxMuTPhi(){
            _gauss_hermite.update_integrand(_func_Vmumu);
            return _gauss_hermite.Integrate();
        }

        void set_GH_points(int p){
            _gauss_hermite.set_polynomial_deg(p);
        }
    };

}
#include "../optimizer/OptimizerFactorizedGHBase-impl.h"