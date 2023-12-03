/**
 * @file GVIFactorizedBase.h
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

#include "helpers/EigenWrapper.h"
#include "GaussHermite.h"
#include "helpers/MatrixHelper.h"
#include "helpers/timer.h"


using namespace std;
using namespace Eigen;

IOFormat CleanFmt(4, 0, ", ", "\n");

namespace vimp{
    class GVIFactorizedBase{
    public:
        virtual ~GVIFactorizedBase(){}
        /**
         * @brief Default Constructor
         */
        GVIFactorizedBase(){}

        /**
         * @brief Construct a new GVIFactorizedBase object
         * 
         * @param dimension The dimension of the state
         */
        GVIFactorizedBase(int dimension, int state_dim, int num_states, int start_index, 
                          double temperature=10.0, double high_temperature=100.0, bool is_linear=false):
                _is_linear{is_linear},
                _dim{dimension},
                _state_dim{state_dim},
                _num_states{num_states},
                _temperature{temperature},
                _high_temperature{high_temperature},
                _mu(_dim),
                _covariance{MatrixXd::Identity(_dim, _dim)},
                _precision{MatrixXd::Identity(_dim, _dim)},
                _dprecision(_dim, _dim),
                _Vdmu(_dim),
                _Vddmu(_dim, _dim),
                _block{state_dim, num_states, start_index, dimension},
                _Pk(dimension, state_dim*num_states)
                {   
                    _joint_size = state_dim * num_states;
                    _Pk.setZero();
                    _Pk.block(0, start_index*state_dim, dimension, dimension) = Eigen::MatrixXd::Identity(dimension, dimension);
                }        
        
    /// public functions
    public:

        /// update the GH approximator
        void updateGH(const VectorXd& x, const MatrixXd& P){
            _gh->update_mean(x);
            _gh->update_P(P); 
        }

        /**
         * @brief Update the step size
         */
        inline void set_step_size(double ss_mean, double ss_precision){
            _step_size_mu = ss_mean;
            _step_size_Sigma = ss_precision; 
        }

        /**
         * @brief Update mean
         */
        inline void update_mu(const VectorXd& new_mu){ 
            _mu = new_mu; 
        }

        /**
         * @brief Update covariance matrix
         */
        inline void update_covariance(const MatrixXd& new_cov){ 
            _covariance = new_cov; 
            _precision = _covariance.inverse();
        }

        inline MatrixXd Pk(){
            return _Pk;
        }

        /**
         * @brief Update the marginal mean.
         */
        inline void update_mu_from_joint(const VectorXd & joint_mean) {
            _mu = _block.extract_vector(joint_mean);
        }

        /**
         * @brief Update the marginal precision matrix.
         */
        inline void update_precision_from_joint(const SpMat& joint_covariance) {
            _covariance = extract_cov_from_joint(joint_covariance);
            _precision = _covariance.inverse();
        }

        inline VectorXd extract_mu_from_joint(const VectorXd & joint_mean) {
            return _block.extract_vector(joint_mean);
        }

        inline SpMat extract_cov_from_joint(const SpMat& joint_covariance) {
            return _block.extract(joint_covariance);
        }


        /**
         * @brief Calculating phi * (partial V) / (partial mu), and 
         * phi * (partial V^2) / (partial mu * partial mu^T)
         */
        virtual void calculate_partial_V(){
            // update the mu and sigma inside the gauss-hermite integrator
            updateGH(_mu, _covariance);

            _Vdmu.setZero();
            _Vddmu.setZero();

            /// Integrate for E_q{_Vdmu} 
            _Vdmu = _gh->Integrate(_func_Vmu);
            _Vdmu = _precision * _Vdmu;

            /// Integrate for E_q{phi(x)}
            double E_phi = _gh->Integrate(_func_phi)(0, 0);
            
            /// Integrate for partial V^2 / ddmu_ 
            MatrixXd E_xxphi{_gh->Integrate(_func_Vmumu)};

            // MatrixXd Vddmu{MatrixXd::Zero(_dim, _dim)};
            _Vddmu.triangularView<Upper>() = (_precision * E_xxphi * _precision - _precision * E_phi).triangularView<Upper>();
            _Vddmu.triangularView<StrictlyLower>() = _Vddmu.triangularView<StrictlyUpper>().transpose();

        }

        void test_integration(){
            std::cout << "=========== test_integration ===========" << std::endl;
            updateGH(_mu, _covariance);
            double E_phi = _gh->Integrate(_func_phi)(0, 0);
            std::cout << "E_phi " << std::endl << E_phi << std::endl;

            VectorXd Vdmu = _gh->Integrate(_func_Vmu);
            _ei.print_matrix(Vdmu, "E_xphi");

            MatrixXd E_xxphi = _gh->Integrate(_func_Vmumu);
            _ei.print_matrix(E_xxphi, "E_xxphi");
        }

        void calculate_partial_V_GH(){
            // update the mu and sigma inside the gauss-hermite integrator
            updateGH(_mu, _covariance);

            _Vdmu.setZero();
            _Vddmu.setZero();

            /// Integrate for E_q{_Vdmu} 
            _Vdmu = _gh->Integrate(_func_Vmu);
            _Vdmu = _precision * _Vdmu;

            /// Integrate for E_q{phi(x)}
            double E_phi = _gh->Integrate(_func_phi)(0, 0);
            
            /// Integrate for partial V^2 / ddmu_ 
            MatrixXd E_xxphi{_gh->Integrate(_func_Vmumu)};

            _Vddmu.triangularView<Upper>() = (_precision * E_xxphi * _precision - _precision * E_phi).triangularView<Upper>();
            _Vddmu.triangularView<StrictlyLower>() = _Vddmu.triangularView<StrictlyUpper>().transpose();

        }

        /**
         * @brief Compute the cost function. V(x) = E_q(\phi(x))
         */
        virtual double fact_cost_value(const VectorXd& joint_mean, const SpMat& joint_cov) {
            
            VectorXd mean_k = extract_mu_from_joint(joint_mean);
            MatrixXd Cov_k = extract_cov_from_joint(joint_cov);

            updateGH(mean_k, Cov_k);

            return _gh->Integrate(_func_phi)(0, 0);
        }

        // /**
        //  * @brief Compute the cost function. V(x) = E_q(\phi(x)) using the current values.
        //  */

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
        inline VectorXd joint_Vdmu_sp() { 
            VectorXd res(_joint_size);
            res.setZero();
            _block.fill_vector(res, _Vdmu);
            return res;
        }

        /**
         * @brief Get the joint Pk.T * V^2 / dmu /dmu * Pk using block insertion
         */
        inline SpMat joint_Vddmu_sp() { 
            SpMat res(_joint_size, _joint_size);
            res.setZero();
            _block.fill(_Vddmu, res);
            return res;
        }

        inline SpMat joint_covariance(){
            SpMat joint_covariance(_joint_size, _joint_size);
            joint_covariance.setZero();
            _block.fill(_covariance, joint_covariance);
            return joint_covariance;
        }

        inline VectorXd joint_mean(){
            VectorXd joint_mean(_joint_size);
            joint_mean.setZero();
            _block.fill_vector(_mu, joint_mean);
            return joint_mean;
        }

        /**
         * @brief Get the mapping matrix Pk
         */
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
        inline MatrixXd negative_log_probability(const VectorXd& x) const{
            return _func_phi(x);
        }

        /**
         * @brief returns the (x-mu)*Phi(x) 
         */
        inline MatrixXd xMu_negative_log_probability(const VectorXd& x) const{
            return _func_Vmu(x);
        }

        /**
         * @brief returns the (x-mu)*Phi(x) 
         */
        inline MatrixXd xMuxMuT_negative_log_probability(const VectorXd& x) const{
            return _func_Vmumu(x);
        }

        /**
         * @brief returns the E_q{phi(x)} = E_q{-log(p(x,z))}
         */
        inline double E_Phi() {
            return _gh->Integrate(_func_phi)(0, 0);
        }

        inline MatrixXd E_xMuPhi(){
            return _gh->Integrate(_func_Vmu);
        }

        inline MatrixXd E_xMuxMuTPhi(){
            return _gh->Integrate(_func_Vmumu);
        }

        void set_GH_points(int p){
            _gh->set_polynomial_deg(p);
        }

        virtual double temperature() const { return _temperature; }

        virtual double high_temperature() const { return _high_temperature; }

        void switch_to_high_temperature(){
            _temperature = _high_temperature;
        }

        /// Public members for the inherited classes access
    public:

        bool _is_linear;

        /// dimension
        int _dim, _state_dim, _num_states, _joint_size;

        VectorXd _mu;
        
        /// Intermediate functions for Gauss-Hermite quadratures, default definition, needed to be overrided by the
        /// derived classes.
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;
        GHFunction _func_phi;
        GHFunction _func_Vmu;
        GHFunction _func_Vmumu;

        /// G-H quadrature class
        using GH = GaussHermite<GHFunction> ;
        std::shared_ptr<GH> _gh;

    protected:
        /// intermediate variables in optimization steps
        VectorXd _Vdmu;
        MatrixXd _Vddmu;

        /// optimization variables
        MatrixXd _precision, _dprecision;
        MatrixXd _covariance;

        // Sparse inverser and matrix helpers
        EigenWrapper _ei;

    protected:
        /// step sizes
        double _step_size_mu = 0.9;
        double _step_size_Sigma = 0.9;

        double _temperature, _high_temperature;
        
        // sparse mapping to sub variables
        TrajectoryBlock _block;

        MatrixXd _Pk;
        
    };

}