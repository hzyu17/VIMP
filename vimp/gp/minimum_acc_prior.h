/**
 * @file minimum_acc.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief minimum acceleration gp model, which is a linear model of the form
 * -log(p(x|z)) = C*||A*x - B*\mu_t||_{\Sigma^{-1}}.
 * @version 0.1
 * @date 2022-07-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */


/**
 * @brief The model A(t) = [0 I; 0 0], u(t)=0, F(t)=[0; I], 
 * Phi(t,s)=[I (t-s)I; 0 I], Q_{i,i+1}=[1/3*(dt)^3Qc 1/2*(dt)^2Qc; 1/2*(dt)^2Qc (dt)Qc]
 * x and v share one same Qc.
 */

#include "linear_factor.h"
#include "helpers/EigenWrapper.h"

namespace vimp{
    class MinimumAccGP : public LinearFactor{
        public: 
            MinimumAccGP(){};
            /**
             * @brief the state is [x; v] where the dimension of x is _dim. 
             * The returned mean is the concatenation of the two consecutive [\mu_i, \mu_{i+1}].
             * The constant velocity linear factor has closed-forms in transition matrix \Phi,
             * the matrices in computing the quadratic costs, (\Lambda, \Psi).
             * Qc is in shape (_dim, _dim); 
             * _Phi and _Q are in shape(2*_dim, 2*_dim)
             * @param Qc 
             * @param delta_t 
             */
            MinimumAccGP(const MatrixXd& Qc, double start_index, const double& delta_t, const VectorXd& mu_0): 
            LinearFactor(),
            _dim{Qc.cols()},
            _dim_state{2*_dim},
            _start_index{start_index},
            _m0{mu_0},
            _target_mu{VectorXd::Zero(2*_dim_state)},
            _delta_t{delta_t}, 
            _Qc{Qc}, 
            _invQc{Qc.inverse()}, 
            _invQ{MatrixXd::Zero(_dim_state, _dim_state)},
            _Phi{_dim_state, _dim_state}{
                _Phi.setZero();
                _Phi << MatrixXd::Identity(_dim, _dim), delta_t*MatrixXd::Identity(_dim, _dim), 
                           MatrixXd::Zero(_dim, _dim), MatrixXd::Identity(_dim, _dim);

                // compute the concatenation [\mu_i, \mu_{i+1}]
                MatrixXd Phi_i(_dim_state, _dim_state);
                Phi_i.setZero();
                Phi_i << MatrixXd::Identity(_dim, _dim), _start_index*delta_t*MatrixXd::Identity(_dim, _dim), 
                         MatrixXd::Zero(_dim, _dim), MatrixXd::Identity(_dim, _dim);

                VectorXd mi = Phi_i * _m0;
                VectorXd mi_next = _Phi * mi;

                _target_mu.segment(0, _dim_state) = mi;
                _target_mu.segment(_dim_state, _dim_state) = mi_next;
                
                _Q = MatrixXd::Zero(_dim_state, _dim_state);
                _Q << _Qc*pow(_delta_t, 3)/3, _Qc*pow(_delta_t, 2)/2, _Qc*pow(_delta_t, 2)/2, Qc*_delta_t;
                compute_invQ();

                // \Lambda = [-\Phi, I], \Psi = [\Phi, -I]
                _Lambda = MatrixXd::Zero(_dim_state, 2*_dim_state);
                _Lambda.block(0, 0, _dim_state, _dim_state) = -_Phi;
                _Lambda.block(0, _dim_state, _dim_state, _dim_state) = MatrixXd::Identity(_dim_state, _dim_state);

                // When a(t)=0, this part is eliminated.
                _Psi = MatrixXd::Zero(_dim_state, 2*_dim_state);
                _Psi.block(0, 0, _dim_state, _dim_state) = _Phi;
                _Psi.block(0, _dim_state, _dim_state, _dim_state) = -MatrixXd::Identity(_dim_state, _dim_state);
            }

        private:
            int _dim, _start_index;
            int _dim_state;
            double _delta_t;
            MatrixXd _Qc, _invQc, _Q, _invQ;
            MatrixXd _Phi, _Lambda, _Psi;
            VectorXd _m0, _target_mu;
            EigenWrapper _ei;
            
        public:
            inline MatrixXd Q() const { return _Q; }

            inline MatrixXd Qc() const { return _Qc; }

            inline MatrixXd Phi() const { return _Phi; }

            /**
             * @brief the cost function
             * @param theta1 [x1; v1]
             * @param theta2 [x2; v2]
             */
            inline double cost(const VectorXd& theta1, const VectorXd& theta2) const{
                double cost = (_Phi*theta1-theta2).transpose()* _invQ * (_Phi*theta1-theta2);
                return cost / 2;
            }

            inline int dim_posvel() const { return 2*_dim; }

            inline void compute_invQ() {
                _invQ = MatrixXd::Zero(2*_dim, 2*_dim);
                _invQ.block(0, 0, _dim, _dim) = 12 * _invQc / pow(_delta_t, 3);
                _invQ.block(0, _dim, _dim, _dim) = -6 * _invQc / pow(_delta_t, 2);
                _invQ.block(_dim, 0, _dim, _dim) = -6 * _invQc / pow(_delta_t, 2);
                _invQ.block(_dim, _dim, _dim, _dim) = 4 * _invQc / _delta_t;
            }

            inline VectorXd get_mu() const { return _target_mu; }

            inline MatrixXd get_precision() const{ return _invQ; }

            inline MatrixXd get_covariance() const{ return _invQ.inverse(); }

            inline MatrixXd get_Lambda() const{ return _Lambda; }

            inline MatrixXd get_Psi() const{ return _Psi; }

            inline double get_C() const{ return 0.5;}
            
    };

} //namespace vimp
