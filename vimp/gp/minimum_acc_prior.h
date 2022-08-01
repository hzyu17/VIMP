/**
 * @file minimum_acc.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief minimum acceleration gp model
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

#include <Eigen/Dense>

using namespace Eigen;

namespace vimp{
    class MinimumAccGP{
        public: 
            MinimumAccGP(){};
            /**
             * @brief state: [x; v], _dim means the dimension of x. 
             * Qc is in shape (_dim, _dim), and 
             * _Phi and _Q in shape(2*_dim, 2*_dim)
             * @param Qc 
             * @param delta_t 
             */
            MinimumAccGP(const MatrixXd& Qc, const double& delta_t): _Qc{Qc}, _invQc{Qc.inverse()}, _delta_t{delta_t}, _dim{Qc.cols()} {
                _Phi = MatrixXd::Zero(2*_dim, 2*_dim);
                _Phi << MatrixXd::Identity(_dim, _dim), delta_t*MatrixXd::Identity(_dim, _dim), MatrixXd::Zero(_dim, _dim), MatrixXd::Identity(_dim, _dim);
                _Q = MatrixXd::Zero(2*_dim, 2*_dim);
                _Q << _Qc*pow(_delta_t, 3)/3, _Qc*pow(_delta_t, 2)/2, _Qc*pow(_delta_t, 2)/2, Qc*_delta_t;
            }

        private:
            MatrixXd _Qc;
            MatrixXd _invQc;
            MatrixXd _Phi;
            MatrixXd _Q;
            double _delta_t;
            int _dim;

        public:
            inline MatrixXd Q() const {
                return _Q;
            }

            inline MatrixXd Qc() const {
                return _Qc;
            }

            inline MatrixXd Phi() const {
                return _Phi;
            }

            inline double cost(const VectorXd& x1, const VectorXd& x2) const{
                assert(x1.size() == x2.size());
                assert(x1.size() == 2*_dim);
                double cost = (_Phi*x1-x2).transpose()* invQ() * (_Phi*x1-x2);
                return cost / 2;
            }

            inline int dim_posvel() const {
                return 2*_dim;
            }

            inline MatrixXd invQ() const {
                MatrixXd invQ{MatrixXd::Zero(2*_dim, 2*_dim)};

                invQ.block(0, 0, _dim, _dim) = 12 * _invQc / pow(_delta_t, 3);
                invQ.block(0, _dim, _dim, _dim) = -6 * _invQc / pow(_delta_t, 2);
                invQ.block(_dim, 0, _dim, _dim) = -6 * _invQc / pow(_delta_t, 2);
                invQ.block(_dim, _dim, _dim, _dim) = 4 * _invQc / _delta_t;

                return invQ;
            }
            
    };

} //namespace vimp
