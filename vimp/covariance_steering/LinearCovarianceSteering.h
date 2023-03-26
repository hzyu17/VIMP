/**
 * @file LinearCovarianceSteering.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Linear covariance steering problem. 
 * See Chen, Yongxin, Tryphon T. Georgiou, and Michele Pavon. 
 * "Optimal steering of a linear stochastic system to a final probability distribution, Part I." 
 * IEEE Transactions on Automatic Control 61.5 (2015): 1158-1169. 
 * @version 0.1
 * @date 2023-03-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <Eigen/Dense>
#include "../helpers/eigen_wrapper.h"
using namespace Eigen;

namespace vimp{
class LinearCovarianceSteering{

public:
    LinearCovarianceSteering(){}

/**
 * @brief Construct a new Linear Covariance Steering object
 * All time varying matrices should be in the shape (m*n, nt)
 */
    LinearCovarianceSteering(const MatrixXd& At, 
                             const MatrixXd& Bt,
                             const MatrixXd& at, 
                             const double& nt,
                             const double& epsilon,
                             const MatrixXd& Qt,
                             const MatrixXd& rt,
                             const VectorXd& m0,
                             const MatrixXd& Sig0,
                             const VectorXd& m1,
                             const MatrixXd& Sig1):
                             _At(At),
                             _Bt(Bt),
                             _at(at),
                             _rt(rt),
                             _Qt(Qt),
                             _nx(_Bt.rows()),
                             _nu(_Bt.cols()),
                             _nt(nt),
                             _Phi(MatrixXd::Identity(2*_nx, 2*_nx)),
                             _Phi11(MatrixXd::Zero(_nx, _nx)),
                             _Phi12(MatrixXd::Zero(_nx, _nx)),
                             _Mt(MatrixXd::Zero(2*_nx * 2*_nx, _nt)),
                             _Pi(MatrixXd::Zero(_nx*_nx, _nt)),
                             _m0(m0),
                             _m1(m1),
                             _Sig0(Sig0),
                             _Sig1(Sig1),
                             _eps(epsilon),
                             _delta_t(1.0/(nt-1)),
                             _Kt(MatrixXd::Zero(_nu*_nx, _nt)),
                             _dt(MatrixXd::Zero(_nu, _nt)){
        std::cout << "debug1" << std::endl;
        MatrixXd Ai(_nx, _nx), Bi(_nx, _nu), Qi(_nx, _nx), Mi(2*_nx, 2*_nx);
        for (int i=0; i< _nt; i++){
            Mi = MatrixXd::Zero(2*_nx, 2*_nx);
            Ai = _ei.decompress3d(_At, _nx, _nx, i);
            Qi = _ei.decompress3d(_Qt, _nx, _nx, i);
            Bi = _ei.decompress3d(_Bt, _nx, _nu, i);
            std::cout << "debug1" << std::endl;
            Mi.block(0, 0, _nx, _nx) = Ai;
            Mi.block(0, _nx, _nx, _nx) = -Bi*Bi.transpose();
            Mi.block(_nx, 0, _nx, _nx) = -Qi;
            Mi.block(_nx, _nx, _nx, _nx) = -Ai.transpose();
            std::cout << "debug1" << std::endl;
            _ei.compress3d(Mi, _Mt, i);
        }

        for (int i=0; i<_nt-1; i++){
            MatrixXd Mi = _ei.decompress3d(_Mt, 2*_nx, 2*_nx, i);
            _Phi = _Phi + Mi*_Phi*_delta_t;
        }

        _Phi11 = _Phi.block(0, 0, _nx, _nx);
        _Phi12 = _Phi.block(0, _nx, _nx, _nx);
        std::cout << "debug1" << std::endl;
    }

    /**
     * @brief print the matrices at time step i.
     */
    void print_matrix_i(int i){
        MatrixXd Ai, Mi, Qi, ai, ri;
        _ei.decompress3d(_At, Ai, _nx, _nx, i);
        _ei.decompress3d(_Qt, Qi, _nx, _nx, i);
        _ei.decompress3d(_Mt, Mi, 2*_nx, 2*_nx, i);
        std::cout << "Ai" << std::endl;
        _ei.print_matrix(Ai);

        std::cout << "ai" << std::endl;
        _ei.print_matrix(_at.col(i));

        std::cout << "Bi" << std::endl;
        _ei.print_matrix(_Bt);

        std::cout << "Qi" << std::endl;
        _ei.print_matrix(Qi);

        std::cout << "ri" << std::endl;
        _ei.print_matrix(_rt.col(i));

        std::cout << "Mi" << std::endl;
        _ei.print_matrix(Mi);
    }

    void update_params(MatrixXd A, MatrixXd B, VectorXd a, 
                        int nt, double eps, 
                        MatrixXd Q, VectorXd r, 
                        VectorXd m0, MatrixXd Sig0, VectorXd mT, MatrixXd SigT){
        _At = A;
        _Bt = B;
        _at = a;
        _nt = nt;
        _eps = eps;
        _Qt = Q;
        _rt = r;
        _m0 = m0;
        _Sig0 = Sig0;
        _m1 = mT;
        _Sig1 = SigT;
    }

    MatrixXd At(){
        return _At;
    }

    MatrixXd At(int i){
        MatrixXd Ai;
        _ei.decompress3d(_At, Ai, _nx, _nx, i);
        return Ai;
    }

    MatrixXd at(){
        return _at;
    }

    MatrixXd at(int i){
        return _at.col(i);
    }

    MatrixXd Bt(){
        return _Bt;
    }

    MatrixXd Bt(int i){
        MatrixXd Bi;
        _ei.decompress3d(_Bt, Bi, _nx, _nu, i);
        return Bi;
    }

    MatrixXd Qt(){
        return _Qt;
    }

    MatrixXd Qt(int i){       
        return _ei.decompress3d(_Qt, _nx, _nx, i);
    }

    MatrixXd rt(){
        return _rt;
    }

    MatrixXd rt(int i){
        return _rt.col(i);
    }

    MatrixXd Kt(){
        return _Kt;
    }

    MatrixXd Kt(int i){
        return _ei.decompress3d(_Kt, _nu, _nx, i);
    }

    MatrixXd dt(){
        return _dt;
    }

    MatrixXd Mt(){
        return _Mt;
    }

    MatrixXd Mt(int i){
        MatrixXd Mi{MatrixXd::Zero(2*_nx, 2*_nx)};
        _ei.decompress3d(_Mt, Mi, 2*_nx, 2*_nx, i);
        return Mi;
    }

    MatrixXd Phi(){
        return _Phi;
    }

    MatrixXd Phi11(){
        return _Phi11;
    }

    MatrixXd Phi12(){
        return _Phi12;
    }

    MatrixXd Pi(){
        return _Pi;
    }

    MatrixXd Pi(int i){
        return _ei.decompress3d(_Pi, _nx, _nx, i);
    }

    void solve(){

        VectorXd s{VectorXd::Zero(2*_nx)};
        MatrixXd a_r{MatrixXd::Zero(2*_nx, _nt)};
        a_r << _at, 
              -_rt;
        
        for (int i=0;i<_nt-1;i++){
            MatrixXd Mi = _ei.decompress3d(_Mt, 2*_nx, 2*_nx, i);
            s = s + (Mi*s + a_r.col(i))*_delta_t;
        }

        VectorXd rhs{_m1 - _Phi11*_m0-s.block(0,0,_nx,1)};
        VectorXd Lambda_0 = _Phi12.colPivHouseholderQr().solve(rhs);

        MatrixXd X(2*_nx, _nt);
        VectorXd X0(2*_nx);
        X0 << _m0, Lambda_0;
        X.col(0) = X0;
        for (int i=0; i<_nt-1; i++){
            MatrixXd Mi = _ei.decompress3d(_Mt, 2*_nx, 2*_nx, i); 
            X.col(i+1) = X.col(i) + _delta_t*(Mi*X.col(i) + a_r.col(i));
        }

        MatrixXd xt{X.block(0,0,_nx,_nt)};
        MatrixXd lbdt{X.block(_nx, 0, _nx, _nt)};
        MatrixXd v(_nu, _nt);

        for (int i=0; i<_nt; i++){
            v.col(i) = -_Bt.transpose()*lbdt.col(i);
        }

        MatrixXd Sig0_inv_sqrt = _ei.psd_invsqrtm(_Sig0);
        _ei.print_matrix(Sig0_inv_sqrt, "Sig0_inv_sqrt");
        
        MatrixXd Sig0_sqrt = _ei.psd_sqrtm(_Sig0);
        MatrixXd temp = _eps*_eps*MatrixXd::Identity(_nx, _nx)/4+
                        Sig0_sqrt*_Phi12.inverse()*_Sig1*_Phi12.transpose().inverse()*Sig0_sqrt;

        MatrixXd Pi_0 = _eps * _Sig0.inverse() / 2.0 - _Phi12.inverse()*_Phi11 - 
                        Sig0_inv_sqrt *_ei.psd_sqrtm(temp)*Sig0_inv_sqrt;
        
        MatrixXd Pi_0_T = Pi_0.transpose();
        Pi_0 = (Pi_0 + Pi_0_T)/2;
        _ei.compress3d(Pi_0, _Pi, 0);
        
        for (int i=0; i<_nt-1; i++){
            MatrixXd l_Pi = _ei.decompress3d(_Pi, _nx, _nx, i);
            MatrixXd Qi = _ei.decompress3d(_Qt, _nx, _nx, i);
            MatrixXd Ai = _ei.decompress3d(_At, _nx, _nx, i);

            MatrixXd Pi_i = l_Pi - _delta_t*(Ai.transpose()*l_Pi+l_Pi*Ai-l_Pi*_Bt*_Bt.transpose()*l_Pi+Qi);
            _ei.compress3d(Pi_i, _Pi, i+1);
        }

        for (int i=0; i<_nt; i++){
            MatrixXd Pi_i = _ei.decompress3d(_Pi, _nx, _nx, i);
            MatrixXd Ki = -_Bt.transpose() * Pi_i;
            _ei.compress3d(Ki, _Kt, i);
            _dt.col(i) = v.col(i) + _Bt.transpose() * Pi_i * xt.col(i);
        }
    }


private:
    MatrixXd _At, _Bt, _at;
    MatrixXd _Qt, _rt;

    int _nx, _nu, _nt;

    MatrixXd _Phi, _Phi11, _Phi12, _Mt, _Pi;

    VectorXd _m0, _m1;
    MatrixXd _Sig0, _Sig1;

    double _eps, _delta_t;

    // feedback gains
    MatrixXd _Kt;
    MatrixXd _dt;

    // helper
    vimp::EigenWrapper _ei;

};
}