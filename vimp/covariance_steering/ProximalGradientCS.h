/**
 * @file ProxGradCovSteerLinDyn.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient algorithm for nonlinear covariance steering. 
 * @version 0.1
 * @date 2023-03-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../robots/NonlinearDynamics.h"

using namespace Eigen;

namespace vimp{

class ProxGradCovSteer{
public:
    ProxGradCovSteer(){};

    ProxGradCovSteer(const MatrixXd& A0, 
                     const VectorXd& a0, 
                     const MatrixXd& B, 
                     double sig,
                     int nt,
                     double eta,
                     const VectorXd& z0,
                     const MatrixXd& Sig0,
                     const VectorXd& zT,
                     const MatrixXd& SigT,
                     const NonlinearDynamics& dyn): 
                     _nx(A0.rows()),
                     _nu(B.cols()),
                     _nt(nt),
                     _Ak(A0),
                     _ak(a0),
                     _B(B),
                     _sig(sig),
                     _Qk(Eigen::MatrixXd::Zero(_nx, _nx)),
                     _rk(Eigen::VectorXd::Zero(_nx)),
                     _hAk(Eigen::MatrixXd::Zero(_nx, _nx)),
                     _hak(Eigen::VectorXd::Zero(_nx)),
                     _zk(z0),
                     _Sigk(Sig0),
                     _z0(z0),
                     _Sig0(Sig0),
                     _zT(zT),
                     _SigT(SigT),
                     _dyn(dyn)
                     {}

    /**
     * @brief The optimization process, including linearization, 
     * sovling a linear CS, and push forward the mean and covariances.
     * @return std::tuple<MatrixXd, VectorXd> 
     */
    std::tuple<MatrixXd, VectorXd> optimize(){
        int num_iter = 20;
        for (int i=0; i<num_iter, i++){
            std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearization = _dyn.Linearize(_zk, _sig, _Ak, _Sigk);
            _hAk = std::get<0>(linearization);
            _B = std::get<1>(linearization);
            _hak = std::get<2>(linearization);
            _nTr = std::get<3>(linearization);

            // In the case where pinv(BBT)==BBT.
            MatrixXd BT = B.transpose();
            MatrixXd pinv_BBT = B*BT / _sig / _sig;
            MatrixXd diffA_T = (_Ak - _hAk).transpose();
            _Qk = _eta / (1+_eta)^2 * diffA_T * pinv_BBT * (_Ak - _hAk);
            _rk = _eta / (2+2*_eta) * _nTr + _eta / (1+_eta)^2 * diffA_T * pinv_BBT * (_Ak - _hAk);


            
        }
        
    }

    /**
     * @brief Solving a linear covariance steering at each iteration.
     * @return std::tuple<MatrixXd, VectorXd> representing (K, d).
     */
    std::tuple<MatrixXd, VectorXd> iterate(){
        

        

    }    

private:
    int _nx, _nu, _nt;
    double _eta, _sig;

    // iteration variables
    MatrixXd _Ak, _B;
    VectorXd _ak;
    MatrixXd _Qk;
    VectorXd _rk;

    // linearizations
    MatrixXd _hAk;
    VectorXd _hak;
    VectorXd _nTr;

    // boundary conditions
    MatrixXd _Sigk, _Sig0, _SigT;
    VectorXd _zk, _z0, _zT;

    // Final result
    MatrixXd _K;
    VectorXd _d;

    // Dynamics class
    NonlinearDynamics _dyn;
    
};
}