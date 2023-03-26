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
#include "LinearCovarianceSteering.h"
#include "../helpers/eigen_wrapper.h"

using namespace Eigen;

namespace vimp{

class ProxGradCovSteer{
public:
    ProxGradCovSteer(){}

    ProxGradCovSteer(MatrixXd A0, 
                     VectorXd a0, 
                     MatrixXd B, 
                     double sig,
                     int nt,
                     double eta,
                     double epsilon,
                     VectorXd z0,
                     MatrixXd Sig0,
                     VectorXd zT,
                     MatrixXd SigT): 
                     _ei(),
                     _nx(A0.rows()),
                     _nu(B.cols()),
                     _nt(nt),
                     _Ak(_ei.replicate3d(A0, _nt)),
                     _ak(_ei.replicate3d(a0, _nt)),
                     _B(_ei.replicate3d(B, _nt)),
                     _sig(sig),
                     _epsilon(epsilon),
                     _dt(_sig / _nt),
                     _Qk(Eigen::MatrixXd::Zero(_nx*_nx, _nt)),
                     _rk(Eigen::VectorXd::Zero(_nx, _nt)),
                     _hAk(Eigen::MatrixXd::Zero(_nx*_nx, _nt)),
                     _hak(Eigen::VectorXd::Zero(_nx, _nt)),
                     _zk(_ei.replicate3d(z0, _nt)),
                     _Sigk(_ei.replicate3d(Sig0, _nt)),
                     _z0(_ei.replicate3d(z0, _nt)),
                     _Sig0(_ei.replicate3d(Sig0, _nt)),
                     _zT(_ei.replicate3d(zT, _nt)),
                     _SigT(_ei.replicate3d(SigT, _nt)),
                     _dyn(_nx, _nu, _nt),
                     _linear_cs(_Ak, _B, _ak, _nt, _epsilon, _Qk, _rk, _z0, _Sig0, _zT,_SigT)
                     {
                        _BT = _B.transpose();
                        _pinvBBT = _B*_BT / _sig / _sig;
                     }

    /**
     * @brief The optimization process, including linearization, 
     * sovling a linear CS, and push forward the mean and covariances.
     * @return std::tuple<MatrixXd, VectorXd> 
     */
    std::tuple<MatrixXd, VectorXd> optimize(){
        int num_iter = 20;
        for (int i=0; i<num_iter; i++){
            step();
        }
    }

    /**
     * @brief Solving a linear covariance steering at each iteration.
     * @return std::tuple<MatrixXd, VectorXd> representing (K, d).
     */
    std::tuple<MatrixXd, VectorXd> step(){
        linearization();

        // In the case where pinv(BBT)==BBT.
        MatrixXd diffA_T = (_Ak - _hAk).transpose();
        _Qk = _eta / (1+_eta) / (1+_eta) * diffA_T * _pinvBBT * (_Ak - _hAk);
        _rk = _eta / (1+_eta) / 2 * _nTr + _eta / (1+_eta) / (1+_eta) * diffA_T * _pinvBBT * (_Ak - _hAk);

        MatrixXd A(_nx, _nx);
        MatrixXd a(_nx);

        A = _Ak / (1+_eta) + _eta * _hAk / (1+_eta);
        a = _ak / (1+_eta) + _eta * _hak / (1+_eta);

        _linear_cs = LinearCovarianceSteering(A, _B, a, _nt, _epsilon, _Qk, _rk, _z0, _Sig0, _zT, _SigT);
        _linear_cs.solve();
        
        _K = _linear_cs.Kt();
        _d = _linear_cs.dt();

        // propagate the mean and the covariance
        _Ak = (_Ak + _eta * _hAk) / (1 + _eta) + _B * _K;
        _ak = (_ak + _eta * _hak) / (1 + _eta) + _B * _d;

        _zk = _zk + (_Ak * _zk + _ak) * _dt;
        _Sigk = _Sigk + (_Ak * _Sigk + _Sigk*_Ak.transpose() + _sig * _B*_BT) * _dt;

    }    

    /**
     * @brief linearization
     */
    void linearization(){
        std::tuple<MatrixXd, MatrixXd, MatrixXd, MatrixXd> linearized = _dyn.linearize(_zk, _sig, _Ak, _Sigk);
        _hAk = std::get<0>(linearized);
        _B = std::get<1>(linearized);
        _hak = std::get<2>(linearized);
        _nTr = std::get<3>(linearized);
    }

private:
    EigenWrapper _ei;
    int _nx, _nu, _nt;
    double _eta, _sig, _epsilon, _dt;

    // iteration variables
    MatrixXd _Ak, _B, _BT, _pinvBBT;
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
    LinearCovarianceSteering _linear_cs;
    
};
}