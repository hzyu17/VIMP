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
    ProxGradCovSteer(){};

    ProxGradCovSteer(MatrixXd A0, 
                     VectorXd a0, 
                     MatrixXd B, 
                     double sig,
                     int nt,
                     double eta,
                     double eps,
                     VectorXd z0,
                     MatrixXd Sig0,
                     VectorXd zT,
                     MatrixXd SigT): 
                     _ei(),
                     _nx(A0.rows()),
                     _nu(B.cols()),
                     _nt(nt),
                     _Akt(_ei.replicate3d(A0, _nt)),
                     _akt(_ei.replicate3d(a0, _nt)),
                     _Bt(_ei.replicate3d(B, _nt)),
                     _sig(sig),
                     _eps(eps),
                     _dt(_sig / _nt),
                     _Qkt(Eigen::MatrixXd::Zero(_nx*_nx, _nt)),
                     _Qt(Eigen::MatrixXd::Zero(_nx*_nx, _nt)),
                     _rkt(Eigen::MatrixXd::Zero(_nx, _nt)),
                     _hAkt(Eigen::MatrixXd::Zero(_nx*_nx, _nt)),
                     _hakt(Eigen::MatrixXd::Zero(_nx, _nt)),
                     _zkt(_ei.replicate3d(z0, _nt)),
                     _Sigkt(_ei.replicate3d(Sig0, _nt)),
                     _z0(z0),
                     _Sig0(Sig0),
                     _zT(zT),
                     _SigT(SigT),
                     _dyn(_nx, _nu, _nt),
                     _linear_cs(_Akt, _Bt, _akt, _nt, _eps, _Qkt, _rkt, _z0, _Sig0, _zT,_SigT)
                     {
                        std::cout << "debug0" << std::endl;
                        _BT = _Bt.transpose();
                        _pinvBBT = _Bt*_BT / _sig / _sig;
                        std::cout << "debug0" << std::endl;
                     }

    /**
     * @brief The optimization process, including linearization, 
     * sovling a linear CS, and push forward the mean and covariances.
     * @return std::tuple<MatrixXd, VectorXd> 
     */
    std::tuple<MatrixXd, VectorXd> optimize(){
        double stop_err = 1e-4, err = 1;
        MatrixXd Ak_prev(_nx*_nx, _nt), ak_prev(_nx, _nt);
        Ak_prev = _Akt;
        ak_prev = _akt;
        while (err > stop_err){
            step();
            err = (Ak_prev - _Akt).norm() / _Akt.norm() / _nt + (ak_prev - _akt).norm() / _akt.norm() / _nt;
            Ak_prev = _Akt;
            ak_prev = _akt;
        }

    }

    /**
     * @brief Solving a linear covariance steering at each iteration.
     * @return std::tuple<MatrixXd, VectorXd> representing (K, d).
     */
    std::tuple<MatrixXd, VectorXd> step(){
        propagate_mean(_Akt, _akt, _Bt);

        linearization();

        // In the case where pinv(BBT)==BBT.
        MatrixXd Aki(_nx, _nx), hAi(_nx, _nx), Bi(_nx, _nu), Qti(_nx, _nx), pinvBBTi(_nx, _nx), Qki(_nx, _nx), nTri(_nx, 1), zi(_nx, 1), rki(_nx, 1);
        for (int i=0; i<_nt; i++){
            Aki = _ei.decompress3d(_Akt, _nx, _nx, i);
            hAi = _ei.decompress3d(_hAkt, _nx, _nx, i);
            Bi = _ei.decompress3d(_Bt, _nx, _nu, i);
            Qti = _ei.decompress3d(_Qt, _nx, _nx, i);
            pinvBBTi = _ei.decompress3d(_pinvBBT, _nx, _nx, i);
            nTri = _ei.decompress3d(_nTr, _nx, 1, i);
            zi = _ei.decompress3d(_zkt, _nx, 1, i);

            Qki = 2 * _eta / (1 + _eta)* Qti + _eta / (1+_eta) / (1+_eta) * (Aki - hAi).transpose() * pinvBBTi * (Aki - hAi);
            rki = -_eta / (1 + _eta) * (Qti * zi) + _eta / (1+_eta) / 2 * nTri + _eta / (1+_eta) / (1+_eta) * (Aki - hAi).transpose() * pinvBBTi * (Aki - hAi);

            _ei.compress3d(Qki, _Qkt, i);
            _ei.compress3d(rki, _rkt, i);
        }
        
        
        MatrixXd Aprior(_nx, _nx);
        VectorXd aprior(_nx);

        Aprior = _Akt / (1+_eta) + _eta * _hAkt / (1+_eta);
        aprior = _akt / (1+_eta) + _eta * _hakt / (1+_eta);

        _linear_cs.update_params(Aprior, _Bt, aprior, _nt, _eps, _Qkt, _rkt, _z0, _Sig0, _zT, _SigT);
        _linear_cs.solve();
        
        _K = _linear_cs.Kt();
        _d = _linear_cs.dt();

        // propagate the mean and the covariance
        MatrixXd fbK(_nx*_nx, _nt);
        MatrixXd fbd(_nx, _nt);

        MatrixXd Ki(_nu, _nx), fbKi(_nx, _nx), di(_nx, 1), fbdi(_nx, 1);
        for (int i=0; i<_nt; i++){
            Bi = _ei.decompress3d(_Bt, _nx, _nu, i);
            Ki = _ei.decompress3d(_K, _nu, _nx, i);
            di = _ei.decompress3d(_d, _nu, 1, i);
            fbKi = Bi * Ki;
            fbdi = Bi * di;
            _ei.compress3d(fbKi, fbK, i);
            _ei.compress3d(fbdi, fbd, i);
        }

        _Akt = (_Akt + _eta * _hAkt) / (1 + _eta) + fbK;
        _akt = (_akt + _eta * _hakt) / (1 + _eta) + fbd;

    }    

    MatrixXd zk(){
        return _zkt;
    }

    MatrixXd Sigk(){
        return _Sigkt;
    }

    MatrixXd Ak(){
        return _Akt;
    }

    MatrixXd ak(){
        return _akt;
    }

    /**
     * @brief replicating a fixed state cost
     */
    void repliacteQt(MatrixXd Q){
        _Qt = _ei.replicate3d(Q, _nt);
    }

    /**
     * @brief linearization
     */
    void linearization(){
        std::tuple<MatrixXd, MatrixXd, MatrixXd, MatrixXd> linearized = _dyn.linearize(_zkt, _sig, _Akt, _Sigkt);
        _hAkt = std::get<0>(linearized);
        _Bt = std::get<1>(linearized);
        _hakt = std::get<2>(linearized);
        _nTr = std::get<3>(linearized);
    }

    void propagate_mean(MatrixXd At, MatrixXd at, MatrixXd Bt){
        // The i_th matrices
        Eigen::VectorXd zi(_nx), znew(_nx), ai(_nx);
        Eigen::MatrixXd Ai(_nx, _nx), Bi(_nx, _nu);
        Eigen::MatrixXd Si(_nx, _nx), Snew(_nx, _nx);
        for (int i=0; i<_nt-1; i++){
            zi = _ei.decompress3d(_zkt, _nx, 1, i);
            Ai = _ei.decompress3d(At, _nx, _nx, i);
            ai = _ei.decompress3d(at, _nx, 1, i);
            Bi = _ei.decompress3d(Bt, _nx, _nu, i);
            Si = _ei.decompress3d(_Sigkt, _nx, _nx, i);

            znew = zi + _dt*(zi + ai);
            Snew = Si + _dt*(Ai*Si + Si*Ai.transpose() + _eps*(Bi*Bi.transpose()));

            _ei.compress3d(znew, _zkt, i+1);
            _ei.compress3d(Snew, _Sigkt, i+1);
        }
    }

private:
    EigenWrapper _ei;
    int _nx, _nu, _nt;
    double _eta, _sig, _eps, _dt;

    // All the variables are time variant (3d matrices)
    // iteration variables
    MatrixXd _Akt, _Bt, _akt, _BT, _pinvBBT;
    MatrixXd _Qkt, _Qt; // Qk is the Q in each iteration, and Qt is the quadratic state cost matrix.
    MatrixXd _rkt;

    // linearizations
    MatrixXd _hAkt, _hakt, _nTr;

    // boundary conditions
    MatrixXd _Sigkt, _Sig0, _SigT;
    VectorXd _z0, _zT;
    MatrixXd _zkt;

    // Final result
    MatrixXd _K;
    MatrixXd _d;

    // Dynamics class
    NonlinearDynamics _dyn;
    LinearCovarianceSteering _linear_cs;
    
};
}