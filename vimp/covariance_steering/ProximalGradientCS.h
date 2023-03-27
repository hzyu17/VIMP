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
#include <memory>
#include <Eigen/QR> 

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
                     MatrixXd SigT,
                     std::shared_ptr<NonlinearDynamics> pdyn): 
                     _ei(),
                     _nx(A0.rows()),
                     _nu(B.cols()),
                     _nt(nt),
                     _eta(eta),
                     _Akt(_ei.replicate3d(A0, _nt)),
                     _akt(_ei.replicate3d(a0, _nt)),
                     _Bt(_ei.replicate3d(B, _nt)),
                     _sig(sig),
                     _eps(eps),
                     _deltt(1.0 / (_nt-1)),
                     _Qkt(Eigen::MatrixXd::Zero(_nx*_nx, _nt)),
                     _Qt(Eigen::MatrixXd::Zero(_nx*_nx, _nt)),
                     _rkt(Eigen::MatrixXd::Zero(_nx, _nt)),
                     _hAkt(Eigen::MatrixXd::Zero(_nx*_nx, _nt)),
                     _hakt(Eigen::MatrixXd::Zero(_nx, _nt)),
                     _nTrt(Eigen::MatrixXd::Zero(_nx, _nt)),
                     _pinvBBT(MatrixXd::Zero(_nx*_nx, _nt)),
                     _zkt(_ei.replicate3d(z0, _nt)),
                     _Sigkt(_ei.replicate3d(Sig0, _nt)),
                     _z0(z0),
                     _Sig0(Sig0),
                     _zT(zT),
                     _SigT(SigT),
                     _dynptr{pdyn},
                     _K(_nu*_nx, _nt),
                     _d(_nu, _nt),
                     _linear_cs(_Akt, _Bt, _akt, _nx, _nu, _nt, _eps, _Qkt, _rkt, _z0, _Sig0, _zT, _SigT)
                     {
                        MatrixXd Bi(_nx, _nu), BiT(_nu, _nx), pinvBBTi(_nx, _nx);
                        for (int i=0; i<_nt; i++){
                            Bi = _ei.decompress3d(_Bt, _nx, _nu, i);
                            BiT = Bi.transpose();
                            pinvBBTi = (Bi * BiT).completeOrthogonalDecomposition().pseudoInverse();;
                            _ei.compress3d(pinvBBTi, _pinvBBT, i);
                        }
                     }

    /**
     * @brief The optimization process, including linearization, 
     * sovling a linear CS, and push forward the mean and covariances.
     * @return std::tuple<MatrixXd, MatrixXd>  representing (Kt, dt)
     */
    std::tuple<MatrixXd, MatrixXd> optimize(){
        double stop_err = 1e-4, err = 1;
        MatrixXd Ak_prev(_nx*_nx, _nt), ak_prev(_nx, _nt);
        Ak_prev = _Akt;
        ak_prev = _akt;
        int i_step = 0;
        while (err > stop_err){
            step(i_step);
            err = (Ak_prev - _Akt).norm() / _Akt.norm() / _nt + (ak_prev - _akt).norm() / _akt.norm() / _nt;
            Ak_prev = _Akt;
            ak_prev = _akt;
            i_step ++;
        }

        return std::make_tuple(_K, _d);
    }

    /**
     * @brief Solving a linear covariance steering at each iteration.
     * @return none, but inside already compute (K, d).
     */
    void step(int indx){
        // propagate the mean and the covariance
        propagate_mean(_Akt, _akt, _Bt);
        linearization();

        MatrixXd Apriort(_nx*_nx, _nt), apriort(_nx, _nt);
        Apriort = _Akt / (1+_eta) + _hAkt * _eta / (1+_eta);
        apriort = _akt / (1+_eta) + _hakt * _eta / (1+_eta);

        // Update Qkt, rkt
        update_Qrk();

        // solve for the linear covariance steering
        _linear_cs.update_params(Apriort, _Bt, apriort, _nx, _nu, _nt, _eps, _Qkt, _rkt, _z0, _Sig0, _zT, _SigT);
        _linear_cs.solve();

        // retrieve (K, d)
        _K = _linear_cs.Kt();
        _d = _linear_cs.dt();

        MatrixXd fbK(_nx*_nx, _nt), fbd(_nx, _nt);
        MatrixXd Ai(_nx, _nx), ai(_nx, 1), Aprior_i(_nx, _nx), aprior_i(_nx, 1), Ki(_nu, _nx), fbKi(_nx, _nx), di(_nx, 1), fbdi(_nx, 1), Bi(_nx, _nu);
        for (int i=0; i<_nt; i++){
            Aprior_i = _ei.decompress3d(Apriort, _nx, _nx, i);
            aprior_i = _ei.decompress3d(apriort, _nx, 1, i);

            Bi = _ei.decompress3d(_Bt, _nx, _nu, i);
            Ki = _ei.decompress3d(_K, _nu, _nx, i);
            di = _ei.decompress3d(_d, _nu, 1, i);

            Ai = Aprior_i + Bi * Ki;
            ai = aprior_i + Bi * di;
            _ei.compress3d(Ai, _Akt, i);
            _ei.compress3d(ai, _akt, i);
        }
    }

    inline MatrixXd zkt(){ return _zkt; }

    inline MatrixXd Sigkt(){ return _Sigkt; }

    inline MatrixXd Akt(){ return _Akt; }

    inline MatrixXd akt(){ return _akt; }

    inline MatrixXd Qkt(){ return _Qkt; }

    inline MatrixXd rkt(){ return _rkt; }

    /**
     * @brief replicating a fixed state cost
     */
    void repliacteQt(MatrixXd Q0){
        _Qt = _ei.replicate3d(Q0, _nt);
    }

    void update_Qrk(){
        MatrixXd Aki(_nx, _nx), aki(_nx, 1), hAi(_nx, _nx), hai(_nx, 1), Bi(_nx, _nu), Qti(_nx, _nx), pinvBBTi(_nx, _nx), Qki(_nx, _nx), nTri(_nx, 1), zi(_nx, 1), rki(_nx, 1);
        MatrixXd temp(_nx, _nx);
        // for each time step
        _Qkt.setZero();
        _rkt.setZero();
        for (int i=0; i<_nt; i++){
            Aki = _ei.decompress3d(_Akt, _nx, _nx, i);
            aki = _ei.decompress3d(_akt, _nx, 1, i);
            hAi = _ei.decompress3d(_hAkt, _nx, _nx, i);
            hai = _ei.decompress3d(_hakt, _nx, 1, i);
            Bi = _ei.decompress3d(_Bt, _nx, _nu, i);
            Qti = _ei.decompress3d(_Qt, _nx, _nx, i);
            pinvBBTi = _ei.decompress3d(_pinvBBT, _nx, _nx, i);
            nTri = _ei.decompress3d(_nTrt, _nx, 1, i);
            zi = _ei.decompress3d(_zkt, _nx, 1, i);
            temp = (Aki - hAi).transpose();
            Qki = Qti * 2 * _eta / (1 + _eta)  + temp * pinvBBTi * (Aki - hAi) * _eta / (1+_eta) / (1+_eta);
            rki = - (Qti * zi) * _eta / (1 + _eta) +  nTri * _eta / (1+_eta) / 2 +  temp * pinvBBTi * (aki - hai) * _eta / (1+_eta) / (1+_eta);

            // update Qkt, rkt
            _ei.compress3d(Qki, _Qkt, i);
            _ei.compress3d(rki, _rkt, i);
        }
        
    }

    /**
     * @brief linearization
     */
    void linearization(){
        std::tuple<MatrixXd, MatrixXd, MatrixXd, MatrixXd> res;
        res = _dynptr->linearize(_zkt, _sig, _Akt, _Sigkt);
        _hAkt = std::get<0>(res);
        _Bt   = std::get<1>(res);
        _hakt = std::get<2>(res);
        _nTrt = std::get<3>(res);
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

            znew = zi + _deltt*(Ai*zi + ai);
            Snew = Si + _deltt*(Ai*Si + Si*Ai.transpose() + _eps*(Bi*Bi.transpose()));

            _ei.compress3d(znew, _zkt, i+1);
            _ei.compress3d(Snew, _Sigkt, i+1);
        }
    }

private:
    EigenWrapper _ei;
    int _nx, _nu, _nt;
    double _eta, _sig, _eps, _deltt;

    // All the variables are time variant (3d matrices)
    // iteration variables
    MatrixXd _Akt, _Bt, _akt, _pinvBBT;
    MatrixXd _Qkt, _Qt; // Qk is the Q in each iteration, and Qt is the quadratic state cost matrix.
    MatrixXd _rkt;

    // linearizations
    MatrixXd _hAkt, _hakt, _nTrt;

    // boundary conditions
    MatrixXd _Sigkt, _Sig0, _SigT;
    VectorXd _z0, _zT;
    MatrixXd _zkt;

    // Final result
    MatrixXd _K;
    MatrixXd _d;

    // Dynamics class
    std::shared_ptr<NonlinearDynamics> _dynptr;
    LinearCovarianceSteering _linear_cs;
    
};
}