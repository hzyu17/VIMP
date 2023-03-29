/**
 * @file PGCSPlannarSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient algorithm for nonlinear covariance steering with plannar obstacles. 
 * @version 0.1
 * @date 2023-03-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ProximalGradientCS.h"
#include "../helpers/hinge2Dhelper.h"

using namespace Eigen;

namespace vimp{

class PGCSPlannarSDF: public ProxGradCovSteer{
public:
    PGCSPlannarSDF(MatrixXd A0, 
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
                     std::shared_ptr<NonlinearDynamics> pdyn,
                     double eps_sdf,
                     gpmp2::PlanarSDF sdf,
                     double sig_obs,
                     double Vscale=1.0):
                            ProxGradCovSteer(A0, a0, B, sig, nt, eta, eps, z0, Sig0, zT, SigT, pdyn, Vscale),
                            _eps_sdf(eps_sdf),
                            _sdf(sdf),
                            _invSig_obs(1.0 / sig_obs){}


    void update_Qrk() override{
        MatrixXd Aki(_nx, _nx), Bi(_nx, _nu), pinvBBTi(_nx, _nx), aki(_nx, 1), 
                 hAi(_nx, _nx), hai(_nx, 1), nTri(_nx, 1), 
                 Qti(_nx, _nx), Qki(_nx, _nx), rki(_nx, 1), zi(_nx, 1);
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

            // Compute hinge loss and its gradients
            double zi_x = zi(0), zi_y = zi(1);
            std::pair<double, VectorXd> hingeloss_gradient;
            MatrixXd J_hxy(1, 2);
            hingeloss_gradient = hingeloss_gradient_point(zi_x, zi_y, _sdf, _eps_sdf, J_hxy);
            double hinge = std::get<0>(hingeloss_gradient);
            Vector4d grad_h;
            grad_h << J_hxy(0), J_hxy(1), J_hxy(0) * zi(2), J_hxy(1) * zi(3);

            MatrixXd Hess(_nx, _nx);
            Hess.setZero();
            // if (hinge > 0){
            //     Hess.block(0, 0, _nx / 2, _nx / 2) = MatrixXd::Identity(_nx / 2, _nx / 2) * _invSig_obs;
            // }
            // Qki
            Qki = _state_cost_scale * Hess * _eta / (1+_eta) + temp * pinvBBTi * (Aki - hAi) * _eta / (1+_eta) / (1+_eta);
            // rki
            rki = _state_cost_scale * grad_h * hinge * _invSig_obs * _eta / (1.0 + _eta) +  nTri * _eta / (1+_eta) / 2 +  temp * pinvBBTi * (aki - hai) * _eta / (1+_eta) / (1+_eta);

            // update Qkt, rkt
            _ei.compress3d(Qki, _Qkt, i);
            _ei.compress3d(rki, _rkt, i);
        }
        
    }

private:
    gpmp2::PlanarSDF _sdf;
    double _eps_sdf;
    double _invSig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    // TODO: For simple 2D it's 1d (1 ball). Needs to be extended to multiple ball checking cases.
};
}