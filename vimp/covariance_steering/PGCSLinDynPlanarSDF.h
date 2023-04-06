/**
 * @file PGCSLinDynPlanarSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient algorithm for nonlinear covariance steering with plannar obstacles, linear dynamics. 
 * @version 0.1
 * @date 2023-03-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ProximalGradientCSLinearDyn.h"
#include "../robots/PlanarPointRobotSDF_pgcs.h"
#include "../helpers/hinge2Dhelper.h"

using namespace Eigen;

namespace vimp{

class PGCSLinDynPlanarSDF: public ProxGradCovSteerLinDyn{
public:
    PGCSLinDynPlanarSDF(const MatrixXd& A0, 
                        const VectorXd& a0, 
                        const MatrixXd& B, 
                        double sig,
                        int nt,
                        double eta,
                        double eps,
                        const VectorXd& z0,
                        const MatrixXd& Sig0,
                        const VectorXd& zT,
                        const MatrixXd& SigT,
                        const std::shared_ptr<LinearDynamics>& pdyn,
                        double eps_sdf,
                        const gpmp2::PlanarSDF& sdf,
                        double sig_obs,
                        double Vscale=1.0):
                            ProxGradCovSteerLinDyn(A0, a0, B, sig, nt, eta, eps, z0, Sig0, zT, SigT, pdyn, Vscale),
                            _eps_sdf(eps_sdf),
                            _sdf(sdf),
                            _invSig_obs(1.0 / sig_obs),
                            _pRsdf(eps_sdf){
                                _pRsdf.update_sdf(sdf);
                            }


    void update_Qrk() override{
        MatrixXd Aki(_nx, _nx), Bi(_nx, _nu), pinvBBTi(_nx, _nx), aki(_nx, 1), 
                 hAi(_nx, _nx), hai(_nx, 1),
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
            pinvBBTi = _ei.decompress3d(_pinvBBTt, _nx, _nx, i);
            zi = _ei.decompress3d(_zkt, _nx, 1, i);
            temp = (Aki - hAi).transpose();

            // Compute hinge loss and its gradients
            
            double zi_x = zi(0), zi_y = zi(1);
            std::pair<double, VectorXd> hingeloss_gradient;
            MatrixXd J_hxy(1, _nx/2);

            hingeloss_gradient = hingeloss_gradient_point(zi_x, zi_y, _sdf, _eps_sdf, J_hxy);
            double hinge = std::get<0>(hingeloss_gradient);

            if (hinge > 0){
                _ei.print_matrix(zi, "zi");
                std::cout << "hinge loss " << hinge << std::endl;
                _ei.print_matrix(J_hxy, "J_hxy");
            }

            MatrixXd grad_h(_nx, 1), velocity(_nx/2, 1);
            // grad_h << J_hxy(0), J_hxy(1), J_hxy(0) * zi(2), J_hxy(1) * zi(3);

            velocity = zi.block(_nx/2, 0,_nx/2,1);
            grad_h.block(0,0,_nx/2,1) = J_hxy.transpose();
            grad_h.block(_nx/2,0,_nx/2,1) = J_hxy.transpose().cwiseProduct(velocity);

            MatrixXd Hess(_nx, _nx);
            Hess.setZero();
            // if (hinge > 0){
            //     Hess.block(0, 0, _nx / 2, _nx / 2) = MatrixXd::Identity(_nx / 2, _nx / 2) * _invSig_obs;
            // }
            // Qki
            Qki = _state_cost_scale * Hess * _eta / (1+_eta) + temp * pinvBBTi * (Aki - hAi) * _eta / (1+_eta) / (1+_eta);
            // rki
            rki = _state_cost_scale * grad_h * hinge * _invSig_obs * _eta / (1.0 + _eta) +  temp * pinvBBTi * (aki - hai) * _eta / (1+_eta) / (1+_eta);

            // update Qkt, rkt
            _ei.compress3d(Qki, _Qkt, i);
            _ei.compress3d(rki, _rkt, i);
        }
        // _ei.print_matrix(_Qkt, "_Qkt");
        // _ei.print_matrix(_rkt, "_rkt");
        
    }

protected:
    gpmp2::PlanarSDF _sdf;
    PlanarPointRobotSDFPGCS _pRsdf;
    double _eps_sdf;
    double _invSig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    // TODO: For simple 2D it's 1d (1 ball). Needs to be extended to multiple ball checking cases.
};
}