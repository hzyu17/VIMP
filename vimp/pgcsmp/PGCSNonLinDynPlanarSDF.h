/**
 * @file PGCSNonLinDynPlanarSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient algorithm for nonlinear covariance steering with plannar obstacles. 
 * @version 0.1
 * @date 2023-03-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ProximalGradientCSNonlinearDyn.h"
#include "../helpers/hinge2Dhelper.h"
#include "../helpers/ExperimentParams.h"

using namespace Eigen;

namespace vimp{

class PGCSNonLinDynPlanarSDF: public ProxGradCovSteerNLDyn{
public:
    PGCSNonLinDynPlanarSDF(const MatrixXd& A0, 
                        const VectorXd& a0, 
                        const MatrixXd& B, 
                        const ExperimentParams& params,
                        const std::shared_ptr<NonlinearDynamics>& pdyn,
                        const gpmp2::PlanarSDF& sdf): ProxGradCovSteerNLDyn(A0, a0, B, params, pdyn),
                                            _eps_sdf(params.eps_sdf()),
                                            _sdf(sdf),
                                            _Sig_obs(params.sig_obs()){}

    PGCSNonLinDynPlanarSDF(const MatrixXd& A0, 
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
                    const std::shared_ptr<NonlinearDynamics>& pdyn,
                    double eps_sdf,
                    const gpmp2::PlanarSDF& sdf,
                    double sig_obs,
                    int max_iter=20): ProxGradCovSteerNLDyn(A0, a0, B, sig, nt, eta, eps, z0, Sig0, zT, SigT, pdyn, max_iter),
                                        _eps_sdf(eps_sdf),
                                        _sdf(sdf),
                                        _Sig_obs(sig_obs){}


    void update_Qrk() override{
        MatrixXd Aki(_nx, _nx), Bi(_nx, _nu), pinvBBTi(_nx, _nx), aki(_nx, 1), 
                 hAi(_nx, _nx), hai(_nx, 1), nTri(_nx, 1), 
                 Qti(_nx, _nx), Qki(_nx, _nx), rki(_nx, 1), zi(_nx, 1);
        MatrixXd temp(_nx, _nx);
        // for each time step
        _Qkt.setZero();
        _rkt.setZero();
        for (int i=0; i<_nt; i++){
            Aki = _ei.decomp3d(_Akt, _nx, _nx, i);
            aki = _ei.decomp3d(_akt, _nx, 1, i);
            hAi = _ei.decomp3d(_hAkt, _nx, _nx, i);
            hai = _ei.decomp3d(_hakt, _nx, 1, i);
            Bi = _ei.decomp3d(_Bt, _nx, _nu, i);
            Qti = _ei.decomp3d(_Qt, _nx, _nx, i);
            pinvBBTi = _ei.decomp3d(_pinvBBTt, _nx, _nx, i);
            nTri = _ei.decomp3d(_nTrt, _nx, 1, i);
            zi = _ei.decomp3d(_zkt, _nx, 1, i);
            temp = (Aki - hAi).transpose();

            // Compute hinge loss and its gradients
            double zi_x = zi(0), zi_y = zi(1);
            std::pair<double, VectorXd> hingeloss_gradient;
            MatrixXd J_hxy(1, _nx/2);

            hingeloss_gradient = hingeloss_gradient_point(zi_x, zi_y, _sdf, _eps_sdf, J_hxy);
            double hinge = std::get<0>(hingeloss_gradient);

            MatrixXd grad_h(_nx, 1), velocity(_nx/2, 1);
            // grad_h << J_hxy(0), J_hxy(1), J_hxy(0) * zi(2), J_hxy(1) * zi(3);

            velocity = zi.block(_nx/2, 0,_nx/2,1);
            grad_h.block(0,0,_nx/2,1) = J_hxy.transpose();
            grad_h.block(_nx/2,0,_nx/2,1) = J_hxy.transpose().cwiseProduct(velocity);
            MatrixXd Hess(_nx, _nx);
            Hess.setZero();
            // if (hinge > 0){
            //     Hess.block(0, 0, _nx / 2, _nx / 2) = MatrixXd::Identity(_nx / 2, _nx / 2) * _Sig_obs;
            // }
            // Qki
            Qki = temp * pinvBBTi * (Aki - hAi) * _eta / (1+_eta) / (1+_eta);
            // rki
            rki = nTri * _eta / (1+_eta) / 2 +  temp * pinvBBTi * (aki - hai) * _eta / (1+_eta) / (1+_eta);

            // update Qkt, rkt
            _ei.compress3d(Qki, _Qkt, i);
            _ei.compress3d(rki, _rkt, i);
        }
        
    }

    void step(int indx) override{
        // std::cout << "----- iter " << indx << " -----" << std::endl;
        // propagate the mean and the covariance
        propagate_mean();

        linearization();
 
        MatrixXd A_prior = _Akt / (1+_eta) + _hAkt * _eta / (1+_eta);
        MatrixXd a_prior = _akt / (1+_eta) + _hakt * _eta / (1+_eta);

        // Update Qkt, rkt
        this->update_Qrk();

        // solve inner loop 
        // solve_linearCS();

        // solve for the linear covariance steering
        _linear_cs.update_params(A_prior, _Bt, a_prior, _Qkt, _rkt);
        _linear_cs.solve();

        // retrieve (K, d)
        _Kt = _linear_cs.Kt();
        _dt = _linear_cs.dt();

        Matrix3D fbK(_nx, _nx, _nt), fbd(_nx, 1, _nt);
        MatrixXd Ai(_nx, _nx), ai(_nx, 1), Aprior_i(_nx, _nx), aprior_i(_nx, 1), Ki(_nu, _nx), fbKi(_nx, _nx), di(_nx, 1), fbdi(_nx, 1), Bi(_nx, _nu);
        for (int i=0; i<_nt; i++){
            Aprior_i = _ei.decomp3d(A_prior, _nx, _nx, i);
            aprior_i = _ei.decomp3d(a_prior, _nx, 1, i);

            Bi = _ei.decomp3d(_Bt, _nx, _nu, i);
            Ki = _ei.decomp3d(_Kt, _nu, _nx, i);
            di = _ei.decomp3d(_dt, _nu, 1, i);

            Ai = Aprior_i + Bi * Ki;
            ai = aprior_i + Bi * di;
            _ei.compress3d(Ai, _Akt, i);
            _ei.compress3d(ai, _akt, i);
        }
    }

protected:
    gpmp2::PlanarSDF _sdf;
    double _eps_sdf;
    double _Sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    // TODO: For simple 2D it's 1d (1 ball). Needs to be extended to multiple ball checking cases.
};
}