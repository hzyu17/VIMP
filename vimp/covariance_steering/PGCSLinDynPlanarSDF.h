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
#include "robots/PlanarPointRobotSDF_pgcs.h"
#include "helpers/hinge2Dhelper.h"
#include "helpers/CostHelper.h"

using namespace Eigen;

namespace vimp{

class PGCSLinDynPlanarSDF: public ProxGradCovSteerLinDyn{
public:
    PGCSLinDynPlanarSDF(const MatrixXd& A0, 
                        const VectorXd& a0, 
                        const MatrixXd& B, 
                        const PGCSParams& params,
                        const std::shared_ptr<LinearDynamics>& pdyn,
                        const gpmp2::PlanarSDF & sdf):
                        ProxGradCovSteerLinDyn(A0, a0, B, pdyn, params),
                        _eps_sdf(params.eps_sdf(), params.radius()),
                        _sdf(sdf),
                        _sig_obs(params.sig_obs()),
                        _pRsdf(params.eps_sdf(), params.radius()),
                        _cost_helper(params.max_iter())
                        {
                            _pRsdf.update_sdf(sdf);
                        }


    double total_hingeloss(){
        double total_hingeloss = 0;
        std::pair<double, VectorXd> hingeloss_gradient;
        MatrixXd zi(_nx, 1);
        for (int i=0; i<_nt; i++){
            zi = _ei.decomp3d(_zkt, _nx, 1, i);
            double zi_x = zi(0), zi_y = zi(1);
            MatrixXd J_hxy(1, _nx/2);
            hingeloss_gradient = hingeloss_gradient_point(zi_x, zi_y, _sdf, _eps_sdf, J_hxy);
            double hinge = std::get<0>(hingeloss_gradient);
            total_hingeloss += hinge;
        }
        return total_hingeloss;
    }

    double total_control_energy(){
        double total_Eu = 0;
        MatrixXd zi(_nx, 1), Ki(_nu, _nx);
        for (int i=0; i<_nt; i++){
            zi = _ei.decomp3d(_zkt, _nx, 1, i);
            Ki = _ei.decomp3d(_Kt, _nu, _nx, i);
            VectorXd u_i = Ki * zi;
            double E_ui = u_i.dot(u_i) * _deltt;
            total_Eu += E_ui;
        }
        return total_Eu;
    }

    /**
     * @brief The optimization process, including recording the costs.
     * @return std::tuple<MatrixXd, MatrixXd>  representing (Kt, dt)
     */
    std::tuple<Matrix3D, Matrix3D> optimize() override{
        double err = 1;

        int i_step = 0;

        double total_cost_prev = this->total_hingeloss() + this->total_control_energy();
        double total_cost = 0.0, hingeloss = 0.0, control_energy=0.0;
        while ((err > _stop_err) && (i_step < _max_iter)){
            step(i_step);
            hingeloss = this->total_hingeloss();
            control_energy = this->total_control_energy();
            total_cost = hingeloss + control_energy;
            err = std::abs(total_cost - total_cost_prev);
            
            _cost_helper.add_cost(i_step, hingeloss, control_energy);
            total_cost_prev = total_cost;
            
            i_step ++;
        }
        
        return std::make_tuple(_Kt, _dt);
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
            Aki = _ei.decomp3d(_Akt, _nx, _nx, i);
            aki = _ei.decomp3d(_akt, _nx, 1, i);
            hAi = _ei.decomp3d(_hAkt, _nx, _nx, i);
            hai = _ei.decomp3d(_hakt, _nx, 1, i);
            Bi = _ei.decomp3d(_Bt, _nx, _nu, i);
            Qti = _ei.decomp3d(_Qt, _nx, _nx, i);
            pinvBBTi = _ei.decomp3d(_pinvBBTt, _nx, _nx, i);
            zi = _ei.decomp3d(_zkt, _nx, 1, i);
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

            velocity = zi.block(_nx/2, 0,_nx/2,1);
            grad_h.block(0,0,_nx/2,1) = J_hxy.transpose();
            grad_h.block(_nx/2,0,_nx/2,1) = J_hxy.transpose().cwiseProduct(velocity);

            MatrixXd Hess(_nx, _nx);
            Hess.setZero();
            // if (hinge > 0){
            //     Hess.block(0, 0, _nx / 2, _nx / 2) = MatrixXd::Identity(_nx / 2, _nx / 2) * _sig_obs;
            // }
            // Qki
            Qki = Hess * _eta / (1+_eta) + temp * pinvBBTi * (Aki - hAi) * _eta / (1+_eta) / (1+_eta);
            // rki
            rki = grad_h * hinge * _sig_obs * _eta / (1.0 + _eta) +  temp * pinvBBTi * (aki - hai) * _eta / (1+_eta) / (1+_eta);

            // update Qkt, rkt
            _ei.compress3d(Qki, _Qkt, i);
            _ei.compress3d(rki, _rkt, i);
        }
        
    }



protected:
    gpmp2::PlanarSDF _sdf;
    PlanarPRSDFExample _pRsdf;

    double _eps_sdf;
    double _sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    // TODO: For simple 2D it's 1d (1 ball). Needs to be extended to multiple ball checking cases.
    CostHelper _cost_helper;
};
}