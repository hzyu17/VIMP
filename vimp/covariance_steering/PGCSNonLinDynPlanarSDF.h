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
#include "helpers/CostHelper.h"
#include "helpers/hinge2Dhelper.h"

using namespace Eigen;

namespace vimp{

template <typename RobotSDF> 
class PGCSNonLinDynPlanarSDF: public ProxGradCovSteerNLDyn{
public:
    PGCSNonLinDynPlanarSDF(const MatrixXd& A0, 
                            const VectorXd& a0, 
                            const MatrixXd& B, 
                            const PGCSParams& params,
                            const std::shared_ptr<NonlinearDynamics>& pdyn,
                            const gpmp2::PlanarSDF& sdf): ProxGradCovSteerNLDyn(A0, a0, B, params, pdyn),
                                            _eps_sdf(params.eps_sdf()),
                                            _sdf(sdf),
                                            _sig_obs(params.sig_obs()){}

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
                                                _sig_obs(sig_obs){}


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
            //     Hess.block(0, 0, _nx / 2, _nx / 2) = MatrixXd::Identity(_nx / 2, _nx / 2) * _sig_obs;
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

    double control_energy(const Matrix3D& zt, const Matrix3D& Sigt, const Matrix3D& Kt, const Matrix3D& dt){
        double total_Eu = 0;
        MatrixXd zi(_nx, 1), Ki(_nu, _nx), di(_nu, 1);

        for (int i=0; i<_nt; i++){
            zi = _ei.decomp3d(zt, _nx, 1, i);
            Ki = _ei.decomp3d(Kt, _nu, _nx, i);
            di = _ei.decomp3d(dt, _nu, 1, i);

            VectorXd u_i(_nu, 1);
            u_i = Ki * zi + di;

            double E_ui = u_i.dot(u_i) * _deltt;

            total_Eu += E_ui;
        }
        return total_Eu;
    }

    double control_energy(){
        return control_energy(_zkt, _Sigkt, _Kt, _dt);
    }

    double hingeloss(const Matrix3D& zt, const Matrix3D& Sigt){
        double hingeloss = 0;
        std::pair<double, VectorXd> hingeloss_gradient;
        MatrixXd zi(_nx, 1);
        for (int i=0; i<_nt; i++){
            zi = _ei.decomp3d(zt, _nx, 1, i);

            // Compute hinge loss and its gradients
            int n_spheres = _robot_sdf.RobotModel().nr_body_spheres();
            std::tuple<VectorXd, MatrixXd> hingeloss_gradient;

            hingeloss_gradient = _robot_sdf.hinge_jacobian(zi.block(0,0,_nx/2,1));

            VectorXd hinge(n_spheres);
            hinge = std::get<0>(hingeloss_gradient);

            MatrixXd Sig_obs{_sig_obs * MatrixXd::Identity(n_spheres, n_spheres)};
            hingeloss += hinge.transpose() * Sig_obs * hinge;
        }
        return hingeloss;
    }

    double hingeloss(){
        return hingeloss(_zkt, _Sigkt);
    }

    /**
     * @brief The optimization process, including recording the costs.
     * @return std::tuple<MatrixXd, MatrixXd>  representing (Kt, dt)
     */
    std::tuple<Matrix3D, Matrix3D, NominalHistory> backtrack() override{
        double err = 1;
        int i_step = 0; 
        double total_cost_prev = 1e9; // initial cost buffer
        bool no_better_stepsize = false;

        NominalHistory hnom;
        std::vector<Matrix3D> v_zt, v_Sigzt;

        while ( ((err<0) || (err>_stop_err)) && (i_step < _max_iter) && (!no_better_stepsize)){
            
            std::cout << "================ iter " << i_step << " ================" << std::endl;

            // backtracking 
            double step_size = _eta; // initial step size
            double best_backtrack_cost = 1e9;
            StepResult best_KtdtAtatztSigt;
            for (int i_bt=0; i_bt<_max_n_backtrack; i_bt++){

                std::cout << " ----- backtracking " << i_bt << " ----- " << std::endl;

                // tentative one step
                StepResult KtdtAtatztSigt; // return type of one step: (Kt, dt, At, at, zt, Sigt) 
                KtdtAtatztSigt = step(i_step, step_size, _Akt, _akt, _Bt, _zkt, _Sigkt);

                // compute the tentative cost
                Matrix3D zt(_nx, 1, _nt), Sigt(_nx, _nx, _nt), Kt(_nu, _nx, _nt), dt(_nu, 1, _nt);
                zt.setZero(); Kt.setZero(); dt.setZero(); Sigt.setZero();

                Kt = std::get<0>(KtdtAtatztSigt);
                dt = std::get<1>(KtdtAtatztSigt);
                zt = std::get<4>(KtdtAtatztSigt);
                Sigt = std::get<5>(KtdtAtatztSigt);

                double total_cost = control_energy(zt, Sigt, Kt, dt) + hingeloss(zt, Sigt);
                // _cost_helper.add_cost(i_step, hingeloss(zt, Sigt), control_energy(zt, Sigt, Kt, dt));

                std::cout << " total cost " << std::fixed << std::setprecision(4) << total_cost << std::endl;
    
                if (total_cost < total_cost_prev){

                    std::cout << "Found better cost " << std::endl;
                    
                    // update the internal parameters
                    update_from_step_res(KtdtAtatztSigt);

                    // register for the current cost
                    err = total_cost_prev - total_cost;
                    total_cost_prev = total_cost;
                    
                    // go to next iteration
                    i_step += 1;

                    v_zt.push_back(_zkt);
                    v_Sigzt.push_back(_Sigkt);
                    
                    break;
                }else{
                    if (total_cost < best_backtrack_cost){
                        best_backtrack_cost = total_cost;
                        best_KtdtAtatztSigt = KtdtAtatztSigt;
                    }
                    // shringking step size
                    step_size = _backtrack_ratio*step_size;
                }

                // no good step size found: taking the smallest step.
                if (i_bt == _max_n_backtrack-1){
                    std::cout << "there is no better step size " << std::endl;

                    // update the internal parameters
                    update_from_step_res(best_KtdtAtatztSigt);

                    // register for the current cost
                    err = total_cost_prev - best_backtrack_cost;
                    total_cost_prev = best_backtrack_cost;
                    
                    // go to next iteration
                    i_step += 1;

                    v_zt.push_back(_zkt);
                    v_Sigzt.push_back(_Sigkt);

                    break;
                }

                std::cout << "step size " << step_size << std::endl;
            }
        }  

        // _cost_helper.plot_costs();

        hnom = make_tuple(v_zt, v_Sigzt);
        return std::make_tuple(_Kt, _dt, hnom);      
    }


protected:
    RobotSDF _robot_sdf;
    gpmp2::PlanarSDF _sdf;
    double _eps_sdf;
    double _sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    CostHelper _cost_helper;
};
}