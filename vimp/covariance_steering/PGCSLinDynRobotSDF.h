/**
 * @file PGCSLinDynRobotSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Template class for nonlinear covariance steering with sdf and linear robot model. 
 * @version 0.1
 * @date 2023-03-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ProximalGradientCSLinearDyn.h"
#include "../helpers/cost_helpers.h"

using namespace Eigen;

namespace vimp{

template <typename RobotSDF> 
class PGCSLinDynRobotSDF: public ProxGradCovSteerLinDyn{
public:
    PGCSLinDynRobotSDF(const MatrixXd& A0, 
                        const VectorXd& a0, 
                        const MatrixXd& B, 
                        const std::shared_ptr<LinearDynamics>& pdyn,
                        ExperimentParams& params):
                        ProxGradCovSteerLinDyn(A0, a0, B, pdyn, params),
                        _eps_sdf(params.eps_sdf()),
                        _Sig_obs(params.sig_obs()),
                        _robot_sdf(params.eps_sdf()),
                        _cost_helper(_max_iter){}


    PGCSLinDynRobotSDF(const MatrixXd& A0, 
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
                            double sig_obs,
                            double stop_err,
                            int max_iter):
                        ProxGradCovSteerLinDyn(A0, a0, B, sig, nt, eta, eps, z0, Sig0, zT, SigT, pdyn, stop_err, max_iter),
                        _eps_sdf(eps_sdf),
                        _Sig_obs(sig_obs),
                        _robot_sdf(eps_sdf),
                        _cost_helper(max_iter){ }

    double hingeloss(const Matrix3D& zt){
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

            MatrixXd Sig_obs{_Sig_obs * MatrixXd::Identity(n_spheres, n_spheres)};
            hingeloss += hinge.transpose() * Sig_obs * hinge;
        }
        return hingeloss;
    }

    double hingeloss(){
        return hingeloss(_zkt);
    }

    double control_energy(const Matrix3D& zt, const Matrix3D& Kt, const Matrix3D& dt){
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
        return control_energy(_zkt, _Kt, _dt);
    }

    /**
     * @brief The optimization process, including recording the costs.
     * @return std::tuple<MatrixXd, MatrixXd>  representing (Kt, dt)
     */
    std::tuple<Matrix3D, Matrix3D, NominalHistory> backtrack(){
        double err = 1;
        int i_step = 0; 
        int MBT = 1; // max backtracking number
        double total_cost_prev = 1e9; // initial cost buffer

        NominalHistory hnom;
        std::vector<Matrix3D> v_zt, v_Sigzt;

        while ((total_cost_prev > _stop_err) && (i_step < _max_iter)){
            
            std::cout << "================ iter " << i_step << " ================" << std::endl;
            // std::cout << "total_cost_prev " << total_cost_prev << std::endl;

            // backtracking 
            double step_size = _eta; // initial step size
            for (int i_bt=0; i_bt<MBT; i_bt++){

                // std::cout << " ----- backtracking " << i_bt << " ----- " << std::endl;
                
                // shringking step size
                // step_size = 0.1*step_size;

                // tentative one step
                StepResult KtdtAtatztSigt; // return type of one step: (Kt, dt, At, at, zt, Sigt) 
                KtdtAtatztSigt = step(i_step, step_size, _Akt, _akt, _Bt, _hAkt, _hakt, _zkt, _Sigkt);

                // compute the tentative cost
                Matrix3D zt(_nx, 1, _nt), Kt(_nu, _nx, _nt), dt(_nu, 1, _nt);
                zt.setZero(); Kt.setZero(); dt.setZero();

                Kt = std::get<0>(KtdtAtatztSigt);
                dt = std::get<1>(KtdtAtatztSigt);
                zt = std::get<4>(KtdtAtatztSigt);

                double total_cost = control_energy(zt, Kt, dt) + hingeloss(zt);
                std::cout << "total cost " << total_cost << std::endl;
                total_cost_prev = total_cost;
                
                // // stop backtracking 
                // if (total_cost < total_cost_prev){
                    
                //     // update the internal parameters
                //     update_from_step_res(AtatKtdtztSigt);

                //     // register for the current cost
                //     total_cost_prev = total_cost;
                    
                //     // go to next iteration
                //     i_step += 1;
                    
                //     break;
                // }

                if (i_bt == MBT-1){
                    update_from_step_res(KtdtAtatztSigt);
                    v_zt.push_back(_zkt);
                    v_Sigzt.push_back(_Sigkt);
                    i_step += 1;
                    break;
                }
                
            }
        }  
        // _cost_helper.plot_costs();

        hnom = make_tuple(v_zt, v_Sigzt);
        return std::make_tuple(_Kt, _dt, hnom);      
    }

    /**
     * @brief The optimization process, including recording the costs.
     * @return std::tuple<MatrixXd, MatrixXd>  representing (Kt, dt)
     */
    std::tuple<Matrix3D, Matrix3D, NominalHistory> optimize() override{
        double err = 1;

        double total_cost_prev = 1e6;

        int i_step = 0;
        NominalHistory hnom;
        std::vector<Matrix3D> v_zt, v_Sigzt;
            
        while ((err > _stop_err) && (i_step < _max_iter)){

            std::cout << "================ iter " << i_step << " ================" << std::endl;     
            step(i_step);

            double total_hingeloss = hingeloss();
            double Eu = control_energy();
            double total_cost = total_hingeloss + Eu;
            
            err = std::abs(total_cost - total_cost_prev);
            
            _cost_helper.add_cost(i_step, total_hingeloss, Eu);
            total_cost_prev = total_cost;

            i_step = i_step+1;

            v_zt.push_back(_zkt);
            v_Sigzt.push_back(_Sigkt);

        }
        // _cost_helper.plot_costs();
        hnom = make_tuple(v_zt, v_Sigzt);
        return std::make_tuple(_Kt, _dt, hnom);
    }

    /**
     * @brief Qrk with given matrices.
     * return: (Qt, rt)
     */
    std::tuple<Matrix3D, Matrix3D> update_Qrk(const Matrix3D& zt, 
                                              const Matrix3D& Sigt, 
                                              const Matrix3D& At, 
                                              const Matrix3D& at, 
                                              const Matrix3D& Bt,
                                              const Matrix3D& hAt,
                                              const Matrix3D& hat,
                                              const double step_size)
    {
        MatrixXd Ai(_nx, _nx), Bi(_nx, _nu), pinvBBTi(_nx, _nx), ai(_nx, 1), 
                 hAi(_nx, _nx), hai(_nx, 1),
                 Qti(_nx, _nx), Qki(_nx, _nx), rki(_nx, 1), zi(_nx, 1);
        MatrixXd temp(_nx, _nx);

        Matrix3D Qt(_nx, _nx, _nt), rt(_nx, 1, _nt);
        Qt.setZero();
        rt.setZero();

        for (int i=0; i<_nt; i++){
            Ai = _ei.decomp3d(At, _nx, _nx, i);
            ai = _ei.decomp3d(at, _nx, 1, i);
            hAi = _ei.decomp3d(hAt, _nx, _nx, i);
            hai = _ei.decomp3d(hat, _nx, 1, i);
            Bi = _ei.decomp3d(Bt, _nx, _nu, i);
            Qti = _ei.decomp3d(_Qt, _nx, _nx, i);
            pinvBBTi = _ei.decomp3d(_pinvBBTt, _nx, _nx, i);
            zi = _ei.decomp3d(zt, _nx, 1, i);
            temp = (Ai - hAi).transpose();
            // Compute hinge loss and its gradients
            int n_spheres = _robot_sdf.RobotModel().nr_body_spheres();
            std::tuple<VectorXd, MatrixXd> hingeloss_gradient;
            
            hingeloss_gradient = _robot_sdf.hinge_jacobian(zi.block(0,0,_nx/2,1));
            VectorXd hinge(n_spheres);
            MatrixXd J_hxy(n_spheres, _nx/2);
            hinge = std::get<0>(hingeloss_gradient);
            J_hxy = std::get<1>(hingeloss_gradient);

            // MatrixXd grad_h(_nx, 1), velocity(_nx/2, 1);
            MatrixXd grad_h(n_spheres, _nx), velocity(1, _nx/2);
            // grad_h << J_hxy(0), J_hxy(1), J_hxy(0) * zi(2), J_hxy(1) * zi(3);
            velocity = zi.block(_nx/2,0,_nx/2,1).transpose();

            for (int i_s=0; i_s<n_spheres; i_s++){
                grad_h.block(i_s,0,1,_nx/2) = J_hxy.row(i_s);
                grad_h.block(i_s,_nx/2,1,_nx/2) = J_hxy.row(i_s).cwiseProduct(velocity);
            }          
            MatrixXd Sig_obs{_Sig_obs * MatrixXd::Identity(n_spheres, n_spheres)};
            MatrixXd Hess(_nx, _nx);
            Hess.setZero();

            // Qki
            Qki = Hess * step_size / (1+step_size) + temp * pinvBBTi * (Ai - hAi) * step_size / (1+step_size) / (1+step_size);
            // rki
            rki = grad_h.transpose() * Sig_obs * hinge * step_size / (1.0 + step_size) +  temp * pinvBBTi * (ai - hai) * step_size / (1+step_size) / (1+step_size);
            // update Qkt, rkt
            _ei.compress3d(Qki, Qt, i);
            _ei.compress3d(rki, rt, i);
        }

        return make_tuple(Qt, rt);
    }


    void update_Qrk() override{
        std::tuple<Matrix3D, Matrix3D> Qtrt;
        Qtrt = update_Qrk(_zkt, _Sigkt, _Akt, _akt, _Bt, _hAkt, _hakt, _eta);
        _Qkt = std::get<0>(Qtrt);
        _rkt = std::get<1>(Qtrt);

        // MatrixXd Aki(_nx, _nx), Bi(_nx, _nu), pinvBBTi(_nx, _nx), aki(_nx, 1), 
        //          hAi(_nx, _nx), hai(_nx, 1),
        //          Qti(_nx, _nx), Qki(_nx, _nx), rki(_nx, 1), zi(_nx, 1);
        // MatrixXd temp(_nx, _nx);
        // // for each time step
        // _Qkt.setZero();
        // _rkt.setZero();

        // for (int i=0; i<_nt; i++){
        //     Aki = _ei.decomp3d(_Akt, _nx, _nx, i);
        //     aki = _ei.decomp3d(_akt, _nx, 1, i);
        //     hAi = _ei.decomp3d(_hAkt, _nx, _nx, i);
        //     hai = _ei.decomp3d(_hakt, _nx, 1, i);
        //     Bi = _ei.decomp3d(_Bt, _nx, _nu, i);
        //     Qti = _ei.decomp3d(_Qt, _nx, _nx, i);
        //     pinvBBTi = _ei.decomp3d(_pinvBBTt, _nx, _nx, i);
        //     zi = _ei.decomp3d(_zkt, _nx, 1, i);
        //     temp = (Aki - hAi).transpose();
        //     // Compute hinge loss and its gradients
        //     int n_spheres = _robot_sdf.RobotModel().nr_body_spheres();
        //     std::tuple<VectorXd, MatrixXd> hingeloss_gradient;
            
        //     hingeloss_gradient = _robot_sdf.hinge_jacobian(zi.block(0,0,_nx/2,1));
        //     VectorXd hinge(n_spheres);
        //     MatrixXd J_hxy(n_spheres, _nx/2);
        //     hinge = std::get<0>(hingeloss_gradient);
        //     J_hxy = std::get<1>(hingeloss_gradient);


        //     // MatrixXd grad_h(_nx, 1), velocity(_nx/2, 1);
        //     MatrixXd grad_h(n_spheres, _nx), velocity(1, _nx/2);
        //     // grad_h << J_hxy(0), J_hxy(1), J_hxy(0) * zi(2), J_hxy(1) * zi(3);
        //     velocity = zi.block(_nx/2,0,_nx/2,1).transpose();

        //     for (int i_s=0; i_s<n_spheres; i_s++){
        //             grad_h.block(i_s,0,1,_nx/2) = J_hxy.row(i_s);
        //         grad_h.block(i_s,_nx/2,1,_nx/2) = J_hxy.row(i_s).cwiseProduct(velocity);
        //     }          
        //     MatrixXd Sig_obs{_Sig_obs * MatrixXd::Identity(n_spheres, n_spheres)};
        //     MatrixXd Hess(_nx, _nx);
        //     Hess.setZero();

        //     // std::cout << "_Sig_obs " << _Sig_obs << std::endl;
        //     // if (hinge > 0){
        //     //     Hess.block(0, 0, _nx / 2, _nx / 2) = MatrixXd::Identity(_nx / 2, _nx / 2) * _Sig_obs;
        //     // }
        //     // Qki
        //     Qki = Hess * _eta / (1+_eta) + temp * pinvBBTi * (Aki - hAi) * _eta / (1+_eta) / (1+_eta);
        //     // rki
        //     rki = grad_h.transpose() * Sig_obs * hinge * _eta / (1.0 + _eta) +  temp * pinvBBTi * (aki - hai) * _eta / (1+_eta) / (1+_eta);
        //     // update Qkt, rkt
        //     _ei.compress3d(Qki, _Qkt, i);
        //     _ei.compress3d(rki, _rkt, i);
        // }
        // // _ei.print_matrix(_Qkt, "_Qkt");
        // // _ei.print_matrix(_rkt, "_rkt");
        
    }

protected:
    RobotSDF _robot_sdf;
    double _eps_sdf;
    double _Sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    // TODO: For simple 2D it's 1d (1 ball). Needs to be extended to multiple ball checking cases.
    CostHelper _cost_helper;
};
}