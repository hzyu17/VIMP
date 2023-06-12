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

    double total_hingeloss(){
        double total_hingeloss = 0;
        std::pair<double, VectorXd> hingeloss_gradient;
        MatrixXd zi(_nx, 1);
        for (int i=0; i<_nt; i++){
            zi = _ei.decomp3d(_zkt, _nx, 1, i);

            // Compute hinge loss and its gradients
            int n_spheres = _robot_sdf.RobotModel().nr_body_spheres();
            std::tuple<VectorXd, MatrixXd> hingeloss_gradient;
            
            hingeloss_gradient = _robot_sdf.hinge_jacobian(zi.block(0,0,2,1));
            VectorXd hinge(n_spheres);
            hinge = std::get<0>(hingeloss_gradient);

            MatrixXd Sig_obs{_Sig_obs * MatrixXd::Identity(n_spheres, n_spheres)};
            total_hingeloss += hinge.transpose() * Sig_obs * hinge;
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
    std::tuple<Matrix3D, Matrix3D, int> optimize() override{
        double err = 1;

        double total_cost_prev = 0.0;

        int i_step = 0;
        while ((err > _stop_err) && (i_step < _max_iter)){
                        
            this->step(i_step);

            double hingeloss = this->total_hingeloss();

            double control_energy = this->total_control_energy();
            double total_cost = hingeloss + control_energy;
            err = std::abs(total_cost - total_cost_prev);
            
            _cost_helper.add_cost(i_step, hingeloss, control_energy);
            total_cost_prev = total_cost;

            i_step = i_step+1;

        }
        // _cost_helper.plot_costs();
        
        return std::make_tuple(_Kt, _dt, i_step);
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

            // std::cout << "_Sig_obs " << _Sig_obs << std::endl;
            // if (hinge > 0){
            //     Hess.block(0, 0, _nx / 2, _nx / 2) = MatrixXd::Identity(_nx / 2, _nx / 2) * _Sig_obs;
            // }
            // Qki
            Qki = Hess * _eta / (1+_eta) + temp * pinvBBTi * (Aki - hAi) * _eta / (1+_eta) / (1+_eta);
            // rki
            rki = grad_h.transpose() * Sig_obs * hinge * _eta / (1.0 + _eta) +  temp * pinvBBTi * (aki - hai) * _eta / (1+_eta) / (1+_eta);
            // update Qkt, rkt
            _ei.compress3d(Qki, _Qkt, i);
            _ei.compress3d(rki, _rkt, i);
        }
        // _ei.print_matrix(_Qkt, "_Qkt");
        // _ei.print_matrix(_rkt, "_rkt");
        
    }

protected:
    RobotSDF _robot_sdf;
    double _eps_sdf;
    double _Sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    // TODO: For simple 2D it's 1d (1 ball). Needs to be extended to multiple ball checking cases.
    CostHelper _cost_helper;
};
}