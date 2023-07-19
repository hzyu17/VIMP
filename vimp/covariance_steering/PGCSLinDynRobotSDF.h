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

#include "covariance_steering/ProximalGradientCSLinearDyn.h"
#include "helpers/CostHelper.h"
#include <iomanip>

using namespace Eigen;

namespace vimp{

template <typename RobotSDF> 
class PGCSLinDynRobotSDF: public ProxGradCovSteerLinDyn{
public:
    virtual ~PGCSLinDynRobotSDF(){}
    
    PGCSLinDynRobotSDF(const MatrixXd& A0, 
                        const VectorXd& a0, 
                        const MatrixXd& B, 
                        const std::shared_ptr<LinearDynamics>& pdyn,
                        PGCSParams& params):
                        ProxGradCovSteerLinDyn(A0, a0, B, pdyn, params),
                        _eps_sdf(params.eps_sdf()),
                        _sig_obs(params.sig_obs()),
                        _robot_sdf(params.eps_sdf(), params.radius(), params.field_file(), params.sdf_file()),
                        _cost_helper(_max_iter){}

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
                KtdtAtatztSigt = step(i_step, step_size, _Akt, _akt, _Bt, _hAkt, _hakt, _zkt, _Sigkt);

                // compute the tentative cost
                Matrix3D zt(_nx, 1, _nt), Sigt(_nx, _nx, _nt), Kt(_nu, _nx, _nt), dt(_nu, 1, _nt);
                zt.setZero(); Kt.setZero(); dt.setZero(); Sigt.setZero();

                Kt = std::get<0>(KtdtAtatztSigt);
                dt = std::get<1>(KtdtAtatztSigt);
                zt = std::get<4>(KtdtAtatztSigt);
                Sigt = std::get<5>(KtdtAtatztSigt);

                double total_cost = control_energy(zt, Sigt, Kt, dt) + hingeloss(zt, Sigt);
                _cost_helper.add_cost(i_step, hingeloss(zt, Sigt), control_energy(zt, Sigt, Kt, dt));

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

        _cost_helper.plot_costs();

        hnom = make_tuple(v_zt, v_Sigzt);
        return std::make_tuple(_Kt, _dt, hnom);      
    }

    std::tuple<Matrix3D, Matrix3D> update_Qrk_NL(const Matrix3D& zt, 
                                                const Matrix3D& Sigt, 
                                                const Matrix3D& At, 
                                                const Matrix3D& at, 
                                                const Matrix3D& Bt,
                                                const Matrix3D& hAt,
                                                const Matrix3D& hat,
                                                const Matrix3D& nTrt,
                                                const double step_size) override{}

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
            MatrixXd grad_h(n_spheres, _nx);
            grad_h.setZero();

            for (int i_s=0; i_s<n_spheres; i_s++){
                grad_h.block(i_s,0,1,_nx/2) = J_hxy.row(i_s);
            }          
            MatrixXd Sig_obs{_sig_obs * MatrixXd::Identity(n_spheres, n_spheres)};
            MatrixXd Hess(_nx, _nx);
            Hess.setZero();
            // Hess = _sig_obs * _sig_obs * MatrixXd::Identity(_nx, _nx);

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
    }

    void save_costs(const string& filename){
        _cost_helper.save_costs(filename);
    }

    template <typename SDF>
    void update_sdf(const SDF& sdf){
        _robot_sdf.update_sdf(sdf);
    }

protected:
    RobotSDF _robot_sdf;
    double _eps_sdf;
    double _sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    CostHelper _cost_helper;
};
}