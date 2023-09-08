/**
 * @file PGCSPlanarQuadSDF.h
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
#include <gpmp2/kinematics/PointRobotModel.h>
#include "robots/PlanarQuadrotorSDFExample.h"
#include "dynamics/PlanarQuadDynamics.h"

using namespace Eigen;

namespace vimp{

class PGCSPlanarQuadSDF: public ProxGradCovSteerNLDyn{
public:

    virtual ~PGCSPlanarQuadSDF(){}

    PGCSPlanarQuadSDF(const MatrixXd& A0, 
                        const VectorXd& a0, 
                        const MatrixXd& B, 
                        const std::shared_ptr<PlanarQuadDynamics>& pdyn,
                        const PGCSParams& params): 

                        ProxGradCovSteerNLDyn(A0, a0, B, params, pdyn),
                        _eps_sdf(params.eps_sdf()),
                        _sig_obs(params.sig_obs()),
                        _robot_sdf(params.eps_sdf(), params.radius(), params.map_name(), params.sdf_file()),
                        _cost_helper(_max_iter){}

    PGCSPlanarQuadSDF(const MatrixXd& A0, 
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
                        const std::shared_ptr<PlanarQuadDynamics>& pdyn,
                        double eps_sdf,
                        const gpmp2::PlanarSDF& sdf,
                        double sig_obs,
                        int max_iter=20): ProxGradCovSteerNLDyn(A0, a0, B, sig, nt, eta, eps, z0, Sig0, zT, SigT, pdyn, max_iter),
                                        _eps_sdf(eps_sdf),
                                        _sdf(sdf),
                                        _sig_obs(sig_obs){}

    std::tuple<Matrix3D, Matrix3D> update_Qrk(const Matrix3D& zt, 
                                                const Matrix3D& Sigt, 
                                                const Matrix3D& At, 
                                                const Matrix3D& at, 
                                                const Matrix3D& Bt,
                                                const Matrix3D& hAt,
                                                const Matrix3D& hat,
                                                const double step_size) override {}

    std::tuple<Matrix3D, Matrix3D> update_Qrk_NL(const Matrix3D& zt, 
                                                const Matrix3D& Sigt, 
                                                const Matrix3D& At, 
                                                const Matrix3D& at, 
                                                const Matrix3D& Bt,
                                                const Matrix3D& hAt,
                                                const Matrix3D& hat,
                                                const Matrix3D& nTrt,
                                                const double step_size) override
    {
        MatrixXd Ai(_nx, _nx), Bi(_nx, _nu), pinvBBTi(_nx, _nx), ai(_nx, 1), 
                 hAi(_nx, _nx), hai(_nx, 1), nTri(_nx, 1),
                 Qti(_nx, _nx), Qki(_nx, _nx), rki(_nx, 1), zi(_nx, 1);
        MatrixXd temp(_nx, _nx);

        Matrix3D Qt(_nx, _nx, _nt), rt(_nx, 1, _nt);
        Qt.setZero();
        rt.setZero();

        // for each time step
        _Qkt.setZero();
        _rkt.setZero();
        Matrix3D par_V_x = grad_V(zt);
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
            nTri = _ei.decomp3d(nTrt, _nx, 1, i);

            // Compute hinge loss and its gradients
            VectorXd par_V_x_i = _ei.decomp3d(par_V_x, _nx, 1, i);
            MatrixXd Hess(_nx, _nx);
            Hess.setZero(); 

            // Qki
            Qki = Hess*step_size/(1+step_size) + temp*pinvBBTi*(Ai - hAi)*step_size/(1+step_size)/(1+step_size);
            // rki
            rki = par_V_x_i*step_size/(1+step_size) + 
                    nTri*step_size/(1+step_size)/2 + 
                    temp*pinvBBTi*(ai - hai)*step_size/(1+step_size) /(1+step_size);
            
            // update Qkt, rkt
            _ei.compress3d(Qki, Qt, i);
            _ei.compress3d(rki, rt, i);
        }

        return make_tuple(Qt, rt);
        
    }


    Matrix3D grad_V(const Matrix3D& zt){
        Matrix3D grad_V_res(_nx, 1, _nt);

        for (int i=0; i<_nt; i++){
            VectorXd zi(_nx, 1);
            zi = _ei.decomp3d(zt, _nx, 1, i);
            std::tuple<VectorXd, MatrixXd> hingeloss_gradient;

            hingeloss_gradient = _robot_sdf.hinge_jacobian(zi);
            VectorXd hinge = std::get<0>(hingeloss_gradient);

            int n_spheres = hinge.rows();
            // Jacobian w.r.t. the 2D positions (px, pz)
            MatrixXd J_hxy(n_spheres, 2);
            J_hxy = std::get<1>(hingeloss_gradient);

            // Jacobian w.r.t. the whole state space
            MatrixXd grad_h(n_spheres, _nx);
            grad_h.setZero();
            
            for (int i_s=0; i_s<n_spheres; i_s++){
                grad_h.block(i_s,0,1,2) = J_hxy.row(i_s);
            }      

            MatrixXd Sig_obs{_sig_obs * MatrixXd::Identity(n_spheres, n_spheres)};
            VectorXd grad_V_i = grad_h.transpose()*Sig_obs*hinge;
            _ei.compress3d(grad_V_i, grad_V_res, i);

        }

        return grad_V_res;

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

            hingeloss_gradient = _robot_sdf.hinge_jacobian(zi.block(0,0,2,1));

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
     * @return std::tuple<MatrixXd, MatrixXd>  representing (Kt, dt, At, Bt, at, zkt, Skt)
     */
    std::tuple<Matrix3D, Matrix3D, NominalHistory> backtrack() override{
        double err = 1;
        int i_step = 0; 
        double total_cost_prev = 1e9; // initial cost buffer
        bool no_better_stepsize = false;

        NominalHistory hnom;
        std::vector<Matrix3D> v_zt, v_Sigzt;

        Matrix3D As_prev(_nx, _nx, _nt), as_prev(_nx, 1, _nt);
        As_prev = _Akt;
        as_prev = _akt;

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
                KtdtAtatztSigt = step(i_step, step_size, _Akt, _Bt, _akt, _zkt, _Sigkt);

                // compute the tentative cost
                Matrix3D zt(_nx, 1, _nt), Sigt(_nx, _nx, _nt), Kt(_nu, _nx, _nt), dt(_nu, 1, _nt);
                zt.setZero(); Kt.setZero(); dt.setZero(); Sigt.setZero();

                Kt = std::get<0>(KtdtAtatztSigt);
                dt = std::get<1>(KtdtAtatztSigt);
                zt = std::get<4>(KtdtAtatztSigt);
                Sigt = std::get<5>(KtdtAtatztSigt);

                double total_cost = control_energy(zt, Sigt, Kt, dt) + hingeloss(zt, Sigt);
                _cost_helper.add_cost(i_step, hingeloss(zt, Sigt), control_energy(zt, Sigt, Kt, dt));

                std::cout << " total cost " << std::fixed << std::setprecision(7) << total_cost << std::endl;
    
                if (total_cost < total_cost_prev){
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
                    std::cout << "There is no better step size." << std::endl;

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

                // std::cout << "step size " << step_size << std::endl;
            }
            
            // double err = (As_prev - _Akt).norm() / _Akt.norm() / _nt + (as_prev - _akt).norm() / _akt.norm() / _nt;
            // std::cout.precision(7);
            // std::cout << "err:  " << err << std::endl;

            As_prev = _Akt;
            as_prev = _akt;

        }  

        _cost_helper.plot_costs();
        hnom = make_tuple(v_zt, v_Sigzt);

        /**
         * Recover the optimal control.
         * */
        std::tuple<Matrix3D, Matrix3D> ztSigtStar = propagate_nominal(_Akt, _akt, _Bt, _z0, _Sig0);
        Matrix3D zt_star = std::get<0>(ztSigtStar);
        Matrix3D Sigt_star = std::get<1>(ztSigtStar);

        std::tuple<LinearDynamics, Matrix3D> linearizing_result = _pdyn->linearize(zt_star, _Akt, Sigt_star);
        
        Matrix3D hAt_star = std::get<0>(linearizing_result).At();
        Matrix3D hat_star = std::get<0>(linearizing_result).at();
        Matrix3D nTr_star = std::get<1>(linearizing_result);

        _hAkt = hAt_star;
        _hakt = hat_star;
        _nTrt = nTr_star;

        Matrix3D r_star(_nx, 1, _nt), Q_star(_nx, _nx, _nt);
        Q_star.setZero();
        r_star.setZero();

        Matrix3D par_V_x = grad_V(zt_star);
        MatrixXd r_star_i(_nx, 1);
        r_star_i.setZero();
        for (int i=0; i<_nt; i++){
            VectorXd par_V_x_i = _ei.decomp3d(par_V_x, _nx, 1, i);
            VectorXd nTri = _ei.decomp3d(nTr_star, _nx, 1, i);
            r_star_i = par_V_x_i + nTri / 2;
            // r_star_i = _ei.decomp3d(nTr_star, _nx, 1, i) / 2;
            _ei.compress3d(r_star_i, r_star, i);
        }

        std::tuple<Matrix3D, Matrix3D, Matrix3D, Matrix3D> LinCSResKtdtAtat;
        LinCSResKtdtAtat = solve_linearCS_return(hAt_star, _Bt, hat_star, Q_star, r_star);

        std::tuple<Matrix3D, Matrix3D, Matrix3D, Matrix3D, Matrix3D, Matrix3D> final_KtdtAtatztSigt;
        final_KtdtAtatztSigt = std::make_tuple(std::get<0>(LinCSResKtdtAtat), 
                                               std::get<1>(LinCSResKtdtAtat),
                                               std::get<2>(LinCSResKtdtAtat),
                                               std::get<3>(LinCSResKtdtAtat),
                                               zt_star,
                                               Sigt_star);

        update_from_step_res(final_KtdtAtatztSigt);

        return std::make_tuple(this->_Kt, this->_dt, hnom);      
    }


    void save_costs(const string& filename){
        _cost_helper.save_costs(filename);
    }


protected:
    PlanarQuadrotorSDFExample _robot_sdf;
    gpmp2::PlanarSDF _sdf;
    double _eps_sdf;
    double _sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    CostHelper _cost_helper;
};
}