/**
 * @file ProximalGradientCSLinearDyn.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient algorithm for nonlinear covariance steering, with linear dynamics. 
 * @version 0.1
 * @date 2023-03-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "../dynamics/LinearDynamics.h"
#include "ProximalGradientCovarianceSteering.h"

using namespace Eigen;

namespace vimp{

class ProxGradCovSteerLinDyn: public ProxGradCovSteer{
public:
    ProxGradCovSteerLinDyn(){};

    virtual ~ProxGradCovSteerLinDyn(){}

    ProxGradCovSteerLinDyn(const MatrixXd& A0, 
                            const VectorXd& a0, 
                            const MatrixXd& B,
                            const std::shared_ptr<LinearDynamics>& pdyn,
                            ExperimentParams& params):
                            ProxGradCovSteer(A0, a0, B, params),
                            _pdyn(pdyn){
                                _hAkt = pdyn->At();
                                _Bt = pdyn->Bt();
                                _hakt = pdyn->at();
                            }

    ProxGradCovSteerLinDyn(const MatrixXd& A0, 
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
                            double stop_err,
                            int max_iter): ProxGradCovSteer(A0, a0, B, sig, nt, eta, eps, z0, Sig0, zT, SigT, stop_err, max_iter), 
                                            _pdyn(pdyn){
                                                _hAkt = pdyn->At();
                                                _Bt = pdyn->Bt();
                                                _hakt = pdyn->at();
                                            }
                                            
    
    /**
     * @brief Solving a linear covariance steering at each iteration.
     * @return none, but inside already compute (K, d).
     */
    void step(int indx) override{

        // propagate the mean and the covariance
        propagate_nominal();

        MatrixXd Aprior = _Akt / (1+_eta) + _hAkt * _eta / (1+_eta);
        MatrixXd aprior = _akt / (1+_eta) + _hakt * _eta / (1+_eta);
        
        // Update Qkt, rkt
        update_Qrk();

        // solve inner loop linear CS
        solve_linearCS(Aprior, _Bt, aprior, _Qkt, _rkt);

    }

    /**
     * @brief A step with given local matrices and a given step size;
     * @return (Kkt, dkt, Akt, akt, zkt, Sigkt) 
     */
    StepResult step(int indx, 
                    double step_size, 
                    const Matrix3D& At, 
                    const Matrix3D& at, 
                    const Matrix3D& Bt,
                    const Matrix3D& hAt,
                    const Matrix3D& hat, 
                    const Matrix3D& zt, 
                    const Matrix3D& Sigt) override
    {
        // propagate the mean and the covariance
        
        std::tuple<Matrix3D, Matrix3D> ztSigt;
        ztSigt = propagate_nominal(At, at, Bt, zt, Sigt);

        Matrix3D ztnew(_nx, 1, _nt), Sigtnew(_nx, _nx, _nt);
        ztnew = std::get<0>(ztSigt);
        Sigtnew = std::get<1>(ztSigt);
        
        MatrixXd Aprior = At / (1 + step_size) + hAt * step_size / (1 + step_size);
        MatrixXd aprior = at / (1 + step_size) + hat * step_size / (1 + step_size);
        
        // Update Qkt, rkt
        std::tuple<Matrix3D, Matrix3D> Qtrt;
        Qtrt = update_Qrk(ztnew, Sigtnew, At, at, Bt, _hAkt, _hakt, step_size);

        Matrix3D Qt(_nx, _nx, _nt), rt(_nx, 1, _nt);
        Qt.setZero(); rt.setZero();
        Qt = std::get<0>(Qtrt);
        rt = std::get<1>(Qtrt);

        // solve inner loop linear CS
        std::tuple<Matrix3D, Matrix3D, Matrix3D, Matrix3D> KtdtAtat;
        KtdtAtat = solve_linearCS_return(Aprior, _Bt, aprior, Qt, rt);

        return std::make_tuple(std::get<0>(KtdtAtat), 
                               std::get<1>(KtdtAtat), 
                               std::get<2>(KtdtAtat), 
                               std::get<3>(KtdtAtat), 
                               std::get<0>(ztSigt), 
                               std::get<1>(ztSigt));
    }

protected:
    std::shared_ptr<LinearDynamics> _pdyn;
    
};
}