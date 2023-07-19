/**
 * @file ProximalGradientCSNonlinearDyn.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient algorithm for nonlinear covariance steering, nonlinear dynamics.
 * @version 0.1
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "covariance_steering/ProximalGradientCovarianceSteering.h"
#include "dynamics/NonlinearDynamics.h"
#include <memory>

using namespace Eigen;

namespace vimp{

class ProxGradCovSteerNLDyn: public ProxGradCovSteer{
public:
    ProxGradCovSteerNLDyn(){};

    virtual ~ProxGradCovSteerNLDyn(){}

    ProxGradCovSteerNLDyn(const MatrixXd& A0, 
                            const VectorXd& a0, 
                            const MatrixXd& B, 
                            const PGCSParams& params,
                            std::shared_ptr<NonlinearDynamics> pdyn): 
                                            ProxGradCovSteer(A0, a0, B, params), 
                                            _pdyn(pdyn){}

    ProxGradCovSteerNLDyn(const MatrixXd& A0, 
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
                        std::shared_ptr<NonlinearDynamics> pdyn,
                        int max_iteration=20): ProxGradCovSteer(A0, a0, B, sig, nt, eta, eps, z0, Sig0, zT, SigT, max_iteration), 
                                            _pdyn(pdyn){}
    

    void step(int indx) override{

        // propagate the mean and the covariance
        propagate_mean();

        std::tuple<LinearDynamics, Matrix3D> res;
        res = _pdyn->linearize(_zkt, _Akt, _Sigkt);

        LinearDynamics h_dyn = std::get<0>(res);
        Matrix3D nTrt = std::get<1>(res);

        // Matrix3D hAt(_nx, _nx, _nt), hat(_nx, 1, _nt);
        _hAkt = h_dyn.At();
        _hakt = h_dyn.at();

        MatrixXd Aprior = _Akt / (1+_eta) + _hAkt * _eta / (1+_eta);
        MatrixXd aprior = _akt / (1+_eta) + _hakt * _eta / (1+_eta);
        
        // Update Qkt, rkt
        update_Qrk();

        // solve inner loop linear CS
        solve_linearCS(Aprior, _Bt, aprior, _Qkt, _rkt);

    }

    StepResult step(int indx, double step_size, 
                    const Matrix3D& At, const Matrix3D& at, const Matrix3D& Bt, 
                    const Matrix3D& hAt, const Matrix3D& hat, 
                    const Matrix3D& zt, const Matrix3D& Sigt){}

    /**
     * @brief A step with given local matrices and a given step size;
     * @return (Kkt, dkt, Akt, akt, zkt, Sigkt) 
     */
    StepResult step(int indx, double step_size, 
                    const Matrix3D& At, const Matrix3D& Bt, const Matrix3D& at, 
                    const Matrix3D& zt, const Matrix3D& Sigt) override
    {
        // propagate the mean and the covariance
        
        std::tuple<Matrix3D, Matrix3D> ztSigt;
        ztSigt = propagate_mean(At, at, Bt, zt, Sigt);

        Matrix3D ztnew(_nx, 1, _nt), Sigtnew(_nx, _nx, _nt);
        ztnew = std::get<0>(ztSigt);
        Sigtnew = std::get<1>(ztSigt);

        std::tuple<LinearDynamics, Matrix3D> res;
        res = _pdyn->linearize(ztnew, At, Sigtnew);

        LinearDynamics h_dyn = std::get<0>(res);
        Matrix3D nTrt = std::get<1>(res);

        Matrix3D hAt(_nx, _nx, _nt), hat(_nx, 1, _nt);
        hAt = h_dyn.At();
        hat = h_dyn.at();
        
        MatrixXd Aprior = At / (1 + step_size) + hAt * step_size / (1 + step_size);
        MatrixXd aprior = at / (1 + step_size) + hat * step_size / (1 + step_size);

        // Update Qkt, rkt
        std::tuple<Matrix3D, Matrix3D> Qtrt;
        Qtrt = update_Qrk_NL(ztnew, Sigtnew, At, at, Bt, hAt, hat, nTrt, step_size);

        Matrix3D Qt(_nx, _nx, _nt), rt(_nx, 1, _nt);
        Qt.setZero(); rt.setZero();
        Qt = std::get<0>(Qtrt);
        rt = std::get<1>(Qtrt);

        // solve inner loop linear CS
        std::tuple<Matrix3D, Matrix3D, Matrix3D, Matrix3D> KtdtAtat;
        KtdtAtat = solve_linearCS_return(Aprior, Bt, aprior, Qt, rt);

        return std::make_tuple(std::get<0>(KtdtAtat), 
                               std::get<1>(KtdtAtat), 
                               std::get<2>(KtdtAtat), 
                               std::get<3>(KtdtAtat), 
                               std::get<0>(ztSigt), 
                               std::get<1>(ztSigt));
    }

    /**
     * @brief linearization
     */
    void linearization(){
        std::tuple<LinearDynamics, Matrix3D> res;
        res = _pdyn->linearize(_zkt, _Akt, _Sigkt);
        _hAkt = std::get<0>(res).At();
        _Bt   = std::get<0>(res).Bt();
        _hakt = std::get<0>(res).at();
        _nTrt = std::get<1>(res);
    }

protected:
    std::shared_ptr<NonlinearDynamics> _pdyn;
    
};
}