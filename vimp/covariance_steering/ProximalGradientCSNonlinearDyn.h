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

#include "ProximalGradientCS.h"
#include "../dynamics/NonlinearDynamics.h"
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
                        double sig,
                        int nt,
                        double eta,
                        double eps,
                        const VectorXd& z0,
                        const MatrixXd& Sig0,
                        const VectorXd& zT,
                        const MatrixXd& SigT,
                        std::shared_ptr<NonlinearDynamics> pdyn,
                        double Vscale=1.0): ProxGradCovSteer(A0, a0, B, sig, nt, eta, eps, z0, Sig0, zT, SigT), 
                                            _pdyn(pdyn){}
    
    /**
     * @brief Solving a linear covariance steering at each iteration.
     * @return none, but inside already compute (K, d).
     */
    void step(int indx){
        std::cout << "----- iter " << indx << " -----" << std::endl;
        // propagate the mean and the covariance
        propagate_mean();
        linearization();

        MatrixXd A_prior = _Akt / (1+_eta) + _hAkt * _eta / (1+_eta);
        MatrixXd a_prior = _akt / (1+_eta) + _hakt * _eta / (1+_eta);

        // Update Qkt, rkt
        update_Qrk();

        // solve inner loop linear CS
        solve_internal_linearCS(A_prior, _Bt, a_prior, _Qkt, _rkt);
    }

    /**
     * @brief linearization
     */
    void linearization(){
        std::tuple<LinearDynamics, Matrix3D> res;
        res = _pdyn->linearize(_zkt, _sig, _Akt, _Sigkt);
        _hAkt = std::get<0>(res).At();
        _Bt   = std::get<0>(res).Bt();
        _hakt = std::get<0>(res).at();
        _nTrt = std::get<1>(res);
    }

protected:
    std::shared_ptr<NonlinearDynamics> _pdyn;
    
};
}