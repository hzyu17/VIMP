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
                            double sig,
                            int nt,
                            double eta,
                            double eps,
                            const VectorXd& z0,
                            const MatrixXd& Sig0,
                            const VectorXd& zT,
                            const MatrixXd& SigT,
                            const std::shared_ptr<LinearDynamics>& pdyn,
                            int max_iter): ProxGradCovSteer(A0, a0, B, sig, nt, eta, eps, z0, Sig0, zT, SigT, max_iter), 
                                                _pdyn(pdyn){
                                                    _hAkt = _sig * pdyn->At();
                                                    _Bt = _sig * pdyn->Bt();
                                                    _hakt = _sig * pdyn->at();
                                                }
                                            
    
    /**
     * @brief Solving a linear covariance steering at each iteration.
     * @return none, but inside already compute (K, d).
     */
    void step(int indx) override{
        std::cout << "----- iter " << indx << " -----" << std::endl;
        // propagate the mean and the covariance
        propagate_mean();
        
        // _ei.print_matrix(_Akt, "_Akt");
        // _ei.print_matrix(_zkt, "_zkt");

        MatrixXd Aprior = _Akt / (1+_eta) + _hAkt * _eta / (1+_eta);
        MatrixXd aprior = _akt / (1+_eta) + _hakt * _eta / (1+_eta);
        
        // Update Qkt, rkt
        update_Qrk();

        // solve inner loop linear CS
        solve_linearCS(Aprior, _Bt, aprior, _Qkt, _rkt);

    }

protected:
    std::shared_ptr<LinearDynamics> _pdyn;
    
};
}