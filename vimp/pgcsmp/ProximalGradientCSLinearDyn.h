/**
 * @file ProximalGradientCSLinearDyn.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient covariance steering with linear dynamics.
 * @version 0.1
 * @date 2023-03-15
 * @copyright Copyright (c) 2023
 */

#pragma once

#include "dynamics/LinearDynamics.h"
#include "ProximalGradientCovarianceSteering.h"

namespace vimp {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @brief Proximal Gradient Covariance Steering for linear dynamics.
 *
 * Specializes ProxGradCovSteer for systems with known linear dynamics,
 * where the linearization (hA, ha) comes directly from the dynamics model.
 */
class ProxGradCovSteerLinDyn : public ProxGradCovSteer {
public:
    ProxGradCovSteerLinDyn() = default;
    virtual ~ProxGradCovSteerLinDyn() = default;

    ProxGradCovSteerLinDyn(const MatrixXd& A0,
                           const VectorXd& a0,
                           const MatrixXd& B,
                           const std::shared_ptr<LinearDynamics>& pdyn,
                           const PGCSParams& params)
        : ProxGradCovSteer(A0, a0, B, params)
        , _pdyn(pdyn)
    {
        // Initialize linearization from dynamics model
        _hAkt = pdyn->At();
        _Bt = pdyn->Bt();
        _hakt = pdyn->at();
    }

    /**
     * @brief Perform one optimization step.
     *
     * 1. Propagate nominal mean and covariance
     * 2. Compute proximal-weighted prior dynamics
     * 3. Update cost matrices Qk, rk
     * 4. Solve linear covariance steering subproblem
     */
    void step(int indx) override {
        propagate_nominal();

        // Proximal averaging: A_prior = (A + η*hA) / (1 + η)
        MatrixXd Aprior = (_Akt + _hAkt * _eta) / (1 + _eta);
        MatrixXd aprior = (_akt + _hakt * _eta) / (1 + _eta);

        update_Qrk();
        solve_linearCS(Aprior, _Bt, aprior, _Qkt, _rkt);
    }

    /**
     * @brief Step with given matrices (unused variant).
     */
    StepResult step(int indx, double step_size,
                    const Matrix3D& At, const Matrix3D& Bt, const Matrix3D& at,
                    const Matrix3D& zt, const Matrix3D& Sigt) {
        return {};  // Not implemented for this class
    }

    /**
     * @brief Step with given matrices and step size.
     *
     * @param indx       Iteration index
     * @param step_size  Proximal step size (η)
     * @param At, Bt, at Current dynamics matrices
     * @param hAt, hat   Linearization matrices
     * @param z0, Sig0   Initial conditions
     * @return StepResult: (Kt, dt, At_cl, at_cl, zt, Sigt)
     */
    StepResult step(int indx, double step_size,
                    const Matrix3D& At, const Matrix3D& Bt, const Matrix3D& at,
                    const Matrix3D& hAt, const Matrix3D& hat,
                    const VectorXd& z0, const MatrixXd& Sig0) override {
        std::cout << " propagate mean " << std::endl;
        auto [zt_new, Sigt_new] = propagate_nominal(At, at, Bt, z0, Sig0);

        // Proximal averaging of dynamics
        MatrixXd Aprior = (At + hAt * step_size) / (1 + step_size);
        MatrixXd aprior = (at + hat * step_size) / (1 + step_size);

        std::cout << " Update Q, r " << std::endl;
        auto [Qt, rt] = update_Qrk(zt_new, Sigt_new, At, at, Bt, hAt, hat, step_size);

        std::cout << " Solve linear CS " << std::endl;
        auto [Kt, dt, At_cl, at_cl] = solve_linearCS_return(Aprior, Bt, aprior, Qt, rt);

        return {Kt, dt, At_cl, at_cl, zt_new, Sigt_new};
    }

protected:
    std::shared_ptr<LinearDynamics> _pdyn;
};

} // namespace vimp