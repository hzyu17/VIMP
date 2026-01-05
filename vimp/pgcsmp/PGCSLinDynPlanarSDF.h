/**
 * @file PGCSLinDynPlanarSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient covariance steering with planar SDF obstacles and linear dynamics.
 * @version 0.1
 * @date 2023-03-15
 * @copyright Copyright (c) 2023
 */

#pragma once

// GTSAM types needed by gpmp2
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
using gtsam::Point2;
using gtsam::Point3;

#include "ProximalGradientCSLinearDyn.h"
#include "robots/PlanarPointRobotSDF_pgcs.h"
#include "helpers/hinge2Dhelper.h"
#include "helpers/CostHelper.h"

namespace vimp {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @brief Proximal Gradient Covariance Steering for linear dynamics with planar SDF obstacles.
 * 
 * Optimizes a trajectory that steers the state distribution while avoiding
 * obstacles represented by a signed distance field (SDF).
 */
class PGCSLinDynPlanarSDF : public ProxGradCovSteerLinDyn {
public:
    PGCSLinDynPlanarSDF(const MatrixXd& A0,
                        const VectorXd& a0,
                        const MatrixXd& B,
                        const PGCSParams& params,
                        const std::shared_ptr<LinearDynamics>& pdyn,
                        const gpmp2::PlanarSDF& sdf)
        : ProxGradCovSteerLinDyn(A0, a0, B, pdyn, params)
        , _sdf(sdf)
        , _eps_sdf(params.eps_sdf(), params.radius())
        , _sig_obs(params.sig_obs())
        , _pRsdf(params.eps_sdf(), params.radius())
        , _cost_helper(params.max_iter())
    {
        _pRsdf.update_sdf(sdf);
    }

    /**
     * @brief Compute total hinge loss over the trajectory.
     */
    double total_hingeloss() {
        double total = 0.0;

        for (int i = 0; i < _nt; i++) {
            MatrixXd zi = _ei.decomp3d(_zkt, _nx, 1, i);
            double hinge = computeHingeLoss(zi(0), zi(1));
            total += hinge;
        }
        return total;
    }

    /**
     * @brief Compute total control energy over the trajectory.
     */
    double total_control_energy() {
        double total = 0.0;

        for (int i = 0; i < _nt; i++) {
            MatrixXd zi = _ei.decomp3d(_zkt, _nx, 1, i);
            MatrixXd Ki = _ei.decomp3d(_Kt, _nu, _nx, i);

            VectorXd u_i = Ki * zi;
            total += u_i.squaredNorm() * _deltt;
        }
        return total;
    }

    /**
     * @brief Run the optimization loop with cost recording.
     * @return Tuple of (Kt, dt) feedback gains
     */
    std::tuple<Matrix3D, Matrix3D> optimize() override {
        double total_cost_prev = total_hingeloss() + total_control_energy();
        double err = 1.0;
        int i_step = 0;

        while (err > _stop_err && i_step < _max_iter) {
            step(i_step);

            double hingeloss = total_hingeloss();
            double control_energy = total_control_energy();
            double total_cost = hingeloss + control_energy;

            err = std::abs(total_cost - total_cost_prev);
            _cost_helper.add_cost(i_step, hingeloss, control_energy);

            total_cost_prev = total_cost;
            i_step++;
        }

        return {_Kt, _dt};
    }

    /**
     * @brief Update the quadratic cost terms Qk and rk for covariance steering.
     * 
     * Incorporates obstacle hinge loss gradients and dynamics mismatch penalties.
     */
    void update_Qrk() override {
        _Qkt.setZero();
        _rkt.setZero();

        for (int i = 0; i < _nt; i++) {
            // Extract matrices at timestep i
            MatrixXd Aki = _ei.decomp3d(_Akt, _nx, _nx, i);
            MatrixXd aki = _ei.decomp3d(_akt, _nx, 1, i);
            MatrixXd hAi = _ei.decomp3d(_hAkt, _nx, _nx, i);
            MatrixXd hai = _ei.decomp3d(_hakt, _nx, 1, i);
            MatrixXd Bi = _ei.decomp3d(_Bt, _nx, _nu, i);
            MatrixXd Qti = _ei.decomp3d(_Qt, _nx, _nx, i);
            MatrixXd pinvBBTi = _ei.decomp3d(_pinvBBTt, _nx, _nx, i);
            MatrixXd zi = _ei.decomp3d(_zkt, _nx, 1, i);

            // Dynamics mismatch: A - hA, a - ha
            MatrixXd dA = Aki - hAi;
            MatrixXd da = aki - hai;
            MatrixXd dA_T_pinvBBT = dA.transpose() * pinvBBTi;

            // Compute hinge loss and gradient
            MatrixXd J_hxy(1, _nx / 2);
            auto [hinge, grad] = hingeloss_gradient_point(zi(0), zi(1), _sdf, _eps_sdf, J_hxy);

            if (hinge > 0) {
                _ei.print_matrix(zi, "zi");
                std::cout << "hinge loss " << hinge << std::endl;
                _ei.print_matrix(J_hxy, "J_hxy");
            }

            // Build full-state gradient: [∂h/∂pos; ∂h/∂pos ⊙ vel]
            MatrixXd grad_h = buildObstacleGradient(zi, J_hxy);

            // Proximal weighting factor
            double alpha = _eta / (1.0 + _eta);
            double alpha_sq = alpha / (1.0 + _eta);

            // Qki: Hessian term (zero here) + dynamics mismatch penalty
            MatrixXd Qki = dA_T_pinvBBT * dA * alpha_sq;

            // rki: obstacle gradient + dynamics mismatch penalty
            MatrixXd rki = grad_h * hinge * _sig_obs * alpha
                         + dA_T_pinvBBT * da * alpha_sq;

            _ei.compress3d(Qki, _Qkt, i);
            _ei.compress3d(rki, _rkt, i);
        }
    }

protected:
    gpmp2::PlanarSDF _sdf;
    PlanarPRSDFExample _pRsdf;

    double _eps_sdf;   ///< SDF epsilon for hinge loss
    double _sig_obs;   ///< Obstacle cost weight (inverse covariance)

    CostHelper _cost_helper;

private:
    /**
     * @brief Compute hinge loss at a position.
     */
    double computeHingeLoss(double x, double y) {
        MatrixXd J_hxy(1, _nx / 2);
        auto [hinge, grad] = hingeloss_gradient_point(x, y, _sdf, _eps_sdf, J_hxy);
        return hinge;
    }

    /**
     * @brief Build the full-state obstacle cost gradient.
     * 
     * For a point robot, the gradient w.r.t. position is J_hxy.
     * The gradient w.r.t. velocity is J_hxy ⊙ velocity (element-wise).
     * 
     * @param zi Current state [pos; vel]
     * @param J_hxy Jacobian of hinge loss w.r.t. position
     * @return Full state gradient [∂h/∂pos; ∂h/∂vel]
     */
    MatrixXd buildObstacleGradient(const MatrixXd& zi, const MatrixXd& J_hxy) {
        const int half_nx = _nx / 2;
        MatrixXd grad_h(_nx, 1);

        VectorXd velocity = zi.block(half_nx, 0, half_nx, 1);
        grad_h.topRows(half_nx) = J_hxy.transpose();
        grad_h.bottomRows(half_nx) = J_hxy.transpose().cwiseProduct(velocity);

        return grad_h;
    }
};

} // namespace vimp