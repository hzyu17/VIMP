/**
 * @file PGCSLinDynRobotSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Template class for proximal gradient covariance steering with SDF obstacles
 *        and linear robot dynamics.
 * @version 0.1
 * @date 2023-03-15
 * @copyright Copyright (c) 2023
 */

#pragma once

#include "pgcsmp/ProximalGradientCSLinearDyn.h"
#include "helpers/CostHelper.h"
#include <iomanip>

namespace vimp {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @brief Proximal Gradient Covariance Steering for linear dynamics with robot SDF obstacles.
 *
 * @tparam RobotSDF Robot model with signed distance field collision checking
 */
template <typename RobotSDF>
class PGCSLinDynRobotSDF : public ProxGradCovSteerLinDyn {
public:
    virtual ~PGCSLinDynRobotSDF() = default;

    PGCSLinDynRobotSDF(const MatrixXd& A0,
                       const VectorXd& a0,
                       const MatrixXd& B,
                       const std::shared_ptr<LinearDynamics>& pdyn,
                       PGCSParams& params)
        : ProxGradCovSteerLinDyn(A0, a0, B, pdyn, params)
        , _robot_sdf(params.eps_sdf(), params.radius(), params.map_name(), params.sdf_file())
        , _eps_sdf(params.eps_sdf())
        , _sig_obs(params.sig_obs())
        , _cost_helper(_max_iter)
    {}


    int nx() const { return _nx; }
    int nt() const { return _nt; }
    int nu() const { return _nu; }

    // ==================== Cost Functions ====================

    /**
     * @brief Compute total hinge loss for obstacle avoidance.
     */
    double hingeloss(const Matrix3D& zt, const Matrix3D& Sigt) {
        double total = 0.0;
        const int n_spheres = _robot_sdf.RobotModel().nr_body_spheres();
        MatrixXd Sig_obs = _sig_obs * MatrixXd::Identity(n_spheres, n_spheres);

        for (int i = 0; i < _nt; i++) {
            MatrixXd zi = _ei.decomp3d(zt, _nx, 1, i);
            VectorXd hinge = computeHingeVector(zi);
            total += hinge.transpose() * Sig_obs * hinge;
        }
        return total;
    }

    double hingeloss() {
        return hingeloss(_zkt, _Sigkt);
    }

    /**
     * @brief Compute total control energy.
     */
    double control_energy(const Matrix3D& zt, const Matrix3D& Sigt,
                          const Matrix3D& Kt, const Matrix3D& dt) {
        double total = 0.0;

        for (int i = 0; i < _nt; i++) {
            MatrixXd zi = _ei.decomp3d(zt, _nx, 1, i);
            MatrixXd Ki = _ei.decomp3d(Kt, _nu, _nx, i);
            MatrixXd di = _ei.decomp3d(dt, _nu, 1, i);

            VectorXd u_i = Ki * zi + di;
            total += u_i.squaredNorm() * _deltt;
        }
        return total;
    }

    double control_energy() {
        return control_energy(_zkt, _Sigkt, _Kt, _dt);
    }

    /**
     * @brief Compute total cost (hinge loss + control energy).
     */
    double total_cost(const Matrix3D& zt, const Matrix3D& Sigt,
                      const Matrix3D& Kt, const Matrix3D& dt) {
        return hingeloss(zt, Sigt) + control_energy(zt, Sigt, Kt, dt);
    }

    // ==================== Optimization ====================

    /**
     * @brief Run optimization with cost recording.
     * @return Tuple of (Kt, dt, nominal_history)
     */
    std::tuple<Matrix3D, Matrix3D, NominalHistory> optimize() override {
        double total_cost_prev = 1e6;
        double err = 1.0;
        int i_step = 0;

        std::vector<Matrix3D> v_zt, v_Sigzt;

        while (err > _stop_err && i_step < _max_iter) {
            printIterationHeader(i_step);

            step(i_step);

            double hl = hingeloss();
            double eu = control_energy();
            double cost = hl + eu;

            err = std::abs(cost - total_cost_prev);
            _cost_helper.add_cost(i_step, hl, eu);
            total_cost_prev = cost;

            v_zt.push_back(_zkt);
            v_Sigzt.push_back(_Sigkt);

            i_step++;
        }

        return {_Kt, _dt, std::make_tuple(v_zt, v_Sigzt)};
    }

    /**
     * @brief Run optimization with backtracking line search.
     * @return Tuple of (Kt, dt, nominal_history)
     */
    std::tuple<Matrix3D, Matrix3D, NominalHistory> backtrack() override {
        double total_cost_prev = 1e9;
        double err = 1.0;
        int i_step = 0;

        std::vector<Matrix3D> v_zt, v_Sigzt;

        while (shouldContinueBacktrack(err, i_step)) {
            printIterationHeader(i_step);

            auto result = backtrackLineSearch(i_step, total_cost_prev);
            auto& [step_result, new_cost, found_improvement] = result;

            update_from_step_res(step_result);

            err = total_cost_prev - new_cost;
            total_cost_prev = new_cost;

            v_zt.push_back(_zkt);
            v_Sigzt.push_back(_Sigkt);

            i_step++;
        }

        return {_Kt, _dt, std::make_tuple(v_zt, v_Sigzt)};
    }

    // ==================== Cost Matrix Updates ====================

    std::tuple<Matrix3D, Matrix3D> update_Qrk_NL(
        const Matrix3D& zt, const Matrix3D& Sigt,
        const Matrix3D& At, const Matrix3D& at, const Matrix3D& Bt,
        const Matrix3D& hAt, const Matrix3D& hat,
        const Matrix3D& nTrt, const double step_size) override
    {
        // Not implemented for this class
        return {};
    }

    /**
     * @brief Update quadratic cost terms Qk and rk.
     *
     * Incorporates obstacle hinge loss gradients and dynamics mismatch penalties.
     */
    std::tuple<Matrix3D, Matrix3D> update_Qrk(
        const Matrix3D& zt, const Matrix3D& Sigt,
        const Matrix3D& At, const Matrix3D& at, const Matrix3D& Bt,
        const Matrix3D& hAt, const Matrix3D& hat,
        const double step_size) override
    {
        Matrix3D Qt(_nx, _nx, _nt);
        Matrix3D rt(_nx, 1, _nt);
        Qt.setZero();
        rt.setZero();

        const int n_spheres = _robot_sdf.RobotModel().nr_body_spheres();
        MatrixXd Sig_obs = _sig_obs * MatrixXd::Identity(n_spheres, n_spheres);

        // Proximal weighting factors
        const double alpha = step_size / (1.0 + step_size);
        const double alpha_sq = alpha / (1.0 + step_size);

        for (int i = 0; i < _nt; i++) {
            // Extract matrices at timestep i
            MatrixXd Ai = _ei.decomp3d(At, _nx, _nx, i);
            MatrixXd ai = _ei.decomp3d(at, _nx, 1, i);
            MatrixXd hAi = _ei.decomp3d(hAt, _nx, _nx, i);
            MatrixXd hai = _ei.decomp3d(hat, _nx, 1, i);
            MatrixXd Bi = _ei.decomp3d(Bt, _nx, _nu, i);
            MatrixXd pinvBBTi = _ei.decomp3d(_pinvBBTt, _nx, _nx, i);
            MatrixXd zi = _ei.decomp3d(zt, _nx, 1, i);

            // Dynamics mismatch terms
            MatrixXd dA = Ai - hAi;
            MatrixXd da = ai - hai;
            MatrixXd dA_T_pinvBBT = dA.transpose() * pinvBBTi;

            // Compute hinge loss and Jacobian (use topRows for matrix)
            std::tuple<VectorXd, MatrixXd> hingeloss_gradient;
            hingeloss_gradient = _robot_sdf.hinge_jacobian(zi.topRows(_nx / 2));
            
            VectorXd hinge = std::get<0>(hingeloss_gradient);
            MatrixXd J_hxy = std::get<1>(hingeloss_gradient);

            // Build obstacle gradient (position only, velocity terms zero)
            MatrixXd grad_h = MatrixXd::Zero(n_spheres, _nx);
            grad_h.leftCols(_nx / 2) = J_hxy;

            // Hessian is zero (Gauss-Newton approximation)
            MatrixXd Hess = MatrixXd::Zero(_nx, _nx);

            // Qki: Hessian + dynamics mismatch penalty
            MatrixXd Qki = Hess * alpha + dA_T_pinvBBT * dA * alpha_sq;

            // rki: obstacle gradient + dynamics mismatch penalty
            MatrixXd rki = grad_h.transpose() * Sig_obs * hinge * alpha
                         + dA_T_pinvBBT * da * alpha_sq;

            _ei.compress3d(Qki, Qt, i);
            _ei.compress3d(rki, rt, i);
        }

        return {Qt, rt};
    }

    void update_Qrk() override {
        std::tuple<Matrix3D, Matrix3D> Qtrt;
        Qtrt = update_Qrk(_zkt, _Sigkt, _Akt, _akt, _Bt, _hAkt, _hakt, _eta);
        _Qkt = std::get<0>(Qtrt);
        _rkt = std::get<1>(Qtrt);
    }

    // ==================== Utilities ====================

    void save_costs(const std::string& filename) {
        _cost_helper.save_costs(filename);
    }

    template <typename SDF>
    void update_sdf(const SDF& sdf) {
        _robot_sdf.update_sdf(sdf);
    }

protected:
    RobotSDF _robot_sdf;
    double _eps_sdf;
    double _sig_obs;  ///< Obstacle cost weight (inverse covariance)
    CostHelper _cost_helper;

private:
    // ==================== Helper Methods ====================

    /**
     * @brief Compute hinge loss vector for all robot spheres at state zi.
     */
    VectorXd computeHingeVector(const MatrixXd& zi) {
        std::tuple<VectorXd, MatrixXd> hingeloss_gradient;
        hingeloss_gradient = _robot_sdf.hinge_jacobian(zi.topRows(_nx / 2));
        return std::get<0>(hingeloss_gradient);
    }

    /**
     * @brief Print iteration header for logging.
     */
    void printIterationHeader(int iter) {
        std::cout << "================ iter " << iter << " ================" << std::endl;
    }

    /**
     * @brief Check if backtracking optimization should continue.
     */
    bool shouldContinueBacktrack(double err, int i_step) {
        return (err < 0 || err > _stop_err) && i_step < _max_iter;
    }

    /**
     * @brief Perform backtracking line search for one iteration.
     * @return Tuple of (best_step_result, best_cost, found_strict_improvement)
     */
    std::tuple<StepResult, double, bool> backtrackLineSearch(int i_step, double cost_prev) {
        double step_size = _eta;
        double best_cost = 1e9;
        StepResult best_result;
        bool found_improvement = false;

        for (int i_bt = 0; i_bt < _max_n_backtrack; i_bt++) {
            std::cout << " ----- backtracking " << i_bt << " ----- " << std::endl;

            // Take tentative step
            StepResult result = this->step(i_step, step_size,
                                           _Akt, _Bt, _akt, _hAkt, _hakt, _z0, _Sig0);

            // Extract results
            Matrix3D Kt = std::get<0>(result);
            Matrix3D dt = std::get<1>(result);
            Matrix3D zt = std::get<4>(result);
            Matrix3D Sigt = std::get<5>(result);

            double cost = total_cost(zt, Sigt, Kt, dt);
            _cost_helper.add_cost(i_step, hingeloss(zt, Sigt), control_energy(zt, Sigt, Kt, dt));

            std::cout << " total cost " << std::fixed << std::setprecision(4) << cost << std::endl;

            // Check for improvement
            if (cost < cost_prev) {
                return {result, cost, true};
            }

            // Track best result seen
            if (cost < best_cost) {
                best_cost = cost;
                best_result = result;
            }

            // Shrink step size
            step_size *= _backtrack_ratio;
        }

        // No strict improvement found, return best attempt
        std::cout << "no better step size found, using smallest step" << std::endl;
        return {best_result, best_cost, false};
    }
};

} // namespace vimp