/**
 * @file ProximalGradientCS.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient algorithm for nonlinear covariance steering.
 * @version 0.1
 * @date 2023-03-15
 * @copyright Copyright (c) 2023
 */

#pragma once

#include "helpers/ExperimentParams.h"
#include "LinearCovarianceSteering.h"
#include "PGCSDataRecorder.h"
#include <memory>
#include <Eigen/QR>

namespace vimp {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using gvi::Matrix3D;
using gvi::EigenWrapper;

// ==================== Type Aliases ====================

/// Step result: (Kt, dt, At, at, zt, Sigt)
using StepResult = std::tuple<Matrix3D, Matrix3D, Matrix3D, Matrix3D, Matrix3D, Matrix3D>;

/// Nominal trajectory history: (zts, Sigts)
using NominalHistory = std::tuple<std::vector<Matrix3D>, std::vector<Matrix3D>>;

/// Linear CS result: (Kt, dt, At, at)
using LinearCSResult = std::tuple<Matrix3D, Matrix3D, Matrix3D, Matrix3D>;

/**
 * @brief Base class for Proximal Gradient Covariance Steering algorithms.
 *
 * Solves nonlinear covariance steering problems by iteratively linearizing
 * and solving linear covariance steering subproblems.
 */
class ProxGradCovSteer {
public:
    ProxGradCovSteer() = default;
    virtual ~ProxGradCovSteer() = default;

    /**
     * @brief Construct from parameter object.
     */
    ProxGradCovSteer(const MatrixXd& A0,
                     const VectorXd& a0,
                     const MatrixXd& B,
                     const PGCSParams& params)
        : _nx(params.nx())
        , _nu(params.nu())
        , _nt(params.nt())
        , _eta(params.step_size())
        , _total_time(params.total_time())
        , _eps(params.eps())
        , _deltt(params.total_time() / (params.nt() - 1))
        , _stop_err(params.stop_err())
        , _backtrack_ratio(params.backtrack_ratio())
        , _max_iter(params.max_iter())
        , _max_n_backtrack(params.max_n_backtrack())
        , _z0(params.m0())
        , _zT(params.mT())
        , _Sig0(params.Sig0())
        , _SigT(params.SigT())
        , _Akt(_ei.replicate3d(A0, params.nt()))
        , _akt(_ei.replicate3d(a0, params.nt()))
        , _Bt(_ei.replicate3d(B, params.nt()))
        , _pinvBBTt(params.nx(), params.nx(), params.nt())
        , _Qkt(params.nx(), params.nx(), params.nt())
        , _Qt(params.nx(), params.nx(), params.nt())
        , _rkt(params.nx(), 1, params.nt())
        , _hAkt(params.nx(), params.nx(), params.nt())
        , _hakt(params.nx(), 1, params.nt())
        , _nTrt(params.nx(), 1, params.nt())
        , _zkt(_ei.replicate3d(params.m0(), params.nt()))
        , _Sigkt(_ei.replicate3d(params.Sig0(), params.nt()))
        , _Kt(params.nu(), params.nx(), params.nt())
        , _dt(params.nu(), 1, params.nt())
        , _linear_cs(_Akt, _Bt, _akt, params.nx(), params.nu(),
                     params.total_time(), params.nt(), params.eps(),
                     _Qkt, _rkt, params.m0(), params.Sig0(), params.mT(), params.SigT())
        , _recorder(_Akt, _Bt, _akt, _Qkt, _rkt, _Kt, _dt, _zkt, _Sigkt)
    {
        initializeCommon();
    }

    /**
     * @brief Construct with explicit parameters.
     */
    ProxGradCovSteer(const MatrixXd& A0,
                     const VectorXd& a0,
                     const MatrixXd& B,
                     double total_time,
                     int nt,
                     double eta,
                     double eps,
                     const VectorXd& z0,
                     const MatrixXd& Sig0,
                     const VectorXd& zT,
                     const MatrixXd& SigT,
                     double stop_err,
                     int max_iteration = 30)
        : _nx(A0.rows())
        , _nu(B.cols())
        , _nt(nt)
        , _eta(eta)
        , _total_time(total_time)
        , _eps(eps)
        , _deltt(total_time / (nt - 1))
        , _stop_err(stop_err)
        , _backtrack_ratio(0.5)  // default
        , _max_iter(max_iteration)
        , _max_n_backtrack(10)   // default
        , _z0(z0)
        , _zT(zT)
        , _Sig0(Sig0)
        , _SigT(SigT)
        , _Akt(_ei.replicate3d(A0, nt))
        , _akt(_ei.replicate3d(a0, nt))
        , _Bt(_ei.replicate3d(B, nt))
        , _pinvBBTt(_nx, _nx, nt)
        , _Qkt(_nx, _nx, nt)
        , _Qt(_nx, _nx, nt)
        , _rkt(_nx, 1, nt)
        , _hAkt(_nx, _nx, nt)
        , _hakt(_nx, 1, nt)
        , _nTrt(_nx, 1, nt)
        , _zkt(_ei.replicate3d(z0, nt))
        , _Sigkt(_ei.replicate3d(Sig0, nt))
        , _Kt(_nu, _nx, nt)
        , _dt(_nu, 1, nt)
        , _linear_cs(_Akt, _Bt, _akt, _nx, _nu, _total_time, nt, _eps,
                     _Qkt, _rkt, _z0, _Sig0, _zT, _SigT)
        , _recorder(_Akt, _Bt, _akt, _Qkt, _rkt, _Kt, _dt, _zkt, _Sigkt)
    {
        initializeCommon();
    }

    // ==================== Optimization Interface ====================

    /**
     * @brief Run the optimization loop.
     *
     * Iteratively linearizes, solves linear CS, and propagates the nominal.
     * @return Tuple of (Kt, dt, nominal_history)
     */
    virtual std::tuple<Matrix3D, Matrix3D, NominalHistory> optimize() {
        MatrixXd Ak_prev = _Akt;
        MatrixXd ak_prev = _akt;
        double err = 1.0;
        int i_step = 1;

        std::vector<Matrix3D> hzt, hSigzt;

        while (err > _stop_err && i_step <= _max_iter) {
            step(i_step);

            err = computeConvergenceError(Ak_prev, ak_prev);
            Ak_prev = _Akt;
            ak_prev = _akt;

            hzt.push_back(_zkt);
            hSigzt.push_back(_Sigkt);

            i_step++;
        }

        return {_Kt, _dt, std::make_tuple(hzt, hSigzt)};
    }

    /**
     * @brief Run optimization with backtracking line search.
     */
    virtual std::tuple<Matrix3D, Matrix3D, NominalHistory> backtrack() {
        return {};  // Override in derived classes
    }

    /**
     * @brief Perform one optimization step (pure virtual).
     */
    virtual void step(int indx) = 0;

    /**
     * @brief Step with given matrices, returning all results.
     */
    virtual StepResult step(int indx, double step_size,
                            const Matrix3D& At, const Matrix3D& Bt, const Matrix3D& at,
                            const Matrix3D& hAt, const Matrix3D& hat,
                            const VectorXd& z0, const MatrixXd& Sig0) {
        return {};  // Override in derived classes
    }

    /**
     * @brief Step with given matrices (without linearization).
     */
    virtual StepResult step(int indx, double step_size,
                            const Matrix3D& At, const Matrix3D& Bt, const Matrix3D& at,
                            const VectorXd& z0, const MatrixXd& Sig0) {
        return {};  // Override in derived classes
    }

    /**
     * @brief Update internal state from step result.
     */
    void update_from_step_res(const StepResult& res) {
        _Kt   = std::get<0>(res);
        _dt   = std::get<1>(res);
        _Akt  = std::get<2>(res);
        _akt  = std::get<3>(res);
        _zkt  = std::get<4>(res);
        _Sigkt = std::get<5>(res);
    }

    // ==================== Cost Matrix Updates ====================

    /**
     * @brief Update Qk and rk from current state (override in derived classes).
     */
    virtual void update_Qrk() {}

    /**
     * @brief Update Qk and rk with given matrices.
     * @return Tuple of (Qt, rt)
     */
    virtual std::tuple<Matrix3D, Matrix3D> update_Qrk(
        const Matrix3D& zt, const Matrix3D& Sigt,
        const Matrix3D& At, const Matrix3D& at, const Matrix3D& Bt,
        const Matrix3D& hAt, const Matrix3D& hat,
        const double step_size) {
        return {};  // Override in derived classes
    }

    /**
     * @brief Update Qk and rk for nonlinear dynamics.
     * @return Tuple of (Qt, rt)
     */
    virtual std::tuple<Matrix3D, Matrix3D> update_Qrk_NL(
        const Matrix3D& zt, const Matrix3D& Sigt,
        const Matrix3D& At, const Matrix3D& at, const Matrix3D& Bt,
        const Matrix3D& hAt, const Matrix3D& hat, const Matrix3D& nTrt,
        const double step_size) {
        return {};  // Override in derived classes
    }

    // ==================== Linear CS Solver ====================

    /**
     * @brief Solve linear CS and return results.
     *
     * Solves the linear covariance steering problem and computes the
     * closed-loop dynamics A_cl = A + B*K, a_cl = a + B*d.
     *
     * @return Tuple of (K, d, A_closed_loop, a_closed_loop)
     */
    LinearCSResult solve_linearCS_return(const MatrixXd& A,
                                         const MatrixXd& B,
                                         const MatrixXd& a,
                                         const MatrixXd& Q,
                                         const MatrixXd& r) {
        _linear_cs.update_params(A, B, a, Q, r);
        _linear_cs.solve();

        Matrix3D Kt = _linear_cs.Kt();
        Matrix3D dt = _linear_cs.dt();
        Matrix3D At(_nx, _nx, _nt);
        Matrix3D at(_nx, 1, _nt);

        // Compute closed-loop dynamics: A_cl = A + B*K, a_cl = a + B*d
        for (int i = 0; i < _nt; i++) {
            MatrixXd Ai_prior = _ei.decomp3d(A, _nx, _nx, i);
            MatrixXd ai_prior = _ei.decomp3d(a, _nx, 1, i);
            MatrixXd Bi = Bt_i(i);
            MatrixXd Ki = _ei.decomp3d(Kt, _nu, _nx, i);
            MatrixXd di = _ei.decomp3d(dt, _nu, 1, i);

            MatrixXd Ai_cl = Ai_prior + Bi * Ki;
            MatrixXd ai_cl = ai_prior + Bi * di;

            _ei.compress3d(Ai_cl, At, i);
            _ei.compress3d(ai_cl, at, i);
        }

        return {Kt, dt, At, at};
    }

    /**
     * @brief Solve linear CS and update internal state.
     */
    void solve_linearCS(const MatrixXd& A, const MatrixXd& B,
                        const MatrixXd& a, const MatrixXd& Q, const MatrixXd& r) {
        auto [Kt, dt, At, at] = solve_linearCS_return(A, B, a, Q, r);
        _Kt = Kt;
        _dt = dt;
        _Akt = At;
        _akt = at;
    }

    // ==================== Nominal Propagation ====================

    /**
     * @brief Propagate mean and covariance forward in time.
     *
     * Integrates the nominal trajectory using Heun's method:
     *   dz/dt = A*z + a
     *   dΣ/dt = A*Σ + Σ*A' + ε*B*B'
     *
     * @return Tuple of (zt, Sigt)
     */
    std::tuple<Matrix3D, Matrix3D> propagate_nominal(const Matrix3D& At,
                                                     const Matrix3D& at,
                                                     const Matrix3D& Bt,
                                                     const VectorXd& z0,
                                                     const MatrixXd& Sig0) {
        Matrix3D zt_new(_nx, 1, _nt);
        Matrix3D Sigt_new(_nx, _nx, _nt);
        zt_new.setZero();
        Sigt_new.setZero();

        // Initial conditions
        _ei.compress3d(z0, zt_new, 0);
        _ei.compress3d(Sig0, Sigt_new, 0);

        // Forward integration using Heun's method
        for (int i = 0; i < _nt - 1; i++) {
            // Current timestep matrices
            MatrixXd Ai = _ei.decomp3d(At, _nx, _nx, i);
            MatrixXd ai = _ei.decomp3d(at, _nx, 1, i);
            MatrixXd Bi = _ei.decomp3d(Bt, _nx, _nu, i);
            MatrixXd zi = _ei.decomp3d(zt_new, _nx, 1, i);
            MatrixXd Si = _ei.decomp3d(Sigt_new, _nx, _nx, i);

            // Next timestep matrices
            MatrixXd Ai_next = _ei.decomp3d(At, _nx, _nx, i + 1);
            MatrixXd ai_next = _ei.decomp3d(at, _nx, 1, i + 1);
            MatrixXd Bi_next = _ei.decomp3d(Bt, _nx, _nu, i + 1);

            // Predictor step (Euler)
            VectorXd z_pred = zi + _deltt * (Ai * zi + ai);
            MatrixXd S_pred = Si + _deltt * (Ai * Si + Si * Ai.transpose() + _eps * Bi * Bi.transpose());

            // Corrector step (Heun's method)
            auto [dz_i, dS_i] = computeNominalDerivatives(zi, Si, Ai, ai, Bi);
            auto [dz_next, dS_next] = computeNominalDerivatives(z_pred, S_pred, Ai_next, ai_next, Bi_next);

            VectorXd z_new = zi + _deltt * (dz_i + dz_next) / 2.0;
            MatrixXd S_new = Si + _deltt * (dS_i + dS_next) / 2.0;

            _ei.compress3d(z_new, zt_new, i + 1);
            _ei.compress3d(S_new, Sigt_new, i + 1);
        }

        return {zt_new, Sigt_new};
    }

    /**
     * @brief Propagate nominal using internal state.
     */
    void propagate_nominal() {
        auto [zt, Sigt] = propagate_nominal(_Akt, _akt, _Bt, _z0, _Sig0);
        _zkt = zt;
        _Sigkt = Sigt;
    }

    // ==================== Accessors ====================

    inline Matrix3D zkt()  const { return _zkt; }
    inline Matrix3D Sigkt() const { return _Sigkt; }
    inline Matrix3D Akt()  const { return _Akt; }
    inline Matrix3D akt()  const { return _akt; }
    inline Matrix3D hAkt() const { return _hAkt; }
    inline Matrix3D hakt() const { return _hakt; }
    inline Matrix3D Qkt()  const { return _Qkt; }
    inline Matrix3D rkt()  const { return _rkt; }

    inline MatrixXd Bt_i(int i) const {
        return _ei.decomp3d(_Bt, _nx, _nu, i);
    }

    /**
     * @brief Set a constant state cost matrix Qt.
     */
    void repliacteQt(const MatrixXd& Q0) {
        _Qt = _ei.replicate3d(Q0, _nt);
    }

protected:
    // ==================== Helper Methods ====================

    /**
     * @brief Common initialization for both constructors.
     */
    void initializeCommon() {
        // Set final time covariance
        _ei.compress3d(_SigT, _Sigkt, _nt - 1);

        // Compute pseudo-inverse of BB' at each timestep
        computePseudoInverseBBT();
    }

    /**
     * @brief Compute (BB')^+ at each timestep.
     */
    void computePseudoInverseBBT() {
        for (int i = 0; i < _nt; i++) {
            MatrixXd Bi = Bt_i(i);
            MatrixXd BBT = Bi * Bi.transpose();
            MatrixXd pinvBBTi = BBT.completeOrthogonalDecomposition().pseudoInverse();
            _ei.compress3d(pinvBBTi, _pinvBBTt, i);
        }
    }

    /**
     * @brief Compute convergence error between iterations.
     */
    double computeConvergenceError(const MatrixXd& Ak_prev, const MatrixXd& ak_prev) const {
        double err_A = (_Akt - Ak_prev).norm() / _Akt.norm() / _nt;
        double err_a = (_akt - ak_prev).norm() / _akt.norm() / _nt;
        return err_A + err_a;
    }

    /**
     * @brief Compute derivatives for nominal propagation.
     * @return Tuple of (dz/dt, dΣ/dt)
     */
    std::pair<VectorXd, MatrixXd> computeNominalDerivatives(
        const VectorXd& z, const MatrixXd& Sig,
        const MatrixXd& A, const VectorXd& a, const MatrixXd& B) const {
        VectorXd dz = A * z + a;
        MatrixXd dSig = A * Sig + Sig * A.transpose() + _eps * B * B.transpose();
        return {dz, dSig};
    }

    // ==================== Member Variables ====================

    mutable EigenWrapper _ei;

    // Problem dimensions
    int _nx, _nu, _nt;

    // Algorithm parameters
    double _eta;              ///< Step size
    double _total_time;       ///< Total time horizon
    double _eps;              ///< Noise scaling
    double _deltt;            ///< Time step
    double _stop_err;         ///< Convergence threshold
    double _backtrack_ratio;  ///< Backtracking shrink factor
    int _max_iter;            ///< Maximum iterations
    int _max_n_backtrack;     ///< Maximum backtracking steps

    // Boundary conditions
    VectorXd _z0, _zT;
    MatrixXd _Sig0, _SigT;

    // Time-varying system matrices (3D tensors)
    Matrix3D _Akt, _Bt, _akt;
    Matrix3D _pinvBBTt;       ///< Pseudo-inverse of BB' at each time

    // Cost matrices
    Matrix3D _Qkt;            ///< Iteration-dependent Q
    Matrix3D _Qt;             ///< Fixed state cost Q
    Matrix3D _rkt;            ///< Linear cost term

    // Linearization matrices
    Matrix3D _hAkt, _hakt;
    Matrix3D _nTrt;           ///< Nonlinear residual term

    // Nominal trajectory
    Matrix3D _zkt, _Sigkt;

    // Controller gains
    Matrix3D _Kt, _dt;

    // Linear CS solver
    LinearCovarianceSteering _linear_cs;

    // Data recorder
    PGCSDataRecorder _recorder;
};

} // namespace vimp