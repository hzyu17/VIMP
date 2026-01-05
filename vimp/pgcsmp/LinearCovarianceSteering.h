/**
 * @file LinearCovarianceSteering.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Linear covariance steering problem.
 * 
 * Reference:
 *   Chen, Yongxin, Tryphon T. Georgiou, and Michele Pavon.
 *   "Optimal steering of a linear stochastic system to a final probability distribution, Part I."
 *   IEEE Transactions on Automatic Control 61.5 (2015): 1158-1169.
 * 
 * @version 0.1
 * @date 2023-03-08
 * @copyright Copyright (c) 2023
 */

#pragma once

#include <Eigen/Dense>
#include "GaussianVI/helpers/EigenWrapper.h"

namespace vimp {

using namespace Eigen;
using gvi::Matrix3D;
using gvi::EigenWrapper;

/**
 * @brief Solves the linear covariance steering problem.
 * 
 * Given linear dynamics: dx = (A(t)x + B(t)u + a(t))dt + sqrt(ε)dw
 * Steers the state distribution from N(m0, Σ0) to N(m1, Σ1)
 * while minimizing integral of (x'Qx + u'u + r'x) dt.
 */
class LinearCovarianceSteering {

public:
    LinearCovarianceSteering() {}

    /**
     * @brief Construct a new Linear Covariance Steering object
     * @note All time-varying matrices should be in shape (m*n, nt)
     */
    LinearCovarianceSteering(const Matrix3D& At, const Matrix3D& Bt, const Matrix3D& at,
                             int nx, int nu, double T, int nt, double epsilon,
                             const Matrix3D& Qt, const Matrix3D& rt,
                             const VectorXd& m0, const MatrixXd& Sig0,
                             const VectorXd& m1, const MatrixXd& Sig1)
        : _At(At), _Bt(Bt), _at(at), _rt(rt), _Qt(Qt)
        , _nx(nx), _nu(nu), _nt(nt)
        , _T(T), _eps(epsilon), _delta_t(T / (nt - 1))
        , _m0(m0), _m1(m1), _Sig0(Sig0), _Sig1(Sig1)
        , _Phi(MatrixXd::Identity(2 * nx, 2 * nx))
        , _Phi11(MatrixXd::Zero(nx, nx))
        , _Phi12(MatrixXd::Zero(nx, nx))
        , _Mt(2 * nx, 2 * nx, nt)
        , _Pit(nx, nx, nt)
        , _Kt(nu, nx, nt)
        , _dt(nu, 1, nt)
    {
        compute_Phi();
    }

    void update_params(const MatrixXd& At, const MatrixXd& Bt, const MatrixXd& at,
                       const MatrixXd& Qt, const MatrixXd& rt) {
        _At = At;
        _Bt = Bt;
        _at = at;
        _Qt = Qt;
        _rt = rt;
        compute_Phi();
    }

    /**
     * @brief Compute the state transition matrix Φ for the Hamiltonian system.
     * 
     * Constructs M(t) = [A, -BB'; -Q, -A'] and integrates using Heun's method.
     */
    void compute_Phi() {
        // Build Hamiltonian matrix M(t) at each time step
        buildHamiltonianMatrices();
        
        // Integrate Φ' = M(t)Φ using Heun's method
        _Phi = MatrixXd::Identity(2 * _nx, 2 * _nx);
        
        for (int i = 0; i < _nt - 1; i++) {
            MatrixXd Mi = getM(i);
            MatrixXd Mi_next = getM(i + 1);
            
            // Heun's method (predictor-corrector)
            MatrixXd Phi_pred = _Phi + Mi * _Phi * _delta_t;
            _Phi = _Phi + _delta_t * (Mi * _Phi + Mi_next * Phi_pred) / 2.0;
        }

        _Phi11 = _Phi.topLeftCorner(_nx, _nx);
        _Phi12 = _Phi.topRightCorner(_nx, _nx);
    }

    std::tuple<Matrix3D, Matrix3D> solve_return() {
        Matrix3D Kt(_nu, _nx, _nt);
        Matrix3D dt(_nu, 1, _nt);

        // Step 1: Integrate the adjoint variable s for the affine terms
        VectorXd s = integrateAffineAdjoint();

        // Step 2: Solve for initial costate λ(0) from boundary conditions
        VectorXd rhs = _m1 - _Phi11 * _m0 - s.head(_nx);
        VectorXd Lambda_0 = _Phi12.colPivHouseholderQr().solve(rhs);

        // Step 3: Forward integrate the state-costate system
        auto [xt, lbdt] = integrateStateCostate(Lambda_0);

        // Step 4: Compute open-loop control v = -B'λ
        MatrixXd v = computeOpenLoopControl(lbdt);

        // Step 5: Solve Riccati equation for feedback gain Π(t)
        solveRiccati();

        // Step 6: Assemble feedback controller: u = Kx + d
        for (int i = 0; i < _nt; i++) {
            MatrixXd Bi = getB(i);
            MatrixXd Pii = getPi(i);
            
            MatrixXd Ki = -Bi.transpose() * Pii;
            _ei.compress3d(Ki, Kt, i);
            dt.col(i) = v.col(i) + Bi.transpose() * Pii * xt.col(i);
        }

        return {Kt, dt};
    }

    void solve() {
        auto [Kt, dt] = solve_return();
        _Kt = Kt;
        _dt = dt;
    }

    // ==================== Accessors ====================
    
    inline MatrixXd Phi()   { return _Phi; }
    inline MatrixXd Phi11() { return _Phi11; }
    inline MatrixXd Phi12() { return _Phi12; }
    
    inline Matrix3D Pit()        { return _Pit; }
    inline MatrixXd Pit(int i)   { return _ei.decomp3d(_Pit, _nx, _nx, i); }
    
    inline Matrix3D Qt()         { return _Qt; }
    inline MatrixXd Qt(int i)    { return _ei.decomp3d(_Qt, _nx, _nx, i); }
    
    inline Matrix3D rt()         { return _rt; }
    inline MatrixXd rt(int i)    { return _rt.col(i); }
    
    inline Matrix3D Kt()         { return _Kt; }
    inline MatrixXd Kt(int i)    { return _ei.decomp3d(_Kt, _nu, _nx, i); }
    
    inline Matrix3D dt()         { return _dt; }
    inline Matrix3D Mt()         { return _Mt; }
    inline Matrix3D at()         { return _at; }
    inline MatrixXd at(int i)    { return _at.col(i); }
    inline Matrix3D Bt()         { return _Bt; }

    Matrix3D At() { return _At; }
    
    MatrixXd At(int i) {
        MatrixXd Ai;
        _ei.decomp3d(_At, Ai, _nx, _nx, i);
        return Ai;
    }

    inline MatrixXd Bt(int i) {
        MatrixXd Bi;
        _ei.decomp3d(_Bt, Bi, _nx, _nu, i);
        return Bi;
    }

    MatrixXd Mt(int i) {
        MatrixXd Mi = MatrixXd::Zero(2 * _nx, 2 * _nx);
        _ei.decomp3d(_Mt, Mi, 2 * _nx, 2 * _nx, i);
        return Mi;
    }

private:
    // ==================== Helper Methods ====================

    /** @brief Extract A(t_i) from the 3D tensor */
    MatrixXd getA(int i) const { return _ei.decomp3d(_At, _nx, _nx, i); }
    
    /** @brief Extract B(t_i) from the 3D tensor */
    MatrixXd getB(int i) const { return _ei.decomp3d(_Bt, _nx, _nu, i); }
    
    /** @brief Extract Q(t_i) from the 3D tensor */
    MatrixXd getQ(int i) const { return _ei.decomp3d(_Qt, _nx, _nx, i); }
    
    /** @brief Extract M(t_i) from the 3D tensor */
    MatrixXd getM(int i) const { return _ei.decomp3d(_Mt, 2 * _nx, 2 * _nx, i); }
    
    /** @brief Extract Π(t_i) from the 3D tensor */
    MatrixXd getPi(int i) const { return _ei.decomp3d(_Pit, _nx, _nx, i); }

    /**
     * @brief Build the Hamiltonian matrix M(t) = [A, -BB'; -Q, -A'] at each time step
     */
    void buildHamiltonianMatrices() {
        MatrixXd Mi(2 * _nx, 2 * _nx);
        
        for (int i = 0; i < _nt; i++) {
            MatrixXd Ai = getA(i);
            MatrixXd Bi = getB(i);
            MatrixXd Qi = getQ(i);

            Mi.topLeftCorner(_nx, _nx)     =  Ai;
            Mi.topRightCorner(_nx, _nx)    = -Bi * Bi.transpose();
            Mi.bottomLeftCorner(_nx, _nx)  = -Qi;
            Mi.bottomRightCorner(_nx, _nx) = -Ai.transpose();
            
            _ei.compress3d(Mi, _Mt, i);
        }
    }

    /**
     * @brief Integrate the affine adjoint equation for s(t).
     * @return Final value s(T)
     */
    VectorXd integrateAffineAdjoint() {
        VectorXd s = VectorXd::Zero(2 * _nx);
        
        // Stack affine terms: [a(t); -r(t)]
        MatrixXd a_r(2 * _nx, _nt);
        a_r.topRows(_nx) = _at;
        a_r.bottomRows(_nx) = -_rt;

        for (int i = 0; i < _nt - 1; i++) {
            MatrixXd Mi = getM(i);
            MatrixXd Mi_next = getM(i + 1);
            
            VectorXd f_i = Mi * s + a_r.col(i);
            VectorXd s_pred = s + f_i * _delta_t;
            VectorXd f_next = Mi_next * s_pred + a_r.col(i + 1);
            
            s = s + _delta_t * (f_i + f_next) / 2.0;
        }
        return s;
    }

    /**
     * @brief Forward integrate the coupled state-costate ODE.
     * @param Lambda_0 Initial costate
     * @return Tuple of (state trajectory, costate trajectory)
     */
    std::pair<MatrixXd, MatrixXd> integrateStateCostate(const VectorXd& Lambda_0) {
        // Stack affine terms
        MatrixXd a_r(2 * _nx, _nt);
        a_r.topRows(_nx) = _at;
        a_r.bottomRows(_nx) = -_rt;

        // Initialize [x(0); λ(0)]
        MatrixXd Xt(2 * _nx, _nt);
        Xt.col(0) << _m0, Lambda_0;

        // Heun's method integration
        for (int i = 0; i < _nt - 1; i++) {
            MatrixXd Mi = getM(i);
            MatrixXd Mi_next = getM(i + 1);
            
            VectorXd f_i = Mi * Xt.col(i) + a_r.col(i);
            VectorXd X_pred = Xt.col(i) + _delta_t * f_i;
            VectorXd f_next = Mi_next * X_pred + a_r.col(i + 1);
            
            Xt.col(i + 1) = Xt.col(i) + _delta_t * (f_i + f_next) / 2.0;
        }

        return {Xt.topRows(_nx), Xt.bottomRows(_nx)};
    }

    /**
     * @brief Compute open-loop control v = -B'λ
     */
    MatrixXd computeOpenLoopControl(const MatrixXd& lbdt) {
        MatrixXd v(_nu, _nt);
        for (int i = 0; i < _nt; i++) {
            v.col(i) = -getB(i).transpose() * lbdt.col(i);
        }
        return v;
    }

    /**
     * @brief Solve the Riccati equation backward in time.
     * 
     * Computes Π(t) satisfying: -Π̇ = A'Π + ΠA - ΠBB'Π + Q
     * with boundary condition derived from covariance steering.
     */
    void solveRiccati() {
        // Compute initial condition Π(0) from boundary constraints
        MatrixXd Sig0_sqrt = _ei.psd_sqrtm(_Sig0);
        MatrixXd Sig0_inv_sqrt = _ei.psd_invsqrtm(_Sig0);
        MatrixXd Phi12_inv = _Phi12.inverse();
        
        MatrixXd temp = (_eps * _eps / 4.0) * MatrixXd::Identity(_nx, _nx)
                      + Sig0_sqrt * Phi12_inv * _Sig1 * Phi12_inv.transpose() * Sig0_sqrt;

        MatrixXd Pi_0 = (_eps / 2.0) * _Sig0.inverse() 
                      - Phi12_inv * _Phi11
                      - Sig0_inv_sqrt * _ei.psd_sqrtm(temp) * Sig0_inv_sqrt;
        
        // Symmetrize
        Pi_0 = (Pi_0 + Pi_0.transpose()) / 2.0;
        _ei.compress3d(Pi_0, _Pit, 0);

        // Forward integrate Riccati (note: this is actually backward in the original time)
        for (int i = 0; i < _nt - 1; i++) {
            MatrixXd Ai = getA(i);
            MatrixXd Bi = getB(i);
            MatrixXd Qi = getQ(i);
            MatrixXd Pii = getPi(i);
            
            MatrixXd Ai_next = getA(i + 1);
            MatrixXd Bi_next = getB(i + 1);
            MatrixXd Qi_next = getQ(i + 1);

            // Riccati dynamics: Π̇ = -(A'Π + ΠA - ΠBB'Π + Q)
            auto riccati_rhs = [&](const MatrixXd& P, const MatrixXd& A, 
                                   const MatrixXd& B, const MatrixXd& Q) {
                return A.transpose() * P + P * A - P * B * B.transpose() * P + Q;
            };

            MatrixXd dPi = riccati_rhs(Pii, Ai, Bi, Qi);
            MatrixXd Pi_pred = Pii - _delta_t * dPi;
            MatrixXd dPi_next = riccati_rhs(Pi_pred, Ai_next, Bi_next, Qi_next);
            
            MatrixXd Pi_next = Pii - _delta_t * (dPi + dPi_next) / 2.0;
            _ei.compress3d(Pi_next, _Pit, i + 1);
        }
    }

    // ==================== Member Variables ====================
    
    // System matrices (time-varying, stored as 3D tensors)
    Matrix3D _At, _Bt, _at;     // Dynamics: dx = (Ax + Bu + a)dt
    Matrix3D _Qt, _rt;          // Cost: ∫(x'Qx + r'x)dt

    // Problem dimensions
    int _nx, _nu, _nt;
    double _T, _eps, _delta_t;

    // Boundary conditions
    VectorXd _m0, _m1;
    MatrixXd _Sig0, _Sig1;

    // Computed quantities
    MatrixXd _Phi, _Phi11, _Phi12;  // State transition matrix partitions
    Matrix3D _Mt;                   // Hamiltonian matrix
    Matrix3D _Pit;                  // Riccati solution

    // Feedback controller: u = Kt * x + dt
    Matrix3D _Kt, _dt;

    // Utility
    mutable EigenWrapper _ei;
};

} // namespace vimp