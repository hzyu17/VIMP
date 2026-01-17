/**
 * @file Quad3D_linearization.h
 * @brief Linearization of 3D quadrotor dynamics for LTV-GVIMP
 * 
 * State: z = [x, y, z, φ, θ, ψ, vx, vy, vz, p, q, r]^T  (12 states)
 *   - (x, y, z): world frame position
 *   - (φ, θ, ψ): roll, pitch, yaw (ZYX Euler angles)
 *   - (vx, vy, vz): body frame velocities
 *   - (p, q, r): body frame angular rates
 * 
 * Input: u = [T, τx, τy, τz]^T  (4 inputs)
 *   - T: total thrust (body z-axis)
 *   - τx, τy, τz: torques about body axes
 * 
 * Dynamics:
 *   ṗ_world = R(φ,θ,ψ) · v_body
 *   η̇ = T(φ,θ) · ω
 *   v̇_body = -ω × v_body + R^T · g_world + F/m
 *   ω̇ = J^{-1}(-ω × Jω + τ)
 */

#pragma once

#include <vector>
#include <cmath>
#include <tuple>
#include <Eigen/Dense>

namespace vimp {

// Physical parameters (can be overridden)
struct Quad3DParams {
    double g = 9.81;       // gravity
    double m = 1.0;        // mass [kg]
    double l = 0.25;       // arm length [m]
    double Jx = 0.02;      // moment of inertia about x [kg·m²]
    double Jy = 0.02;      // moment of inertia about y [kg·m²]
    double Jz = 0.04;      // moment of inertia about z [kg·m²]
    double c_tau = 0.01;   // torque coefficient
};

/**
 * @brief Compute rotation matrix from body to world frame (ZYX Euler convention)
 */
inline Eigen::Matrix3d rotation_matrix_zyx(double phi, double theta, double psi) {
    const double cp = std::cos(phi),   sp = std::sin(phi);
    const double ct = std::cos(theta), st = std::sin(theta);
    const double cy = std::cos(psi),   sy = std::sin(psi);
    
    Eigen::Matrix3d R;
    R << cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp,
         sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp,
         -st,    ct*sp,             ct*cp;
    return R;
}

/**
 * @brief Linearize 3D quadrotor dynamics along a trajectory
 * 
 * @param zt Trajectory matrix [nt x 12], each row is a state
 * @param params Physical parameters
 * @return tuple of (A matrices, B matrices, affine terms)
 */
inline std::tuple<std::vector<Eigen::MatrixXd>, 
                  std::vector<Eigen::MatrixXd>, 
                  std::vector<Eigen::VectorXd>>
quad3d_linearization_deterministic(const Eigen::MatrixXd& zt,
                                   const Quad3DParams& params = Quad3DParams{}) {
    using namespace Eigen;
    
    const int nt = zt.rows();
    constexpr int nx = 12;  // state dimension
    constexpr int nu = 4;   // input dimension
    
    // State indices
    enum StateIdx { X=0, Y=1, Z=2, PHI=3, THETA=4, PSI=5, 
                    VX=6, VY=7, VZ=8, P=9, Q=10, R=11 };
    
    std::vector<MatrixXd> hA(nt, MatrixXd::Zero(nx, nx));
    std::vector<MatrixXd> hB(nt, MatrixXd::Zero(nx, nu));
    std::vector<VectorXd> ha(nt, VectorXd::Zero(nx));
    
    // Gyroscopic coupling coefficients
    const double k1 = (params.Jy - params.Jz) / params.Jx;
    const double k2 = (params.Jz - params.Jx) / params.Jy;
    const double k3 = (params.Jx - params.Jy) / params.Jz;
    
    for (int i = 0; i < nt; ++i) {
        // ===================== Extract state =====================
        const double phi   = zt(i, PHI);    // roll
        const double theta = zt(i, THETA);  // pitch
        const double psi   = zt(i, PSI);    // yaw
        const double vx    = zt(i, VX);     // body velocity x
        const double vy    = zt(i, VY);     // body velocity y
        const double vz    = zt(i, VZ);     // body velocity z
        const double p     = zt(i, P);      // roll rate
        const double q     = zt(i, Q);      // pitch rate
        const double r     = zt(i, R);      // yaw rate
        
        // ===================== Trigonometric terms =====================
        const double cp = std::cos(phi),   sp = std::sin(phi);
        const double ct = std::cos(theta), st = std::sin(theta);
        const double cy = std::cos(psi),   sy = std::sin(psi);
        const double tt = std::tan(theta);
        const double sect = 1.0 / ct;
        
        // ===================== Rotation matrix =====================
        const Matrix3d Rot = rotation_matrix_zyx(phi, theta, psi);
        const Vector3d v_body(vx, vy, vz);
        
        // ===================== Gravity in body frame =====================
        // g_world = [0, 0, -g]^T
        // g_body = R^T * g_world
        const double gx_b =  params.g * st;
        const double gy_b = -params.g * ct * sp;
        const double gz_b = -params.g * ct * cp;
        
        // ===================== Nonlinear dynamics f(z) =====================
        VectorXd fz(nx);
        
        // Position dynamics: ṗ = R * v_body
        fz.segment<3>(X) = Rot * v_body;
        
        // Euler angle dynamics: η̇ = T(η) * ω
        // T(φ,θ) = [1,  sp*tt,  cp*tt ]
        //          [0,  cp,     -sp   ]
        //          [0,  sp/ct,  cp/ct ]
        fz(PHI)   = p + q*sp*tt + r*cp*tt;
        fz(THETA) = q*cp - r*sp;
        fz(PSI)   = q*sp*sect + r*cp*sect;
        
        // Body velocity dynamics: v̇ = -ω × v + g_body + F/m
        // (thrust term goes in B matrix)
        fz(VX) = q*vz - r*vy + gx_b;
        fz(VY) = r*vx - p*vz + gy_b;
        fz(VZ) = p*vy - q*vx + gz_b;
        
        // Angular velocity dynamics: ω̇ = J^{-1}(-ω × Jω + τ)
        // (torque terms go in B matrix)
        fz(P) = k1 * q * r;
        fz(Q) = k2 * p * r;
        fz(R) = k3 * p * q;
        
        // ===================== Jacobian A = ∂f/∂z =====================
        MatrixXd A = MatrixXd::Zero(nx, nx);
        
        // --- Partial derivatives of R w.r.t. Euler angles ---
        Matrix3d dR_dphi, dR_dtheta, dR_dpsi;
        
        dR_dphi << 0,  cy*st*cp + sy*sp,  -cy*st*sp + sy*cp,
                   0,  sy*st*cp - cy*sp,  -sy*st*sp - cy*cp,
                   0,  ct*cp,             -ct*sp;
        
        dR_dtheta << -cy*st,  cy*ct*sp,  cy*ct*cp,
                     -sy*st,  sy*ct*sp,  sy*ct*cp,
                     -ct,    -st*sp,    -st*cp;
        
        dR_dpsi << -sy*ct,  -sy*st*sp - cy*cp,  -sy*st*cp + cy*sp,
                    cy*ct,   cy*st*sp - sy*cp,   cy*st*cp + sy*sp,
                    0,       0,                  0;
        
        // ∂(ṗ)/∂(φ,θ,ψ) = ∂R/∂(·) * v_body
        A.block<3,1>(X, PHI)   = dR_dphi * v_body;
        A.block<3,1>(X, THETA) = dR_dtheta * v_body;
        A.block<3,1>(X, PSI)   = dR_dpsi * v_body;
        
        // ∂(ṗ)/∂(vx,vy,vz) = R
        A.block<3,3>(X, VX) = Rot;
        
        // --- Euler angle rate equations ---
        // φ̇ = p + q*sp*tt + r*cp*tt
        A(PHI, PHI)   = q*cp*tt - r*sp*tt;
        A(PHI, THETA) = (q*sp + r*cp) * sect * sect;
        A(PHI, P)     = 1.0;
        A(PHI, Q)     = sp * tt;
        A(PHI, R)     = cp * tt;
        
        // θ̇ = q*cp - r*sp
        A(THETA, PHI) = -q*sp - r*cp;
        A(THETA, Q)   = cp;
        A(THETA, R)   = -sp;
        
        // ψ̇ = q*sp*sect + r*cp*sect
        A(PSI, PHI)   = (q*cp - r*sp) * sect;
        A(PSI, THETA) = (q*sp + r*cp) * sect * tt;
        A(PSI, Q)     = sp * sect;
        A(PSI, R)     = cp * sect;
        
        // --- Body velocity equations ---
        // v̇x = q*vz - r*vy + g*st
        A(VX, THETA) = params.g * ct;
        A(VX, VY)    = -r;
        A(VX, VZ)    = q;
        A(VX, Q)     = vz;
        A(VX, R)     = -vy;
        
        // v̇y = r*vx - p*vz - g*ct*sp
        A(VY, PHI)   = -params.g * ct * cp;
        A(VY, THETA) = params.g * st * sp;
        A(VY, VX)    = r;
        A(VY, VZ)    = -p;
        A(VY, P)     = -vz;
        A(VY, R)     = vx;
        
        // v̇z = p*vy - q*vx - g*ct*cp
        A(VZ, PHI)   = params.g * ct * sp;
        A(VZ, THETA) = params.g * st * cp;
        A(VZ, VX)    = -q;
        A(VZ, VY)    = p;
        A(VZ, P)     = vy;
        A(VZ, Q)     = -vx;
        
        // --- Angular velocity equations ---
        // ṗ = k1*q*r
        A(P, Q) = k1 * r;
        A(P, R) = k1 * q;
        
        // q̇ = k2*p*r
        A(Q, P) = k2 * r;
        A(Q, R) = k2 * p;
        
        // ṙ = k3*p*q
        A(R, P) = k3 * q;
        A(R, Q) = k3 * p;
        
        hA[i] = A;
        
        // ===================== Input matrix B =====================
        // u = [T, τx, τy, τz]
        MatrixXd B = MatrixXd::Zero(nx, nu);
        B(VZ, 0) = 1.0 / params.m;     // thrust → v̇z
        B(P, 1)  = 1.0 / params.Jx;    // τx → ṗ
        B(Q, 2)  = 1.0 / params.Jy;    // τy → q̇
        B(R, 3)  = 1.0 / params.Jz;    // τz → ṙ
        
        hB[i] = B;
        
        // ===================== Affine term =====================
        // ha = f(z) - A*z
        ha[i] = fz - A * zt.row(i).transpose();
    }
    
    return std::make_tuple(hA, hB, ha);
}

/**
 * @brief Convert 4 motor forces to [T, τx, τy, τz]
 * 
 * Motor layout (+ configuration, looking from above):
 *       1(front)
 *          |
 *    4 ----+---- 2
 *          |
 *       3(back)
 * 
 * @param motor_forces [f1, f2, f3, f4]
 * @param params Physical parameters
 * @return [T, τx, τy, τz]
 */
inline Eigen::Vector4d motor_to_wrench(const Eigen::Vector4d& motor_forces,
                                       const Quad3DParams& params = Quad3DParams{}) {
    Eigen::Matrix4d M;
    M << 1.0,       1.0,       1.0,        1.0,        // T = sum of forces
         0.0,      -params.l,  0.0,        params.l,   // τx = l*(f4-f2)
         params.l,  0.0,      -params.l,   0.0,        // τy = l*(f1-f3)
         params.c_tau, -params.c_tau, params.c_tau, -params.c_tau;  // τz from drag
    
    return M * motor_forces;
}

/**
 * @brief Get B matrix that maps motor forces directly to state derivatives
 * 
 * This combines the motor-to-wrench mapping with the wrench-to-acceleration mapping.
 */
inline Eigen::MatrixXd get_motor_input_B(const Quad3DParams& params = Quad3DParams{}) {
    constexpr int nx = 12;
    constexpr int nu_motors = 4;
    
    // Wrench B matrix
    Eigen::MatrixXd B_wrench = Eigen::MatrixXd::Zero(nx, 4);
    B_wrench(8, 0)  = 1.0 / params.m;
    B_wrench(9, 1)  = 1.0 / params.Jx;
    B_wrench(10, 2) = 1.0 / params.Jy;
    B_wrench(11, 3) = 1.0 / params.Jz;
    
    // Motor to wrench mapping
    Eigen::Matrix4d M;
    M << 1.0,       1.0,       1.0,        1.0,
         0.0,      -params.l,  0.0,        params.l,
         params.l,  0.0,      -params.l,   0.0,
         params.c_tau, -params.c_tau, params.c_tau, -params.c_tau;
    
    return B_wrench * M;
}

} // namespace vimp
