"""
3D Quadrotor Dynamics and Linearization

State: [x, y, z, φ, θ, ψ, vx, vy, vz, p, q, r] (12D)
  - Position: (x, y, z) in world frame
  - Attitude: (φ, θ, ψ) roll, pitch, yaw (ZYX Euler)
  - Velocity: (vx, vy, vz) in body frame
  - Angular rate: (p, q, r) in body frame

Control: [T, τx, τy, τz] (4D)
  - T: total thrust (along body z-axis)
  - τx, τy, τz: body torques
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple


@dataclass
class Quad3DParams:
    """Physical parameters for 3D quadrotor."""
    mass: float = 1.0       # kg
    Jx: float = 0.01        # kg·m² (roll inertia)
    Jy: float = 0.01        # kg·m² (pitch inertia)
    Jz: float = 0.02        # kg·m² (yaw inertia)
    arm_length: float = 0.25  # m
    g: float = 9.81         # m/s²
    
    # Collision geometry
    body_radius: float = 0.1
    rotor_radius: float = 0.05
    use_rotor_spheres: bool = True
    
    @property
    def J(self) -> np.ndarray:
        return np.diag([self.Jx, self.Jy, self.Jz])
    
    @property
    def J_inv(self) -> np.ndarray:
        return np.diag([1.0/self.Jx, 1.0/self.Jy, 1.0/self.Jz])


def rotation_matrix_zyx(phi: float, theta: float, psi: float) -> np.ndarray:
    """
    Rotation matrix from body to world frame using ZYX Euler angles.
    
    Args:
        phi: Roll angle (rotation about x)
        theta: Pitch angle (rotation about y)
        psi: Yaw angle (rotation about z)
    
    Returns:
        3x3 rotation matrix R such that v_world = R @ v_body
    """
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cy, sy = np.cos(psi), np.sin(psi)
    
    R = np.array([
        [cy*ct, cy*st*sp - sy*cp, cy*st*cp + sy*sp],
        [sy*ct, sy*st*sp + cy*cp, sy*st*cp - cy*sp],
        [-st,   ct*sp,            ct*cp]
    ])
    return R


def rotation_matrix_derivatives(phi: float, theta: float, psi: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Partial derivatives of rotation matrix w.r.t. Euler angles.
    
    Returns:
        (dR/dphi, dR/dtheta, dR/dpsi)
    """
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cy, sy = np.cos(psi), np.sin(psi)
    
    # dR/dphi
    dR_dphi = np.array([
        [0, cy*st*cp + sy*sp, -cy*st*sp + sy*cp],
        [0, sy*st*cp - cy*sp, -sy*st*sp - cy*cp],
        [0, ct*cp,            -ct*sp]
    ])
    
    # dR/dtheta
    dR_dtheta = np.array([
        [-cy*st, cy*ct*sp, cy*ct*cp],
        [-sy*st, sy*ct*sp, sy*ct*cp],
        [-ct,    -st*sp,   -st*cp]
    ])
    
    # dR/dpsi
    dR_dpsi = np.array([
        [-sy*ct, -sy*st*sp - cy*cp, -sy*st*cp + cy*sp],
        [cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp],
        [0,      0,                  0]
    ])
    
    return dR_dphi, dR_dtheta, dR_dpsi


def euler_rate_matrix(phi: float, theta: float) -> np.ndarray:
    """
    Matrix T such that euler_dot = T @ omega_body.
    
    Maps body angular rates [p, q, r] to Euler angle rates [phi_dot, theta_dot, psi_dot].
    """
    cp, sp = np.cos(phi), np.sin(phi)
    ct = np.cos(theta)
    tt = np.tan(theta)
    
    # Avoid singularity at theta = ±90°
    if abs(ct) < 1e-6:
        ct = np.sign(ct) * 1e-6 if ct != 0 else 1e-6
        tt = np.sin(theta) / ct
    
    T = np.array([
        [1, sp*tt, cp*tt],
        [0, cp,    -sp],
        [0, sp/ct, cp/ct]
    ])
    return T


def quad3d_dynamics(z: np.ndarray, u: np.ndarray, params: Quad3DParams) -> np.ndarray:
    """
    Continuous-time nonlinear dynamics: dz/dt = f(z, u)
    
    Args:
        z: State vector [12]
        u: Control vector [4] = [T, τx, τy, τz]
        params: Quadrotor parameters
    
    Returns:
        dz: State derivative [12]
    """
    # Unpack state
    phi, theta, psi = z[3], z[4], z[5]
    vx, vy, vz = z[6], z[7], z[8]
    p, q, r = z[9], z[10], z[11]
    
    # Unpack control
    T, tau_x, tau_y, tau_z = u[0], u[1], u[2], u[3]
    
    # Rotation matrix
    R = rotation_matrix_zyx(phi, theta, psi)
    
    dz = np.zeros(12)
    
    # Position derivative: p_dot = R @ v_body
    v_body = np.array([vx, vy, vz])
    dz[0:3] = R @ v_body
    
    # Euler angle derivative: euler_dot = T_euler @ omega
    T_euler = euler_rate_matrix(phi, theta)
    omega = np.array([p, q, r])
    dz[3:6] = T_euler @ omega
    
    # Body velocity derivative
    # a_body = -omega × v + R^T @ g_world + [0, 0, T/m]
    g_world = np.array([0, 0, -params.g])
    g_body = R.T @ g_world
    omega_cross_v = np.cross(omega, v_body)
    
    dz[6] = -omega_cross_v[0] + g_body[0]
    dz[7] = -omega_cross_v[1] + g_body[1]
    dz[8] = -omega_cross_v[2] + g_body[2] + T / params.mass
    
    # Angular rate derivative: J @ omega_dot = -omega × (J @ omega) + tau
    J_omega = params.J @ omega
    omega_cross_J_omega = np.cross(omega, J_omega)
    tau = np.array([tau_x, tau_y, tau_z])
    
    dz[9:12] = params.J_inv @ (-omega_cross_J_omega + tau)
    
    return dz


def quad3d_A_matrix(z: np.ndarray, params: Quad3DParams) -> np.ndarray:
    """
    Jacobian of dynamics w.r.t. state: A = ∂f/∂z
    
    Args:
        z: State vector [12]
        params: Quadrotor parameters
    
    Returns:
        A: Jacobian matrix [12 x 12]
    """
    phi, theta, psi = z[3], z[4], z[5]
    vx, vy, vz = z[6], z[7], z[8]
    p, q, r = z[9], z[10], z[11]
    
    A = np.zeros((12, 12))
    
    # Rotation matrix and derivatives
    R = rotation_matrix_zyx(phi, theta, psi)
    dR_dphi, dR_dtheta, dR_dpsi = rotation_matrix_derivatives(phi, theta, psi)
    v_body = np.array([vx, vy, vz])
    
    # ∂(p_dot)/∂(euler): dR/d(euler) @ v_body
    A[0:3, 3] = dR_dphi @ v_body
    A[0:3, 4] = dR_dtheta @ v_body
    A[0:3, 5] = dR_dpsi @ v_body
    
    # ∂(p_dot)/∂(v_body): R
    A[0:3, 6:9] = R
    
    # ∂(euler_dot)/∂(euler): derivative of T_euler
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    
    if abs(ct) < 1e-6:
        ct = np.sign(ct) * 1e-6 if ct != 0 else 1e-6
    
    tt = st / ct
    sec_t = 1.0 / ct
    sec_t_sq = sec_t * sec_t
    
    # ∂(euler_dot)/∂phi
    dT_dphi = np.array([
        [0, cp*tt, -sp*tt],
        [0, -sp,   -cp],
        [0, cp*sec_t, -sp*sec_t]
    ])
    A[3:6, 3] = dT_dphi @ np.array([p, q, r])
    
    # ∂(euler_dot)/∂theta
    dT_dtheta = np.array([
        [0, sp*sec_t_sq, cp*sec_t_sq],
        [0, 0, 0],
        [0, sp*tt*sec_t, cp*tt*sec_t]
    ])
    A[3:6, 4] = dT_dtheta @ np.array([p, q, r])
    
    # ∂(euler_dot)/∂(omega): T_euler
    T_euler = euler_rate_matrix(phi, theta)
    A[3:6, 9:12] = T_euler
    
    # ∂(v_body_dot)/∂(euler): ∂(R^T @ g)/∂(euler)
    g_world = np.array([0, 0, -params.g])
    A[6:9, 3] = dR_dphi.T @ g_world
    A[6:9, 4] = dR_dtheta.T @ g_world
    A[6:9, 5] = dR_dpsi.T @ g_world
    
    # ∂(v_body_dot)/∂(v_body): -[omega]_×
    omega_skew = np.array([
        [0, -r, q],
        [r, 0, -p],
        [-q, p, 0]
    ])
    A[6:9, 6:9] = -omega_skew
    
    # ∂(v_body_dot)/∂(omega): [v_body]_×
    v_skew = np.array([
        [0, -vz, vy],
        [vz, 0, -vx],
        [-vy, vx, 0]
    ])
    A[6:9, 9:12] = v_skew
    
    # ∂(omega_dot)/∂(omega)
    Jx, Jy, Jz = params.Jx, params.Jy, params.Jz
    omega = np.array([p, q, r])
    J_omega = params.J @ omega
    
    J_omega_skew = np.array([
        [0, -J_omega[2], J_omega[1]],
        [J_omega[2], 0, -J_omega[0]],
        [-J_omega[1], J_omega[0], 0]
    ])
    
    omega_cross_J = np.array([
        [0, -r*Jz, q*Jy],
        [r*Jz, 0, -p*Jx],
        [-q*Jy, p*Jx, 0]
    ])
    
    A[9:12, 9:12] = params.J_inv @ (-J_omega_skew - omega_cross_J)
    
    return A


def quad3d_B_matrix(z: np.ndarray, params: Quad3DParams) -> np.ndarray:
    """
    Jacobian of dynamics w.r.t. control: B = ∂f/∂u
    
    Args:
        z: State vector [12]
        params: Quadrotor parameters
    
    Returns:
        B: Jacobian matrix [12 x 4]
    """
    B = np.zeros((12, 4))
    
    # Thrust affects vz_body_dot
    B[8, 0] = 1.0 / params.mass
    
    # Torques affect omega_dot
    B[9, 1] = 1.0 / params.Jx
    B[10, 2] = 1.0 / params.Jy
    B[11, 3] = 1.0 / params.Jz
    
    return B


def linearize_quad3d(z: np.ndarray, u: np.ndarray, dt: float, 
                     params: Quad3DParams) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Linearize and discretize quadrotor dynamics at (z, u).
    
    Returns discrete-time matrices (A_d, B_d, a_d) such that:
        z_{k+1} ≈ A_d @ z_k + B_d @ u_k + a_d
    
    Args:
        z: State vector [12]
        u: Control vector [4]
        dt: Time step
        params: Quadrotor parameters
    
    Returns:
        A_d: Discrete state matrix [12 x 12]
        B_d: Discrete input matrix [12 x 4]
        a_d: Discrete affine term [12]
    """
    # Continuous-time linearization
    A_c = quad3d_A_matrix(z, params)
    B_c = quad3d_B_matrix(z, params)
    f_c = quad3d_dynamics(z, u, params)
    
    # Euler discretization
    A_d = np.eye(12) + dt * A_c
    B_d = dt * B_c
    
    # Affine term: captures linearization error
    a_d = dt * (f_c - A_c @ z - B_c @ u)
    
    return A_d, B_d, a_d


def linearize_trajectory(zk: np.ndarray, Sk: np.ndarray, 
                         As: np.ndarray, params: Quad3DParams, 
                         dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Linearize dynamics along entire trajectory.
    
    Args:
        zk: Mean trajectory [nt, nx]
        Sk: Covariance trajectory [nt, nx, nx]
        As: Current closed-loop dynamics [nt, nx, nx]
        params: Quadrotor parameters
        dt: Time step
    
    Returns:
        hAk: Linearized A matrices [nt, nx, nx]
        hBk: Linearized B matrices [nt, nx, nu]
        hak: Linearized affine terms [nt, nx]
        nTr: Trace terms for cost [nt, nx]
    """
    nt, nx = zk.shape
    nu = 4
    
    hAk = np.zeros((nt, nx, nx))
    hBk = np.zeros((nt, nx, nu))
    hak = np.zeros((nt, nx))
    nTr = np.zeros((nt, nx))
    
    for i in range(nt):
        # Nominal control (hover thrust)
        u_nom = np.array([params.mass * params.g, 0, 0, 0])
        
        # Linearize at this state
        A_d, B_d, a_d = linearize_quad3d(zk[i], u_nom, dt, params)
        
        hAk[i] = A_d
        hBk[i] = B_d
        hak[i] = a_d
        
        # Trace term for stochastic cost
        nTr[i] = np.zeros(nx)
    
    return hAk, hBk, hak, nTr


def simulate_quad3d(z0: np.ndarray, u_seq: np.ndarray, dt: float, 
                    params: Quad3DParams) -> np.ndarray:
    """
    Simulate quadrotor trajectory given control sequence.
    
    Args:
        z0: Initial state [12]
        u_seq: Control sequence [nt, 4]
        dt: Time step
        params: Quadrotor parameters
    
    Returns:
        z_traj: State trajectory [nt+1, 12]
    """
    nt = u_seq.shape[0]
    z_traj = np.zeros((nt + 1, 12))
    z_traj[0] = z0
    
    for i in range(nt):
        # RK4 integration
        z = z_traj[i]
        u = u_seq[i]
        
        k1 = quad3d_dynamics(z, u, params)
        k2 = quad3d_dynamics(z + 0.5*dt*k1, u, params)
        k3 = quad3d_dynamics(z + 0.5*dt*k2, u, params)
        k4 = quad3d_dynamics(z + dt*k3, u, params)
        
        z_traj[i+1] = z + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
    
    return z_traj


def hover_equilibrium(position: np.ndarray, yaw: float, 
                      params: Quad3DParams) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute hover equilibrium state and control.
    
    Args:
        position: Desired [x, y, z]
        yaw: Desired yaw angle
        params: Quadrotor parameters
    
    Returns:
        z_eq: Equilibrium state [12]
        u_eq: Equilibrium control [4]
    """
    z_eq = np.zeros(12)
    z_eq[0:3] = position
    z_eq[5] = yaw  # yaw angle
    
    u_eq = np.array([params.mass * params.g, 0, 0, 0])
    
    return z_eq, u_eq