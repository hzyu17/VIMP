"""
Linear Covariance Steering with Time Normalization.

Solves the covariance steering problem with hard terminal constraints.
Time normalization ensures numerical stability for any tf.

Control magnitude scales as O(1/tf) - this is physics, not a bug.
"""

import numpy as np
from scipy.integrate import solve_ivp
import warnings


def symmetric(A):
    """Enforce symmetry of a matrix."""
    return 0.5 * (A + A.T)


def safe_matrix_sqrt(A, min_eval=1e-14):
    """Compute matrix square root with eigenvalue clamping."""
    A = symmetric(A)
    evals, evecs = np.linalg.eigh(A)
    evals_clamped = np.maximum(evals, min_eval)
    return evecs @ np.diag(np.sqrt(evals_clamped)) @ evecs.T


def safe_matrix_inv_sqrt(A, min_eval=1e-14):
    """Compute inverse square root with eigenvalue clamping."""
    A = symmetric(A)
    evals, evecs = np.linalg.eigh(A)
    evals_clamped = np.maximum(evals, min_eval)
    return evecs @ np.diag(1.0 / np.sqrt(evals_clamped)) @ evecs.T


def stable_solve(A, B, reg=1e-12):
    """Solve A @ X = B with regularization for ill-conditioned A."""
    try:
        return np.linalg.solve(A, B)
    except np.linalg.LinAlgError:
        A_reg = A + reg * np.eye(A.shape[0])
        return np.linalg.solve(A_reg, B)


def stable_inverse(A, reg=1e-12):
    """SVD-based matrix inverse with regularization."""
    U, s, Vh = np.linalg.svd(A, full_matrices=False)
    s_inv = np.where(s > reg, 1.0 / s, 0.0)
    return Vh.T @ np.diag(s_inv) @ U.T


def linear_covcontrol(A, B, a, epsilon, Q, r, m0, Sig0, mT, SigT, tf,
                      ode_method='DOP853', ode_rtol=1e-8, ode_atol=1e-10,
                      reg=1e-12, verbose=False):
    """
    Linear covariance steering with time normalization.
    
    Internally normalizes to τ ∈ [0,1] for numerical stability,
    then returns controls in physical units.
    
    Parameters
    ----------
    A : ndarray (nt, nx, nx) - State dynamics matrices
    B : ndarray (nt, nx, nu) - Control input matrices
    a : ndarray (nt, nx) - Drift terms
    epsilon : float - Covariance steering parameter
    Q : ndarray (nt, nx, nx) - State cost matrices
    r : ndarray (nt, nx) - Reference state costs
    m0 : ndarray (nx,) - Initial mean
    Sig0 : ndarray (nx, nx) - Initial covariance
    mT : ndarray (nx,) - Terminal mean (hard constraint)
    SigT : ndarray (nx, nx) - Terminal covariance (hard constraint)
    tf : float - Final time
    ode_method : str - ODE solver ('RK45', 'RK23', 'DOP853')
    ode_rtol, ode_atol : float - ODE solver tolerances
    reg : float - Regularization for matrix operations
    verbose : bool - Print diagnostics
    
    Returns
    -------
    K : ndarray (nt, nu, nx) - Feedback gains
    d : ndarray (nt, nu) - Feedforward controls
    Pi : ndarray (nt, nx, nx) - Value function matrices
    lbd : ndarray (nt, nx) - Costate trajectory
    """
    nt, nx, nu = B.shape
    
    # ===========================================
    # Time normalization: τ = t/tf, τ ∈ [0, 1]
    # dx/dτ = tf * (Ax + Bu + a)
    # ===========================================
    A_n = A * tf
    B_n = B * tf
    a_n = a * tf
    Q_n = Q * tf
    r_n = r * tf
    
    dt = 1.0 / (nt - 1)
    
    Sig0 = symmetric(Sig0)
    SigT = symmetric(SigT)
    
    if verbose:
        print(f"tf = {tf:.4f}, normalized dt = {dt:.6f}")
    
    # Build Hamiltonian M = [A, -BB'; -Q, -A']
    M = np.zeros((nt, 2*nx, 2*nx))
    for i in range(nt):
        BBT = symmetric(B_n[i] @ B_n[i].T)
        M[i, :nx, :nx] = A_n[i]
        M[i, :nx, nx:] = -BBT
        M[i, nx:, :nx] = -symmetric(Q_n[i])
        M[i, nx:, nx:] = -A_n[i].T
    
    # Interpolators
    def M_interp(tau):
        tau = np.clip(tau, 0, 1)
        idx = tau / dt
        i_low = min(int(np.floor(idx)), nt - 1)
        i_high = min(i_low + 1, nt - 1)
        if i_low == i_high:
            return M[i_low]
        alpha = idx - i_low
        return (1 - alpha) * M[i_low] + alpha * M[i_high]
    
    def f_interp(tau):
        tau = np.clip(tau, 0, 1)
        idx = tau / dt
        i_low = min(int(np.floor(idx)), nt - 1)
        i_high = min(i_low + 1, nt - 1)
        if i_low == i_high:
            return np.concatenate([a_n[i_low], -r_n[i_low]])
        alpha = idx - i_low
        return np.concatenate([
            (1 - alpha) * a_n[i_low] + alpha * a_n[i_high],
            -((1 - alpha) * r_n[i_low] + alpha * r_n[i_high])
        ])
    
    t_span = (0, 1)
    t_eval = np.linspace(0, 1, nt)
    
    # Solve Φ ODE
    def ode_Phi(tau, y):
        return (M_interp(tau) @ y.reshape(2*nx, 2*nx)).flatten()
    
    result_Phi = solve_ivp(ode_Phi, t_span, np.eye(2*nx).flatten(),
                           method=ode_method, t_eval=t_eval,
                           rtol=ode_rtol, atol=ode_atol)
    
    Phi = result_Phi.y[:, -1].reshape(2*nx, 2*nx)
    Phi11, Phi12 = Phi[:nx, :nx], Phi[:nx, nx:]
    
    Phi12_cond = np.linalg.cond(Phi12)
    if verbose:
        print(f"Phi12 condition: {Phi12_cond:.2e}")
    
    # Solve s ODE
    def ode_s(tau, s):
        return M_interp(tau) @ s + f_interp(tau)
    
    result_s = solve_ivp(ode_s, t_span, np.zeros(2*nx),
                         method=ode_method, t_eval=t_eval,
                         rtol=ode_rtol, atol=ode_atol)
    s_final = result_s.y[:, -1]
    
    # Boundary condition for λ(0)
    lambda0 = stable_solve(Phi12, mT - Phi11 @ m0 - s_final[:nx], reg=reg)
    
    # Solve mean/costate trajectory
    def ode_ml(tau, ml):
        return M_interp(tau) @ ml + f_interp(tau)
    
    result_ml = solve_ivp(ode_ml, t_span, np.concatenate([m0, lambda0]),
                          method=ode_method, t_eval=t_eval,
                          rtol=ode_rtol, atol=ode_atol)
    
    ml = result_ml.y.T
    bx = ml[:, :nx]
    lbd = ml[:, nx:]
    
    # Feedforward: v = -B'λ
    v = np.zeros((nt, nu))
    for i in range(nt):
        v[i] = -B_n[i].T @ lbd[i]
    
    # Covariance steering: Π(0)
    inv_Phi12 = stable_inverse(Phi12, reg=reg)
    sqrtSig0 = safe_matrix_sqrt(Sig0)
    sqrtInvSig0 = safe_matrix_inv_sqrt(Sig0)
    
    inner = symmetric(inv_Phi12 @ SigT @ inv_Phi12.T)
    tmp = symmetric((epsilon**2 / 4) * np.eye(nx) + sqrtSig0 @ inner @ sqrtSig0)
    sqrt_tmp = safe_matrix_sqrt(tmp)
    
    Pi0 = symmetric(
        (epsilon / 2) * stable_inverse(Sig0, reg=reg)
        - inv_Phi12 @ Phi11
        - sqrtInvSig0 @ sqrt_tmp @ sqrtInvSig0
    )
    
    # Solve [X; Y] for Π = YX⁻¹
    def ode_XY(tau, y):
        return (M_interp(tau) @ y.reshape(2*nx, nx)).flatten()
    
    XY0 = np.vstack([np.eye(nx), Pi0])
    result_XY = solve_ivp(ode_XY, t_span, XY0.flatten(),
                          method=ode_method, t_eval=t_eval,
                          rtol=ode_rtol, atol=ode_atol)
    
    XY = result_XY.y.reshape(2*nx, nx, nt)
    
    Pi = np.zeros((nt, nx, nx))
    Pi[0] = Pi0
    for i in range(1, nt):
        X_i, Y_i = XY[:nx, :, i], XY[nx:, :, i]
        try:
            Pi[i] = symmetric(stable_solve(X_i.T, Y_i.T, reg=reg).T)
        except:
            Pi[i] = symmetric(Y_i @ stable_inverse(X_i, reg=reg))
    
    # Feedback gains: K = -B'Π, d = v + B'Π·m
    K = np.zeros((nt, nu, nx))
    d = np.zeros((nt, nu))
    for i in range(nt):
        K[i] = -B_n[i].T @ Pi[i]
        d[i] = v[i] + B_n[i].T @ Pi[i] @ bx[i]
    
    if verbose:
        # Compute control trajectory and costs for diagnostics
        u_traj = np.array([K[i] @ bx[i] + d[i] for i in range(nt)])
        u_norms = np.linalg.norm(u_traj, axis=1)
        cost_normalized = np.trapz(np.sum(u_traj**2, axis=1), t_eval)
        cost_physical = tf * cost_normalized
        print(f"Max |u|: {np.max(u_norms):.4e}")
        print(f"Cost (physical): {cost_physical:.4e}")
        print(f"Cost (normalized): {cost_normalized:.4e}")
    
    return K, d, Pi, lbd