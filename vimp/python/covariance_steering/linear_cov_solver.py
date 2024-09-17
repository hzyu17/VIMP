import numpy as np
from numba import njit
from numba import float64   
from scipy.integrate import odeint

def dsdPhi_odeint(z, t, *args):
    """
    Args:
        z (array): concatenated state x, co-state s, and flattened Phi [x, s, Phi]
        t (scalar): time variable
        args[0]: linearization (function): linearization function, the input is x, and the output is (A,B,a).
        args[1]: get_Qr (function): the function which takes state x, reference x_r as inputs, and returns quadratic matrices (Q, r).
        args[2]: n (scalar): state dimension
        
    Returns:
        [dx/dt, ds/dt, dPhi/dt]: odes
    """
    
    # dyn = args[0]
    linearization = args[0]
    get_Qr = args[1]
    n = args[2]
    
    x = z[:n]
    s = z[n:n+2*n]
    X = z[n+2*n:].reshape((2*n, 2*n))
    
    A, B, a = linearization(x)
    
    Q, r = get_Qr(x)
    
    top_row = np.concatenate((A, -B @ B.T), axis=1)
    bottom_row = np.concatenate((-Q, -A.T), axis=1)
    M = np.concatenate((top_row, bottom_row), axis=0)

    # state dynamics
    dx_dt = A @ x + a
    
    # co-state dynamics
    ds_dt = M @ s + np.concatenate((a, -r), axis=0)
    
    # covariance dynamics
    dX_dt = M @ X

    return np.concatenate([dx_dt, ds_dt, dX_dt.flatten()])

def dxdXl_odeint(z, t, *args):
    """
    Args:
        z (array): concatenated x (nx) and Xl (2*nx) [x, Xl]
        t (scalar): time variable
        # args[0]: dyn (function): system dynamics 
        args[0]: linearization (function): linearization function, the input is x, and the output is (A,B,a).
        args[1]: get_Qr (function): the function which takes state x, reference x_r as inputs, and returns quadratic matrices (Q, r).
        args[2]: n (scalar): state dimension
        
    Returns:
        [dx/dt, dPhi/dt]: odes
    """
    
    # dyn = args[0]
    linearization = args[0]
    get_Qr = args[1]
    n = args[2]
    
    x = z[:n]
    Xl = z[n:]
    
    A, B, a = linearization(x)
    
    Q, r = get_Qr(x)
    
    top_row = np.concatenate((A, -B @ B.T), axis=1)
    bottom_row = np.concatenate((-Q, -A.T), axis=1)
    M = np.concatenate((top_row, bottom_row), axis=0)

    # state dynamics
    # dx_dt = dyn(t, x)
    dx_dt = A @ x + a
    
    # state dynamics
    dXl_dt = M @ Xl + np.concatenate((a, -r), axis=0)
    
    return np.concatenate([dx_dt, dXl_dt])


def dx_dvXY(z, t, *args):
    """
    Args:
        z (array): concatenated x (nx) and flattened v_XY (2*nx, nx) [x, v_XY]
        t (scalar): time variable
        # args[0]: dyn (function): system dynamics 
        args[0]: linearization (function): linearization function, the input is x, and the output is (A,B,a).
        args[1]: get_Qr (function): the function which takes state x, reference x_r as inputs, and returns quadratic matrices (Q, r).
        args[2]: n (scalar): state dimension
        
    Returns:
        [dx/dt, dv_XY/dt]: odes
    """
    
    # dyn = args[0]
    linearization = args[0]
    get_Qr = args[1]
    n = args[2]
    
    x = z[:n]
    v_XY = z[n:].reshape((2*n, n))
    
    A, B, a = linearization(x)
    
    Q, _ = get_Qr(x)
    
    top_row = np.concatenate((A, -B @ B.T), axis=1)
    bottom_row = np.concatenate((-Q, -A.T), axis=1)
    M = np.concatenate((top_row, bottom_row), axis=0)

    # state dynamics
    # dx_dt = dyn(t, x)
    dx_dt = A @ x + a
    
    # state dynamics
    dvXY_dt = M @ v_XY
    
    return np.concatenate([dx_dt, dvXY_dt.flatten()])
    
        
# @njit((float64[:,:,:], float64[:,:,:], float64[:,:], 
#         float64, float64[:,:,:], float64[:,:],
#         float64[:], float64[:,:], float64[:], float64[:,:], float64))
def linear_covcontrol(linearization, get_Qr, epsilon, m0, Sig0, mT, SigT, tf, nt):
    _, B, _ = linearization(m0)
    nx, nu = B.shape
    # dt = tf / (nt - 1.0)
    I = np.eye(nx)
    
    t = np.linspace(0, tf, nt)  # Time points
    
    s0 = np.zeros(2 * nx, dtype=np.float64)
    Phi0 = np.eye(2 * nx, dtype=np.float64).flatten()
    
    initial_conditions_s_Phi = np.concatenate([m0, s0, Phi0.flatten()])  # Combine initial conditions

    # Solve the coupled system of ODEs using odeint
    # tuple: (linearization_functtion, state_dim)
    # args = (dynamics, linearization, get_Qr, nx)
    args = (linearization, get_Qr, nx)
    solution_s_Phi = odeint(dsdPhi_odeint, t=t, y0=initial_conditions_s_Phi, args=args)

    # Extract the solution
    s = solution_s_Phi[-1, nx:nx+2*nx]
    Phi = solution_s_Phi[-1, nx+2*nx:].reshape((2*nx, 2*nx))
    
    # M = np.ascontiguousarray(np.zeros((nt, 2*nx, 2*nx), dtype=np.float64))
    # B = np.ascontiguousarray(B)
    # Q = np.ascontiguousarray(Q)
    # A = np.ascontiguousarray(A)
    
    # for i in range(nt):
    #     top_row = np.concatenate((A[i], -B[i] @ B[i].T), axis=1)
    #     bottom_row = np.concatenate((-Q[i], -A[i].T), axis=1)
    #     M[i] = np.concatenate((top_row, bottom_row), axis=0)
    
    # Phi = np.ascontiguousarray(np.eye(2 * nx, dtype=np.float64))
    # for i in range(nt - 1):
    #     Phi_next = Phi + dt * (M[i] @ Phi)
    #     Phi = Phi + (M[i] @ Phi + M[i+1] @ Phi_next) * (dt/2.0)

    

    # s = np.ascontiguousarray(np.zeros(2 * nx, dtype=np.float64))
    # for i in range(nt - 1):
    #     s_next = s + dt * (M[i] @ s + np.concatenate((a[i], -r[i]), axis=0))
    #     s = s + (dt/2.0) * ((M[i] @ s + np.concatenate((a[i], -r[i]), axis=0)) + 
    #                         (M[i+1] @ s_next + np.concatenate((a[i + 1], -r[i + 1]), axis=0))) 

    # print("s")
    # print(s)
    # print("Phi")
    # print(Phi)
    
    Phi12 = Phi[:nx, nx:2*nx]
    Phi11 = Phi[:nx, :nx]
    
    lambda0 = np.linalg.solve(Phi12, mT - Phi11 @ m0 - s[0:nx])
    # Xl = np.ascontiguousarray(np.zeros((nt, 2 * nx), dtype=np.float64))
    # Xl[0, :nx] = m0
    # Xl[0, nx:2*nx] = lambda0
    
    # for i in range(nt - 1):
    #     Xl_next = Xl[i] + dt * (M[i] @ Xl[i] + np.concatenate((a[i], -r[i]), axis=0))
    #     Xl[i + 1] = Xl[i] + (dt/2.0) * ((M[i] @ Xl[i] + np.concatenate((a[i], -r[i]), axis=0)) + 
    #                                     (M[i + 1] @ Xl_next + np.concatenate((a[i+1], -r[i+1]), axis=0)))

    
    Xl0 = np.zeros(2*nx, dtype=np.float64)
    Xl0[0, :nx] = m0
    Xl0[0, nx:2*nx] = lambda0
    
    initial_conditions_x_Xl = np.concatenate([m0, Xl0])  # Combine initial conditions

    # Solve the coupled system of ODEs using odeint
    # tuple: (linearization_functtion, state_dim)
    # args = (dynamics, linearization, get_Qr, nx)
    solution_Xl = odeint(dxdXl_odeint, t=t, y0=initial_conditions_x_Xl, args=args)

    # Extract the solution for x and X
    Xl = solution_Xl[:, nx:]
    
    bx = Xl[:, :nx]
    lbd = Xl[:, nx:2*nx]

    v = np.zeros((nt, nu), dtype=np.float64)
    
    # B = np.ascontiguousarray(B)
    for i in range(nt):
        v[i] = -B[i].T @ lbd[i]
        
    inv_Phi12 = np.linalg.solve(Phi12, np.eye(nx))
    
    # Compute the sqrtm of Sig0 and invSig0
    invSig0 = np.linalg.solve(Sig0, np.eye(nx))
    eval_invSig0, evec_invSig0 = np.linalg.eigh(invSig0)
    sqrtInvSig0 = evec_invSig0 @ np.diag(np.sqrt(eval_invSig0)) @ evec_invSig0.T

    # Sig0
    eval_Sig0, evec_Sig0 = np.linalg.eigh(Sig0)
    sqrtSig0 = evec_Sig0 @ np.diag(np.sqrt(eval_Sig0)) @ evec_Sig0.T
    
    # Compute the sqrtm of (epsilon^2*(I/4) + sqrtSig0@inv_Phi12@SigT@inv_Phi12.T@sqrtSig0)
    tmp = epsilon**2 * I/4 + sqrtSig0 @ inv_Phi12 @ SigT @ inv_Phi12.T @ sqrtSig0
    tmp = (tmp + tmp.T) / 2
        
    eval_tmp, evec_tmp = np.linalg.eigh(tmp)
    sqrt_tmp = evec_tmp @ np.diag(np.sqrt(eval_tmp)) @ evec_tmp.T

    Pi0 = epsilon*invSig0/2 - inv_Phi12 @ Phi11 - sqrtInvSig0 @ sqrt_tmp @ sqrtInvSig0
    Pi0 = (Pi0 + Pi0.T) / 2
    
    # Pi = np.zeros((nt, nx, nx), dtype=np.float64)
    # Pi[0] = (Pi0 + Pi0.T) / 2
    # v_XY = np.zeros((nt, 2*nx, nx), dtype=np.float64)
    # v_XY[0, :nx, :nx] = np.eye(nx)
    # v_XY[0, nx:, :nx] = Pi[0]
    
    # for i in range(nt - 1):
    #     dXY = M[i] @ v_XY[i]
    #     next_XY = v_XY[i] + dXY*dt
    #     dXY_next = M[i+1]@next_XY
        
    #     v_XY[i+1] = v_XY[i] + (dXY + dXY_next)*(dt/2.0) 
    #     X_next = v_XY[i+1,:nx,:nx]
    #     Y_next = v_XY[i+1,nx:,:nx]
    #     inv_X_next = np.linalg.solve(X_next, np.eye(nx))
    #     Pi[i+1] = Y_next @ inv_X_next
    
        # parPi_part = A[i].T @ Pi[i] + Pi[i] @ A[i] - Pi[i] @ B[i] @ B[i].T @ Pi[i] + Q[i]
        # Pi_next = Pi[i] - dt * parPi_part
        
        # parPi_part_next = A[i+1].T @ Pi_next + Pi_next @ A[i+1] - \
        #                   Pi_next @ B[i+1] @ B[i+1].T @ Pi_next + Q[i+1]
        
        # Pi[i+1] = Pi[i] - dt / 2 * (parPi_part + parPi_part_next)
    
    v_XY0 = np.zeros((2*nx, nx), dtype=np.float64)
    v_XY0[:nx, :nx] = np.eye(nx)
    v_XY0[nx:, :nx] = Pi0
    
    initial_conditions_Pi = np.concatenate([m0, v_XY0.flatten()])  # Combine initial conditions

    # Solve the coupled system of ODEs using odeint
    # tuple: (linearization_functtion, state_dim)
    # args = (dynamics, linearization, get_Qr, nx)
    solution_Pi = odeint(dsdPhi_odeint, t=t, y0=initial_conditions_Pi, args=args)

    # Extract the solution for x and X
    Pi = solution_Pi[:, nx:].reshape((-1, 2*nx, nx))
    
    K = np.ascontiguousarray(np.zeros((nt, nu, nx), dtype=np.float64))
    d = np.ascontiguousarray(np.zeros((nt, nu), dtype=np.float64))
    
    Pi = np.ascontiguousarray(Pi)
    
    for i in range(nt):
        K[i] = -B[i].T @ Pi[i]
        d[i] = v[i] + B[i].T @ Pi[i] @ bx[i]
        
    return K, d, Pi, lbd

