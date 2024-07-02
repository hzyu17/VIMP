import numpy as np
from numba import njit
from numba import float64    
        
        
def linear_covcontrol(A, B, a, epsilon, Q, r, m0, Sig0, mT, SigT, tf):
    nt, nx, nu = B.shape
    dt = tf / (nt - 1.0)
    
    M = np.zeros((nt, 2*nx, 2*nx), dtype=np.float64)
    
    for i in range(nt):
        top_row = np.concatenate((A[i], -B[i] @ B[i].T), axis=1)
        bottom_row = np.concatenate((-Q[i], -A[i].T), axis=1)
        M[i] = np.concatenate((top_row, bottom_row), axis=0)
    
    # ======================
    # Solve \Phi ODE: Euler
    # ======================
    # Phi_euler = np.eye(2 * nx, dtype=np.float64)
    # for i in range(nt - 1):
    #     Phi_next = Phi_euler + dt * (M[i] @ Phi_euler)
    #     Phi_euler = Phi_euler + (M[i] @ Phi_euler + M[i+1] @ Phi_next) * (dt/2.0)
        
    # =================
    # Solve \Phi ODE
    # =================
    from scipy.integrate import solve_ivp
    
    def compute_M(t_idx):
        i = min(int(t_idx / dt), nt - 1)  # Convert continuous time to nearest time index, cap at nt-1
        top_row = np.concatenate((A[i], -B[i] @ B[i].T), axis=1)
        bottom_row = np.concatenate((-Q[i], -A[i].T), axis=1)
        return np.concatenate((top_row, bottom_row), axis=0)
    
    def ode_system_Phi(t, y):
        y_reshaped = y.reshape((2*nx, 2*nx))
        dydt = compute_M(t) @ y_reshaped
        return dydt.flatten()  # Flatten because solve_ivp expects a flat array

    # Initial condition
    Phi0 = np.eye(2 * nx).flatten()  # Flatten the initial condition into a 1D array

    # Time span
    t_span = (0, dt * (nt - 1))

    # Call the solver
    result = solve_ivp(ode_system_Phi, t_span, Phi0, method='RK23', t_eval=np.linspace(0, dt * (nt - 1), nt))

    # Reshape the result to get the solution matrices at each time step
    Phi = result.y.reshape((2*nx, 2*nx, nt))[:,:,-1]
    
    Phi12 = Phi[:nx, nx:2*nx]
    Phi11 = Phi[:nx, :nx]

    # ====================
    # End Solve \Phi ODE
    # ====================
    
    # =================
    # Solve s ODE
    # =================
    def ode_system_s(t, s):
        t_idx = min(int(t / dt), nt - 1)  # Convert continuous time to nearest time index
        M_t = M[t_idx]
        a_t = a[t_idx]
        r_t = r[t_idx]
        f_t = np.concatenate((a_t, -r_t), axis=0)
        dsdt = M_t @ s + f_t
        return dsdt
    
    # Initial condition
    s0 = np.zeros(2 * nx, dtype=np.float64)
    t_span = (0, dt * (nt - 1))

    result = solve_ivp(ode_system_s, t_span, s0, method='RK23', t_eval=np.linspace(0, dt * (nt - 1), nt))

    s_solution = result.y
    s = s_solution[:, -1]
    
    # =================
    # End Solve s: ODE
    # =================

    # s_euler = np.zeros(2 * nx, dtype=np.float64)
    # for i in range(nt - 1):
    #     s_next = s_euler + dt * (M[i] @ s_euler + np.concatenate((a[i], -r[i]), axis=0))
    #     s_euler = s_euler + (dt/2.0) * ((M[i] @ s_euler + np.concatenate((a[i], -r[i]), axis=0)) + 
    #                                     (M[i+1] @ s_next + np.concatenate((a[i + 1], -r[i + 1]), axis=0))) 

    # print("diff s & s_euler", np.linalg.norm(s - s_euler))
    
    lambda0 = np.linalg.solve(Phi12, mT - Phi11 @ m0 - s[0:nx])
    
    # =========
    # Solve Xl
    # =========
    def ode_system_Xl(t, Xl):
        t_idx = min(int(t / dt), nt - 1)  # Convert continuous time to the nearest time index
        M_t = M[t_idx]
        a_t = a[t_idx]
        r_t = r[t_idx]
        f_t = np.concatenate((a_t, -r_t), axis=0)
        dXldt = M_t @ Xl + f_t
        return dXldt

    # Initial condition
    Xl0 = np.zeros(2 * nx)
    Xl0[:nx] = m0
    Xl0[nx:2*nx] = lambda0

    t_span = (0, dt * (nt - 1))
    result = solve_ivp(ode_system_Xl, t_span, Xl0, method='RK23', t_eval=np.linspace(0, dt * (nt - 1), nt))

    # Solution
    Xl = result.y.T
    
    # ============
    # End Solve Xl
    # ============
    
    # Xl = np.zeros((nt, 2 * nx), dtype=np.float64)
    # Xl[0, :nx] = m0
    # Xl[0, nx:2*nx] = lambda0

    # for i in range(nt - 1):
    #     Xl_next = Xl[i] + dt * (M[i] @ Xl[i] + np.concatenate((a[i], -r[i]), axis=0))
    #     Xl[i + 1] = Xl[i] + (dt/2.0) * ((M[i] @ Xl[i] + np.concatenate((a[i], -r[i]), axis=0)) + 
    #                                     (M[i + 1] @ Xl_next + np.concatenate((a[i+1], -r[i+1]), axis=0)))

    # print("diff Xl & s_euler", np.linalg.norm(Xl - Xl_solver))
    
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
    tmp = epsilon**2 * np.eye(nx)/4 + sqrtSig0 @ inv_Phi12 @ SigT @ inv_Phi12.T @ sqrtSig0
    tmp = (tmp + tmp.T) / 2
    
    eval_tmp, evec_tmp = np.linalg.eigh(tmp)
    sqrt_tmp = evec_tmp @ np.diag(np.sqrt(eval_tmp)) @ evec_tmp.T
    
    # Initial conditoin for Pi for linear CS 
    Pi0 = epsilon*invSig0/2 - inv_Phi12 @ Phi11 - sqrtInvSig0 @ sqrt_tmp @ sqrtInvSig0
    
    # ==============
    # Solve for XY
    # ==============
    def ode_system_XY(t, y):
        y_reshaped = y.reshape((2*nx, nx))
        dydt = compute_M(t) @ y_reshaped
        return dydt.flatten()  # Flatten because solve_ivp expects a flat array

    # Time span for the integration
    t_span = (0, dt * (nt - 1))

    # Solve the ODE
    v_XY0 = np.zeros((2 * nx, nx))
    v_XY0[:nx, :nx] = np.eye(nx)
    v_XY0[nx:, :nx] = (Pi0 + Pi0.T) / 2

    # Flatten initial conditions for use with solve_ivp
    v_XY0_flat = v_XY0.flatten()
    result = solve_ivp(ode_system_XY, t_span, v_XY0_flat, method='RK23', t_eval=np.linspace(0, dt * (nt - 1), nt))

    # Reshape the result to retrieve v_XY at each time step
    v_XY_solution = result.y.reshape((2 * nx, nx, -1))

    Pi = np.zeros((nt, nx, nx), dtype=np.float64)
    Pi[0] = (Pi0 + Pi0.T) / 2
    for i in range(1, nt):
        X_i = v_XY_solution[:nx,:nx,i]
        Y_i = v_XY_solution[nx:,:nx,i]
        inv_Xi = np.linalg.solve(X_i, np.eye(nx))
        Pi[i] = Y_i @ inv_Xi
        
    # ===================
    # End Solve for Pi
    # ===================
    
    # Pi_euler = np.zeros((nt, nx, nx), dtype=np.float64)
    # Pi_euler[0] = (Pi0 + Pi0.T) / 2
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
    #     Pi_euler[i+1] = Y_next @ inv_X_next
    
    # print("diff Pi solver & euler", np.linalg.norm(Pi - Pi_euler))
    
    K = np.zeros((nt, nu, nx), dtype=np.float64)
    d = np.zeros((nt, nu), dtype=np.float64)
        
    for i in range(nt):
        K[i] = -B[i].T @ Pi[i]
        d[i] = v[i] + B[i].T @ Pi[i] @ bx[i]
        
    return K, d, Pi, lbd

