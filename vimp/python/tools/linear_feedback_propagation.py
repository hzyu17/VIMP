import numpy as np
from numba import njit, prange

@njit(parallel=True)
def optimal_deterministic_trj(A, B, a, K, d, x0, tf):
    nt, nu, nx = K.shape
    dt = tf / (nt -1)    
    xs = np.zeros((nt, nx))
    xs[0] = x0
    ustar = np.zeros((nt, nu))

    xt = x0
    # if len(A.shape) == 3:  # Time varying
    for i in prange(nt - 1):
        ut = K[i] @ xt + d[i]
        ustar[i] = ut
        
        # ---------- mean ----------
        x_next = xt + (A[i] @ xt + B[i] @ ut + a[i]) * dt
        ut_next = K[i+1] @ x_next + d[i+1]

        xs[i+1] = xt + (A[i] @ xt + B[i] @ ut + a[i] +
                            A[i+1] @ x_next + B[i+1] @ ut_next + a[i+1]) * dt / 2
                            
        xt = xs[i+1]

    return xs, ustar

# @njit(parallel=True)
def propagate_Sig(A, B, K, epsilon, Sig0, tf):
    nt, nu, nx = K.shape
    
    trj_cov = np.zeros((nt, nx, nx))
    trj_cov[0] = Sig0
    Sigt = Sig0
    dt = tf / (nt - 1)

    # if len(A.shape) == 3:  # Time varying
    for i in range(nt - 1):
        # ---------- covariance ----------
        A_cl = A[i] + np.dot(B[i], K[i])
        diff_cov_i = np.dot(A_cl, Sigt) + np.dot(Sigt, A_cl.T) + epsilon * np.dot(B[i], B[i].T)
        Sigt_new = Sigt + dt * diff_cov_i

        A_cl_new = A[i+1] + np.dot(B[i+1], K[i+1])
        diff_cov_new = np.dot(A_cl_new, Sigt_new) + np.dot(Sigt_new, A_cl_new.T) + epsilon * np.dot(B[i+1], B[i+1].T)
        trj_cov[i+1] = Sigt + (dt / 2) * (diff_cov_i + diff_cov_new)

        Sigt = trj_cov[i+1]

    return trj_cov

# @njit(parallel=True)
def mean_cov(A, B, a, K, d, epsilon, x0, Sig0, nt, tf):
    dt = tf/(nt-1)
    trj_xstar, ustar = optimal_deterministic_trj(A, B, a, K, d, x0, dt)
    trj_cov = propagate_Sig(A, B, K, epsilon, Sig0, nt, tf)
    return trj_xstar, trj_cov

## Closed-loop system propagation
@njit(parallel=True)
def mean_cov_cl(A, B, a, epsilon, m0, Sig0, dt):
    nx, _, nt = B.shape
    zk = np.zeros((nt, nx))
    Sk = np.zeros((nt, nx, nx))
    zk[0] = m0
    Sk[0] = Sig0

    for i in prange(nt-1):
        A_cl = A[i]
        A_cl_new = A[i+1]

        # zk
        diff_zk = A_cl @ zk[i] + a[i]
        zk_new = zk[i] + dt * diff_zk

        diff_zk_new = A_cl_new @ zk_new + a[i+1]
        zk[i+1] = zk[i] + (dt / 2) * (diff_zk + diff_zk_new)

        # Sk
        Sigt = Sk[i]
        diff_cov_i = A_cl @ Sigt + Sigt @ A_cl.T + epsilon * B[i] @ B[i].T
        Sigt_new = Sigt + dt * diff_cov_i

        diff_cov_new = A_cl_new@Sigt_new + Sigt_new@A_cl_new.T + epsilon*B[i+1] @ B[i+1].T

        Sk[i+1] = Sigt + (dt / 2) * (diff_cov_i + diff_cov_new)

    return zk, Sk

    