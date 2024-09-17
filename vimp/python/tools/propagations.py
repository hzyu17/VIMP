## Defines the propagation functions for the mean and for the covariance, 
# for both feedback system and close-loop systems
# Hongzhe YU

import numba
import numpy as np
from numba import njit, prange
from numba import float64, int64

# @njit(numba.types.Tuple((float64[:,:], float64[:,:]))(float64[:,:,:], float64[:,:,:], float64[:,:], # A, B, a
#                                                       float64[:,:,:], float64[:,:], # K, d
#                                                       float64[:], float64))
def optimal_deterministic_trj(A, B, a, K, d, x0, tf):
    nt, nu, nx = K.shape
    dt = tf / (nt -1)    
    xs = np.zeros((nt, nx))
    xs[0] = x0
    ustar = np.zeros((nt, nu))

    xt = x0
    # A = np.ascontiguousarray(A)
    # B = np.ascontiguousarray(B)
    # K = np.ascontiguousarray(K)
    # xt = np.ascontiguousarray(xt)
    
    for i in range(nt - 1):
        ut = K[i] @ xt + d[i]
        ustar[i] = ut
        
        # ---------- mean ----------
        x_next = xt + (A[i] @ xt + B[i] @ ut + a[i]) * dt
        ut_next = K[i+1] @ x_next + d[i+1]

        xs[i+1] = xt + (A[i] @ xt + B[i] @ ut + a[i] +
                        A[i+1] @ x_next + B[i+1] @ ut_next + a[i+1]) * dt / 2
                            
        xt = xs[i+1]

    return xs, ustar

# @njit(float64[:,:,:](float64[:,:,:], float64[:,:,:], float64[:,:,:], float64, float64[:,:], float64))
def propagate_Sig(A, B, K, epsilon, Sig0, tf):
    nt, nu, nx = K.shape
    
    trj_cov = np.ascontiguousarray(np.zeros((nt, nx, nx)))
    trj_cov[0] = Sig0
    dt = tf / (nt - 1)

    B = np.ascontiguousarray(B)
    A = np.ascontiguousarray(A)
    K = np.ascontiguousarray(K)
    
    for i in range(nt - 1):
        # ---------- covariance ----------
        A[i] = A[i] + np.dot(B[i], K[i])
        diff_cov_i = np.dot(A[i], trj_cov[i]) + np.dot(trj_cov[i], A[i].T) + epsilon * np.dot(B[i], B[i].T)
        Sigt_new = trj_cov[i] + dt * diff_cov_i

        A[i+1] = A[i+1] + np.dot(B[i+1], K[i+1])
        diff_cov_new = np.dot(A[i+1], Sigt_new) + np.dot(Sigt_new, A[i+1].T) + epsilon * np.dot(B[i+1], B[i+1].T)
        trj_cov[i+1] = trj_cov[i] + (dt / 2) * (diff_cov_i + diff_cov_new)

    return trj_cov

# @njit
def mean_cov(A, B, a, K, d, epsilon, x0, Sig0, nt, tf):
    dt = tf/(nt-1)
    trj_xstar, ustar = optimal_deterministic_trj(A, B, a, K, d, x0, dt)
    trj_cov = propagate_Sig(A, B, K, epsilon, Sig0, tf)
    return trj_xstar, trj_cov

# @njit(numba.types.Tuple((float64[:,:], float64[:,:,:]))(float64[:,:,:], float64[:,:,:], float64[:,:], # A, B, a
#                                                         float64, # epsilon
#                                                         float64[:], float64[:,:], float64)) # m0, Sig0, tf
def mean_cov_cl(A, B, a, epsilon, m0, Sig0, tf):
    from scipy.integrate import solve_ivp
    nt = A.shape[0]
    nx = m0.shape[0]
    dt = tf / (nt - 1)

    def odes(t, y):

        zk = y[:nx].flatten()
        Sk = y[nx:].reshape(nx, nx)
        
        # Compute the current time index
        i = min(int(t / dt), nt - 1)
        
        # Differential equations
        d_zk = A[i] @ zk + a[i]
        d_Sk = A[i] @ Sk + Sk @ A[i].T + epsilon * B[i] @ B[i].T
        
        return np.concatenate([d_zk, d_Sk.flatten()])

    # Initial condition
    initial_conditions = np.concatenate([m0, Sig0.flatten()])
    t_span = (0, tf)
    t_eval = np.linspace(0, tf, nt)

    # Solve the ODE
    result = solve_ivp(odes, t_span, initial_conditions, method='RK23', t_eval=t_eval, vectorized=True)

    # Extract zk and Sk from the solution
    zk = result.y[:nx, :].T
    Sk = np.transpose(result.y[nx:, :].reshape(nx, nx, nt), (2,0,1))

    return zk, Sk



def mean_cov_cl_euler(A, B, a, epsilon, m0, Sig0, tf):
    nt = A.shape[0]
    zk = np.zeros_like(a)
    Sk = np.zeros_like(A)
    
    zk[0] = m0
    Sk[0] = Sig0
    
    dt = tf / (nt-1)
    
    for i in range(nt-1):
        # zk
        diff_zk = A[i] @ zk[i] + a[i]
        zk_new = zk[i] + dt * diff_zk

        diff_zk_new = A[i+1] @ zk_new + a[i+1]
        zk[i+1] = zk[i] + (dt / 2) * (diff_zk + diff_zk_new)

        # Sk
        diff_cov_i = A[i] @ Sk[i] + Sk[i] @ A[i].T + epsilon * B[i] @ B[i].T
        Sigt_new = Sk[i] + dt * diff_cov_i

        diff_cov_new = A[i+1]@Sigt_new + Sigt_new@A[i+1].T + epsilon*B[i+1] @ B[i+1].T

        Sk[i+1] = Sk[i] + (dt / 2) * (diff_cov_i + diff_cov_new)

    return zk, Sk


def compute_control_signal(m0, Ks, ds):
    nt, nu, nx = Ks.shape
    ut = np.zeros((nt, nu))
    xt = m0
    for i in range(nt):
        ut[i] = Ks[i]@xt + ds[i]
    return ut
    
## Trajectory roll outs for a given nonlinear system, and compute the empirical mean and covariance
@njit((numba.types.FunctionType(float64[:,:](float64[:], float64[:,:,:], float64[:,:], float64, float64)), # feedback_sde function
       float64[:], float64[:,:], # x0, Sig0
       float64[:,:,:], float64[:,:], # Ks, ds
       float64, float64, int64), # tf, epsilon, nsamples
      parallel=True)
def rollout_nl(feedback_sde, x0, Sig0, Ks, ds, tf, epsilon, nsamples):
    nt, nu, nx = Ks.shape
    
    # Initialize trajectory_samples array
    trajectory_samples = np.zeros((nt, nx, nsamples), dtype=np.float64)
    eval_Sig0, evec_Sig0 = np.linalg.eigh(Sig0)
    sqrt_Sig0 = evec_Sig0 @ np.diag(np.sqrt(eval_Sig0)) @ evec_Sig0.T
    
    for i_s in prange(nsamples):
        # sample from the initial Gaussian distribution
        x0_sample = x0 + sqrt_Sig0@np.random.randn(nx)
        trajectory_samples[:,:,i_s] = feedback_sde(x0_sample, Ks, ds, tf, epsilon)

    # Empirical terminal covariance of nonlinear rollout
    samples_tf = trajectory_samples[-1]
        
    mean_tf = np.sum(samples_tf, axis=1) / nsamples
        
    sample_minus_mean = samples_tf - mean_tf.reshape((nx, 1))
    
    cov_i = np.zeros((nx, nx), dtype=np.float64)
    for i_s in prange(nsamples):
        cov_i += np.outer(sample_minus_mean[:, i_s], sample_minus_mean[:, i_s])
    cov_tf_nl = cov_i / nsamples
    
    return trajectory_samples, mean_tf, cov_tf_nl