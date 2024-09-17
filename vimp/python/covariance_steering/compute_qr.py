import numpy as np
import numba
from numba import njit, prange, float64

import jax
import jax.numpy as jnp
from jax import jit

# @njit(numba.types.Tuple((float64[:,:,:], float64[:,:]))(float64[:,:,:], float64[:,:], float64[:,:,:], float64[:,:], float64[:,:], 
#        float64, float64[:,:,:], float64[:,:,:], float64[:,:], float64[:,:]))
def compute_qr(As, as_, hAk, hak, nTr, eta, B, Qt, rt, zt):
    nt = B.shape[0]
    Qk = np.zeros_like(Qt, dtype=np.float64)
    rk = np.zeros_like(rt, dtype=np.float64)
    
    for i in prange(nt):
        tmp = (As[i] - hAk[i]).T @ B[i]
        Qk[i] = (eta/(eta+1.0)) * Qt[i] + (eta /(1.0+eta)**2) * tmp@(tmp.T)
        
        c1 = eta / (eta + 1.0)
        c2 = eta / (1.0 + eta)**2
        c3 = eta / 2.0 / (1+eta)
        rk[i] = c1*(Qt[i]@zt[i] + rt[i]) + c2*(tmp@B[i].T@(as_[i]-hak[i])) + c3*nTr[i]

    return Qk, rk

