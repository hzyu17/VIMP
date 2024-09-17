from numba import njit, prange, types
import numpy as np

import os
file_path = os.path.abspath(__file__)
cur_dir = os.path.dirname(file_path)
py_dir = os.path.abspath(os.path.join(cur_dir, '..'))
debug_dir = os.path.abspath(os.path.join(os.path.join(py_dir, 'debug'),'linear_CS'))
import sys
sys.path.append(py_dir)

from covariance_steering.compute_qr import *
from covariance_steering.compute_qr_pquad import *
from covariance_steering.linear_cov import *
# from covariance_steering.linear_cov_solver import *
from covariance_steering.pcs_data import *
from tools.propagations import *
from tools.draw_pquadsdf_trj import *

## Visualization for debugging
import plotly.graph_objects as go
from tools.plot_samples_2d import *

from tools.logger import *
from dynamics.planar_quad import *    


# Proximal gradient covariance steering using a nominal trajectory, quadratic loss
# return values: (As, as_)
# @njit(types.Tuple((float64[:,:,:], float64[:,:]))(
#        types.FunctionType(types.Tuple((float64[:,:,:], float64[:,:,:], float64[:,:], float64[:,:]))(float64[:,:,:], float64[:,:], float64[:,:,:])), # linearize_trj
#        float64[:,:], float64[:,:,:], # zk, Sk
#        float64[:,:,:], float64[:,:,:], float64[:,:], # As, Bs, as_
#        float64[:,:,:], float64[:,:], # Qt, rt
#        float64, float64, int64, # epsilon, eta, iterations
#        float64[:], float64[:, :], float64[:], float64[:, :], # x0, Sig0, xT, SigT
#        float64)) # tf
def prox_cs_zkSk(linearize_pt,
                 linearize_trj, 
                 zk, Sk, 
                 As, Bs, as_, 
                 Qt, rt, 
                 epsilon, eta, iterations, 
                 x0, Sig0, xT, SigT, tf):
    nt = As.shape[0]
    for k in range(iterations):
        hAk, hBk, hak, nTr = linearize_trj(Sk, zk, As)
        Aprior = (eta * As + hAk) / (1 + eta)
        aprior = (eta * as_ + hak) / (1 + eta)
        
        Qk, rk = compute_qr(As, as_, hAk, hak, nTr, eta, Bs, Qt, rt, zk)

        # K, d, _, _ = linear_covcontrol(linearize_pt, get_Qr, epsilon, x0, Sig0, xT, SigT, tf, nt)
        
        K, d, _, _ = linear_covcontrol(Aprior, Bs, aprior, epsilon, Qk, rk, x0, Sig0, xT, SigT, tf)
        
        for i in range(nt):
            As[i] = Aprior[i] + hBk[i] @ K[i]
            as_[i] = aprior[i] + hBk[i] @ d[i]
        
        zk, Sk = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)  
        
    return As, as_
    

# # return values: (As, as_, Ks, ds, Pi_star, lbd_star)
# @njit(types.Tuple((float64[:,:,:], float64[:,:], float64[:,:,:], float64[:,:], float64[:,:,:], float64[:,:]))(
#        types.FunctionType(types.Tuple((float64[:,:], float64[:,:], float64[:]))(float64[:])), # linearization_pt
#        types.FunctionType(types.Tuple((float64[:,:,:], float64[:,:,:], float64[:,:], float64[:,:]))(float64[:,:,:], float64[:,:], float64[:,:,:])), # linearization_trj
#        float64[:,:,:], float64[:,:], # Qt, rt
#        float64, float64, int64, # epsilon, eta, iterations
#        float64[:], float64[:, :], float64[:], float64[:, :], # x0, Sig0, xT, SigT
#        float64)) # tf
#     #   parallel=True)
def proximal_cov(linearize_pt, linearize_trj, 
                 Qt, rt, 
                 epsilon, eta, iterations, 
                 x0, Sig0, xT, SigT, tf):
    
    nt = Qt.shape[0]
    
    nx = x0.shape[0]
    
    hA1, hB1, ha1 = linearize_pt(x0)

    nu = hB1.shape[1]
    
    As = np.zeros((nt, nx, nx), dtype=np.float64)
    Bs = np.zeros((nt, nx, nu), dtype=np.float64)
    as_ = np.zeros((nt, nx), dtype=np.float64)
    
    for i in range(nt):
        Bs[i] = hB1
    
    zk, Sk = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
    
    As, as_ = prox_cs_zkSk(linearize_pt, linearize_trj, zk, Sk, As, Bs, as_, Qt, rt, epsilon, eta, iterations, x0, Sig0, xT, SigT, tf)
    
    # Recover optimal control
    zkstar, Skstar = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
    hAstar, hBstar, hastar, nTr = linearize_trj(Skstar, zkstar, As)

    # Costs for recovering optimal control
    Qstar = Qt
    rstar = nTr / 2.0
    # Ks, ds, Pi_star, lbd_star = linear_covcontrol(linearize_pt, get_Qr, epsilon, x0, Sig0, xT, SigT, tf, nt)
    Ks, ds, Pi_star, lbd_star = linear_covcontrol(hAstar, hBstar, hastar, epsilon, Qstar, rstar, x0, Sig0, xT, SigT, tf)
    
    return As, as_, Ks, ds, Pi_star, lbd_star
