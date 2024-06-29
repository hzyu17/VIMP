## Dynamics for a 2d quadrotor in a control-affine form
# Hongzhe Yu
# 12/19/2023

import os

file_path = os.path.abspath(__file__)
dynamics_dir = os.path.dirname(file_path)
col_cost_dir = os.path.abspath(os.path.join(os.path.join(os.path.join(dynamics_dir, '..'), 'sdf_robot'), 'scripts'))

import sys
sys.path.append(col_cost_dir)

from collision_costs_2d import *

import numba
from numba import njit, float64, prange
import numpy as np

global g, m, l, J
g = 9.81
m=0.486;
J=0.00383;
l=0.25;

@njit(float64[:,:](float64[:], float64[:,:,:], float64[:,:], float64, float64), parallel=True,)
def feedback_planarquad_sde_trj(x0, K, d, tf, eps):
    global g, m, l, J
    
    nt, nu, nx = K.shape
    
    x_trj = np.ascontiguousarray(np.zeros((nt, nx), dtype=np.float64))
    x_trj[0] = x0
    dt = tf / (nt-1)
    
    # x_next = np.zeros(nx, dtype=np.float64)
    # ut = np.zeros(nu, dtype=np.float64)
    # ut_next = np.zeros(nu, dtype=np.float64)
    # fx = np.zeros(nx, dtype=np.float64)
    # gu = np.zeros(nx, dtype=np.float64)
    
    B = np.ascontiguousarray(np.asarray([[0.0, 0.0], 
                                        [0.0, 0.0], 
                                        [0.0, 0.0], 
                                        [0.0, 0.0], 
                                        [1.0/np.sqrt(2), 1.0/np.sqrt(2)], 
                                        [1.0/np.sqrt(2), -1.0/np.sqrt(2)]], dtype=np.float64))
    
    K = np.ascontiguousarray(K)

    dWt = np.ascontiguousarray(np.random.randn(nt,nu))
    BdWt = np.zeros((nt, nx), dtype=np.float64)
    
    for i in prange(nt):
        BdWt[i] = np.sqrt(eps * dt) * B @ dWt[i]
    
    for i in range(nt-1):       
        fx = np.array([x_trj[i,3]*np.cos(x_trj[i,2]) - x_trj[i,4]*np.sin(x_trj[i,2]),
                        x_trj[i,3]*np.sin(x_trj[i,2]) + x_trj[i,4]*np.cos(x_trj[i,2]),
                        x_trj[i,5],
                        x_trj[i,4]*x_trj[i,5] - g*np.sin(x_trj[i,2]),
                        -x_trj[i,3]*x_trj[i,5] - g*np.cos(x_trj[i,2]),
                        0.0], dtype=np.float64)
        
        gu = B@(K[i]@x_trj[i] + d[i])
        
        x_next = x_trj[i] + (fx + gu) * dt
        ut_next = K[i+1]@x_next + d[i+1]
        
        fx_next = np.array([x_next[3]*np.cos(x_next[2]) - x_next[4]*np.sin(x_next[2]),
                            x_next[3]*np.sin(x_next[2]) + x_next[4]*np.cos(x_next[2]),
                            x_next[5],
                            x_next[4]*x_next[5] - g*np.sin(x_next[2]),
                            -x_next[3]*x_next[5] - g*np.cos(x_next[2]),
                            0.0], dtype=np.float64)
        
        gu_next = B@ut_next

        x_trj[i+1] = x_trj[i] + (fx + gu + fx_next + gu_next) * (dt/2) + BdWt[i]

    return x_trj


def propagate_linearized_feedback(As, as_, Ks, ds, x0, tf, eps):
    global g, m, l, J
    
    nt, nu, nx = Ks.shape
    
    x_trj = np.zeros((nt, nx), dtype=np.float64)
    x_trj[0] = x0
    dt = tf / (nt-1)
    
    B = np.asarray([[0.0, 0.0], 
                    [0.0, 0.0], 
                    [0.0, 0.0], 
                    [0.0, 0.0], 
                    [1.0/np.sqrt(2), 1.0/np.sqrt(2)], 
                    [1.0/np.sqrt(2), -1.0/np.sqrt(2)]], dtype=np.float64)
    

    dWt = np.random.randn(nt,nu)
    BdWt = np.zeros((nt, nx), dtype=np.float64)
    
    for i in range(nt):
        BdWt[i] = np.sqrt(eps * dt) * B @ dWt[i]
    
    xt = x0
    x_trj[0] = xt
    for i in range(nt-1):       
        fx = As[i]@xt + as_[i]
        gu = B@(Ks[i]@x_trj[i] + ds[i])
            
        x_trj[i+1] = x_trj[i] + (fx + gu)* dt + BdWt[i]

    return x_trj


# @njit(float64[:](float64[:], float64[:]), parallel=True)
def fxgu_nl(xt, ut):
    g = 9.81
    fx = np.array([xt[3]*np.cos(xt[2]) - xt[4]*np.sin(xt[2]), 
                   xt[3]*np.sin(xt[2]) + xt[4]*np.cos(xt[2]), 
                   xt[5], 
                   xt[4]*xt[5] - g*np.sin(xt[2]), 
                   -xt[3]*xt[5] - g*np.cos(xt[2]), 
                   0.0], dtype=np.float64)
    
    B = np.array([[0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0],
                  [1/np.sqrt(2.0), 1/np.sqrt(2.0)],
                  [1/np.sqrt(2.0), -1/np.sqrt(2.0)]], dtype=np.float64)
    
    fxgu = fx + B@ut
    return fxgu

def fxgu_nl_odesolver(t, xt, *args):
    ut = args[0]
    g = 9.81
    fx = np.array([xt[3]*np.cos(xt[2]) - xt[4]*np.sin(xt[2]), 
                   xt[3]*np.sin(xt[2]) + xt[4]*np.cos(xt[2]), 
                   xt[5], 
                   xt[4]*xt[5] - g*np.sin(xt[2]), 
                   -xt[3]*xt[5] - g*np.cos(xt[2]), 
                   0.0], dtype=np.float64)
    
    B = np.array([[0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0],
                  [1/np.sqrt(2.0), 1/np.sqrt(2.0)],
                  [1/np.sqrt(2.0), -1/np.sqrt(2.0)]], dtype=np.float64)
    
    return fx + B@ut


# @njit(float64[:](float64[:], float64), parallel=True)
def gdWt(dWt, eps):
    B = np.array([[0.0, 0.0],
                [0.0, 0.0],
                [0.0, 0.0],
                [0.0, 0.0],
                [1.0/np.sqrt(2), 1.0/np.sqrt(2)],
                [1.0/np.sqrt(2), -1.0/np.sqrt(2)]], dtype=np.float64)
    return np.sqrt(eps) * B@dWt

# linearize around the nominal x (1 point)
# @njit(numba.types.Tuple((float64[:,:], float64[:,:], float64[:]))(float64[:]), parallel=True)
def planarquad_linearization_pt(pt):
    global g, m, l, J
    hA = np.array([[0.0, 0.0, -pt[3]*np.sin(pt[2]) - pt[4]*np.cos(pt[2]), np.cos(pt[2]), -np.sin(pt[2]), 0], 
                    [0.0, 0.0, pt[3]*np.cos(pt[2]) - pt[4]*np.sin(pt[2]), np.sin(pt[2]), np.cos(pt[2]), 0], 
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0], 
                    [0.0, 0.0, -g*np.cos(pt[2]), 0.0, pt[5], pt[4]], 
                    [0.0, 0.0, g*np.sin(pt[2]), -pt[5], 0, -pt[3]], 
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float64)
    
    fx_pt = np.array([pt[3]*np.cos(pt[2]) - pt[4]*np.sin(pt[2]),
                    pt[3]*np.sin(pt[2]) + pt[4]*np.cos(pt[2]),
                    pt[5],
                    pt[4]*pt[5] - g*np.sin(pt[2]),
                    -pt[3]*pt[5] - g*np.cos(pt[2]),
                    0.0], dtype=np.float64)
    
    hB = np.array([[0.0, 0.0],
                    [0.0, 0.0],
                    [0.0, 0.0],
                    [0.0, 0.0],
                    [1.0/np.sqrt(2), 1.0/np.sqrt(2)],
                    [1.0/np.sqrt(2), -1.0/np.sqrt(2)]], dtype=np.float64)
    
    ha = fx_pt - hA@pt
    
    return hA, hB, ha

# @njit(numba.types.Tuple((float64[:,:,:], 
#                          float64[:,:,:], 
#                          float64[:,:], 
#                          float64[:,:]))(float64[:,:,:], 
#                                         float64[:,:], 
#                                         float64[:,:,:]), parallel=True)
def planarquad_linearization(St, zt, At):
    global g, m, l, J
    
    nt, nx = zt.shape
    hA = np.zeros((nt, nx, nx), dtype=np.float64)
    ha = np.zeros((nt, nx), dtype=np.float64)
    nTr = np.zeros((nt, nx), dtype=np.float64)
    hB = np.zeros((nt, nx, 2), dtype=np.float64)
    
    hBi = np.array([[0.0, 0.0], 
                    [0.0, 0.0], 
                    [0.0, 0.0], 
                    [0.0, 0.0], 
                    [1.0/np.sqrt(2), 1.0/np.sqrt(2)], 
                    [1.0/np.sqrt(2), -1.0/np.sqrt(2)]], dtype=np.float64)
    
    for i in prange(nt):
        # fzi = fx(zt[i])
        # hAi = jacfwd(fx)(zt[i])
        
        fzi = np.array([zt[i,3]*np.cos(zt[i,2]) - zt[i,4]*np.sin(zt[i,2]),
                                zt[i,3]*np.sin(zt[i,2]) + zt[i,4]*np.cos(zt[i,2]),
                                zt[i,5],
                                zt[i,4]*zt[i,5] - g*np.sin(zt[i,2]),
                               -zt[i,3]*zt[i,5] - g*np.cos(zt[i,2]),
                                0.0], dtype=np.float64)
        
        hA[i] = np.array([[0.0, 0.0, -zt[i,3]*np.sin(zt[i,2]) - zt[i,4]*np.cos(zt[i,2]), np.cos(zt[i,2]), -np.sin(zt[i,2]), 0.0], 
                        [0.0, 0.0, zt[i,3]*np.cos(zt[i,2]) - zt[i,4]*np.sin(zt[i,2]), np.sin(zt[i,2]), np.cos(zt[i,2]), 0.0], 
                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0], 
                        [0.0, 0.0, -9.81*np.cos(zt[i,2]), 0.0, zt[i,5], zt[i,4]], 
                        [0.0, 0.0, 9.81*np.sin(zt[i,2]), -zt[i,5], 0.0, -zt[i,3]], 
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                        ], dtype=np.float64)
        
        hB[i] = hBi
        ha[i] = fzi - hA[i]@zt[i]
        
        # tr = lambda x: jnp.trace(gradf_TBBTgradf_fn(x)@St[i])
        # nTr[i] = jacfwd(tr)(zt[i])
        
        # grad_nabla_f = BBTgradf_fn(zt[i])@St[i]
        # nTri = np.zeros(nx, dtype=np.float64)
        # nnz_x, nnz_y = jnp.nonzero(hAi)
        # for l,m in zip(nnz_x, nnz_y):
        #     jacobian_fn_nnz = lambda x: jacobian_fn(x)[l, m]
        #     nTri = nTri + grad_nabla_f[l, m]*jacfwd(jacobian_fn_nnz)(zt[i])
        
        # nTr[i] = nTri
            
        # analytical    
        nTr[i] = np.array([0.0, 0.0, 96.2361*St[i,2,2]*np.sin(2*zt[i,2])-9.81*St[i,2,3]*zt[i,5]*np.cos(zt[i,2]) - 9.81*St[i,2,5]*zt[i,3]*np.cos(zt[i,2]) - 9.81*St[i,3,2]*zt[i,5]*np.cos(zt[i,2]) - 9.81*St[i,5,2]*zt[i,3]*np.cos(zt[i,2]) - 19.6200*At[i,4,0]*St[i,2,0]*np.cos(zt[i,2]) - 19.6200*At[i,4,1]*St[i,2,1]*np.cos(zt[i,2]) - 19.6200*At[i,4,2]*St[i,2,2]*np.cos(zt[i,2]) - 19.6200*At[i,4,3]*St[i,2,3]*np.cos(zt[i,2]) - 19.6200*At[i,4,4]*St[i,2,4]*np.cos(zt[i,2]) - 19.6200*At[i,4,5]*St[i,2,5]*np.cos(zt[i,2]), 
                           St[i,3,5]*zt[i,5] + St[i,5,3]*zt[i,5] + 2*St[i,5,5]*zt[i,3] - 9.81*St[i,2,5]*np.sin(zt[i,2]) - 9.81*St[i,5,2]*np.sin(zt[i,2]) + 2*At[i,4,0]*St[i,5,0] + 2*At[i,4,1]*St[i,5,1] + 2*At[i,4,2]*St[i,5,2] + 2*At[i,4,3]*St[i,5,3] + 2*At[i,4,4]*St[i,5,4] + 2*At[i,4,5]*St[i,5,5], 
                           0.0, 
                           2*St[i,3,3]*zt[i,5] + St[i,3,5]*zt[i,3] + St[i,5,3]*zt[i,3] - 9.81*St[i,2,3]*np.sin(zt[i,2]) - 9.81*St[i,3,2]*np.sin(zt[i,2]) + 2*At[i,4,0]*St[i,3,0] + 2*At[i,4,1]*St[i,3,1] + 2*At[i,4,2]*St[i,3,2] + 2*At[i,4,3]*St[i,3,3] + 2*At[i,4,4]*St[i,3,4] + 2*At[i,4,5]*St[i,3,5]
                           ], dtype=np.float64)
        
        # nTr[i] = np.array([0.0, 0.0, 11.3653*St[i,2,2]*np.sin(2*zt[i,2]) - 1.1585*St[i,2,3]*zt[i,5]*np.cos(zt[i,2]) - 1.1585*St[i,2,5]*zt[i,3]*np.cos(zt[i,2]) - 1.1585*St[i,3,2]*zt[i,5]*np.cos(zt[i,2]) - 1.1585*St[i,5,2]*zt[i,3]*np.cos(zt[i,2]) - 2.3171*At[i,4,0]*St[i,2,0]*np.cos(zt[i,2]) - 2.3171*At[i,4,1]*St[i,2,1]*np.cos(zt[i,2]) - 2.3171*At[i,4,2]*St[i,2,2]*np.cos(zt[i,2]) - 2.3171*At[i,4,3]*St[i,2,3]*np.cos(zt[i,2]) - 2.3171*At[i,4,4]*St[i,2,4]*np.cos(zt[i,2]) - 2.3171*At[i,4,5]*St[i,2,5]*np.cos(zt[i,2]),
        #                    0.1181*St[i,3,5]*zt[i,5] + 0.1181*St[i,5,3]*zt[i,5] + 0.2362*St[i,5,5]*zt[i,3] - 1.1585*St[i,2,5]*np.sin(zt[i,2]) - 1.1585*St[i,5,2]*np.sin(zt[i,2]) + 0.2362*At[i,4,0]*St[i,5,0] + 0.2362*At[i,4,1]*St[i,5,1] + 0.2362*At[i,4,2]*St[i,5,2] + 0.2362*At[i,4,3]*St[i,5,3] + 0.2362*At[i,4,4]*St[i,5,4] + 0.2362*At[i,4,5]*St[i,5,5],
        #                    0.0,
        #                    0.2362*St[i,3,3]*zt[i,5] + 0.1181*St[i,3,5]*zt[i,3] + 0.1181*St[i,5,3]*zt[i,3] - 1.1585*St[i,2,3]*np.sin(zt[i,2]) - 1.1585*St[i,3,2]*np.sin(zt[i,2]) + 0.2362*At[i,4,0]*St[i,3,0] + 0.2362*At[i,4,1]*St[i,3,1] + 0.2362*At[i,4,2]*St[i,3,2] + 0.2362*At[i,4,3]*St[i,3,3] + 0.2362*At[i,4,4]*St[i,3,4] + 0.2362*At[i,4,5]*St[i,3,5]], 
        #                   dtype=np.float64)
        
        # 10.0*hBi
        # nTr[i] = np.array([0.0, 0.0, 0.9624*St[i,2,2]*np.sin(2*zt[i,2]) - 0.0981*St[i,2,3]*zt[i,5]*np.cos(zt[i,2]) - 0.0981*St[i,2,5]*zt[i,3]*np.cos(zt[i,2]) - 0.0981*St[i,3,2]*zt[i,5]*np.cos(zt[i,2]) - 0.0981*St[i,5,2]*zt[i,3]*np.cos(zt[i,2]) - 0.1962*At[i,4,0]*St[i,2,0]*np.cos(zt[i,2]) - 0.1962*At[i,4,1]*St[i,2,1]*np.cos(zt[i,2]) - 0.1962*At[i,4,2]*St[i,2,2]*np.cos(zt[i,2]) - 0.1962*At[i,4,3]*St[i,2,3]*np.cos(zt[i,2]) - 0.1962*At[i,4,4]*St[i,2,4]*np.cos(zt[i,2]) - 0.1962*At[i,4,5]*St[i,2,5]*np.cos(zt[i,2]),
        #                    0.0100*St[i,3,5]*zt[i,5] + 0.0100*St[i,5,3]*zt[i,5] + 0.0200*St[i,5,5]*zt[i,3] - 0.0981*St[i,2,5]*np.sin(zt[i,2]) - 0.0981*St[i,5,2]*np.sin(zt[i,2]) + 0.0200*At[i,4,0]*St[i,5,0] + 0.0200*At[i,4,1]*St[i,5,1] + 0.0200*At[i,4,2]*St[i,5,2] + 0.0200*At[i,4,3]*St[i,5,3] + 0.0200*At[i,4,4]*St[i,5,4] + 0.0200*At[i,4,5]*St[i,5,5], 
        #                    0.0,
        #                    0.0200*St[i,3,3]*zt[i,5] + 0.0100*St[i,3,5]*zt[i,3] + 0.0100*St[i,5,3]*zt[i,3] - 0.0981*St[i,2,3]*np.sin(zt[i,2]) - 0.0981*St[i,3,2]*np.sin(zt[i,2]) + 0.0200*At[i,4,0]*St[i,3,0] + 0.0200*At[i,4,1]*St[i,3,1] + 0.0200*At[i,4,2]*St[i,3,2] + 0.0200*At[i,4,3]*St[i,3,3] + 0.0200*At[i,4,4]*St[i,3,4] + 0.0200*At[i,4,5]*St[i,3,5]
        #                    ], 
        #                   dtype=np.float64)
        
    return hA, hB, ha, nTr


def planarquad_linearization_deterministic(zt):
    global g, m, l, J
    
    nt, nx = zt.shape
    hA = np.zeros((nt, nx, nx), dtype=np.float64)
    ha = np.zeros((nt, nx), dtype=np.float64)
    hB = np.zeros((nt, nx, 2), dtype=np.float64)
    
    hBi = np.array([[0.0, 0.0], 
                    [0.0, 0.0], 
                    [0.0, 0.0], 
                    [0.0, 0.0], 
                    [1.0/np.sqrt(2), 1.0/np.sqrt(2)], 
                    [1.0/np.sqrt(2), -1.0/np.sqrt(2)]], dtype=np.float64)
    
    for i in prange(nt):
        fzi = np.array([zt[i,3]*np.cos(zt[i,2]) - zt[i,4]*np.sin(zt[i,2]),
                                zt[i,3]*np.sin(zt[i,2]) + zt[i,4]*np.cos(zt[i,2]),
                                zt[i,5],
                                zt[i,4]*zt[i,5] - g*np.sin(zt[i,2]),
                               -zt[i,3]*zt[i,5] - g*np.cos(zt[i,2]),
                                0.0], dtype=np.float64)
        
        hA[i] = np.array([[0.0, 0.0, -zt[i,3]*np.sin(zt[i,2]) - zt[i,4]*np.cos(zt[i,2]), np.cos(zt[i,2]), -np.sin(zt[i,2]), 0.0], 
                        [0.0, 0.0, zt[i,3]*np.cos(zt[i,2]) - zt[i,4]*np.sin(zt[i,2]), np.sin(zt[i,2]), np.cos(zt[i,2]), 0.0], 
                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0], 
                        [0.0, 0.0, -9.81*np.cos(zt[i,2]), 0.0, zt[i,5], zt[i,4]], 
                        [0.0, 0.0, 9.81*np.sin(zt[i,2]), -zt[i,5], 0.0, -zt[i,3]], 
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                        ], dtype=np.float64)
        
        hB[i] = hBi
        ha[i] = fzi - hA[i]@zt[i]
            
    return hA, hB, ha


def get_Qr(x):
    nx = len(x)
    coeff_Q = 1.0
    
    xT = np.zeros(nx, dtype=np.float64)
    
    Q = coeff_Q * np.eye(nx)
    r = -2.0*Q@xT
    
    return Q, r

