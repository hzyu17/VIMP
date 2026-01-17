## Compute Q and r for planar quadrotor with obstacles
# 01/24/2024

import numpy as np
from cost_functions.pquad_collision_costs import *
from covariance_steering.compute_qr import *

def compute_qr_pquad(As, as_, hAk, hak, nTr, eta, B, Qt, rt, zt, pquadsdf):
    Qk, rk = compute_qr(As, as_, hAk, hak, nTr, eta, B, Qt, rt, zt)
    nx = rk.shape[1]
    nt = B.shape[0]
    col_costs = 0.0
    
    gradient_col_states_nt = np.zeros((nt, nx), dtype=np.float64)
    for i in range(nt):       
        col_cost, gradient_col_states = pquadsdf.collision_loss_gradient(zt[i])
        col_costs += col_cost
        gradient_col_states_nt[i] = eta * gradient_col_states / (eta + 1.0) 
    
    print("np.linalg.norm(rk): ", np.linalg.norm(rk))
    print("np.linalg.norm(gradient_col_states_nt): ", np.linalg.norm(gradient_col_states_nt))
    
    rk = rk + gradient_col_states_nt

    return Qk, rk, col_costs, gradient_col_states_nt

