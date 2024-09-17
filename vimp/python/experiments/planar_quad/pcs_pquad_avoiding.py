## Collision avoiding experiment for a planar quadrotor
# Hongzhe Yu
# 05/29/2024

import sys
import os

exp_script_dir = os.path.dirname(__file__)
py_dir = os.path.abspath(os.path.join(os.path.join(exp_script_dir, '..'),'..'))
collision_cost_dir = os.path.abspath(os.path.join(os.path.join(py_dir, 'sdf_robot'), 'scripts'))

sys.path.append(collision_cost_dir)
sys.path.append(py_dir)

from dynamics.planar_quad import *
from tools.draw_pquadsdf_trj import *
import numpy as np
from covariance_steering.prox_cov_sdf import *

def construct_Qtrt(x0, nt, xT):
    nx = len(x0)

    # running cost
    Qt = np.zeros((nt, nx, nx), dtype=np.float64)
    rt = np.zeros((nt, nx), dtype=np.float64)
        
    return Qt, rt


from sdf_robot.scripts.generate_sdf_2d import *

if __name__ == '__main__':
    nx = 6
    nu = 2
    
    # ============== 
    # boundary mean
    # ==============
    
    # ============================= 4 experiment settings =============================
    exp_data = {}
    
    for exp_index in range(1,5):
    
        if (exp_index==1):
            # -------
            # exp 1
            # -------
            x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
            xT = np.array([25.0, 30.0, np.pi/6, -1.0, 2.0, 0.01], dtype=np.float64)
            eps_obs = 0.8
            slope = 5.0
            sig_obs = 80
            radius = 1.0
            tf = 4.0
        elif exp_index == 2:
            # -------
            # exp 2
            # -------
            x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
            xT = np.array([17.0, 35.0, -np.pi/8, 1.0, 2.0, 0.01], dtype=np.float64)
            eps_obs = 0.8
            slope = 5.0
            sig_obs = 50
            radius = 1.2
            tf = 4.5
        elif exp_index == 3:
            # -------
            # exp 3
            # -------
            x0 = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
            xT = np.array([5.0, 35.0, -np.pi/6, 1.0, 1.0, -0.02], dtype=np.float64)
            eps_obs = 0.8
            slope = 5.0
            sig_obs = 80
            radius = 1.0
            tf = 4.0
        elif exp_index == 4:
            # -------
            # exp 4
            # -------
            x0 = np.array([25.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
            xT = np.array([12.0, 35.0, np.pi/6, -2.0, 1.0, 0.0], dtype=np.float64)
            eps_obs = 0.8
            slope = 5.0
            sig_obs = 85
            radius = 1.0
            tf = 4.0
        
        # ============================= End of 4 experiment settings =============================
        
        x0[3:6] = (xT[0:3] - x0[0:3]) / tf
        
        # ===================== 
        # boundary covariance
        # =====================
        coeff_Sig0 = 0.01
        coeff_SigT = 0.01

        Sig0 = np.eye(nx)*coeff_Sig0
        SigT = np.eye(nx)*coeff_SigT
        
        nt = 3000
        dt = tf / (nt-1)
        
        epsilon = 1.0
        n_iter = 20
        n_iter_linesearch = 5
        stepsize = 3e-1
        
        # --------------------------------------------- 
        # Define the obstacle object and collision cost
        # --------------------------------------------- 
        
        Qt, rt = construct_Qtrt(x0, nt, xT)
        
        As = np.zeros((nt, nx, nx), dtype=np.float64)
        Bs = np.zeros((nt, nx, nu), dtype=np.float64)
        as_ = np.zeros((nt, nx), dtype=np.float64)
        
        # -------------------- 
        # PGCS initialization
        # -------------------- 
        hA1, hB1, ha1 = planarquad_linearization_pt(x0)

        nu = hB1.shape[1]
        
        for i in range(nt):
            Bs[i] = hB1
            
        # ----------------------- 
        # nominal initialization
        # ----------------------- 
        zk, Sk = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
        
        # --------------------------------------------- 
        # PGCS algorithm for linearized ltv system
        # --------------------------------------------- 
        As, as_, zk, Sk, gradient_col_states_nt = proximal_cov_pquadsdf_zkSk(planarquad_linearization, zk, Sk, 
                                                                            As, Bs, as_, Qt, rt, 
                                                                            eps_obs, slope, sig_obs, radius, 
                                                                            epsilon, stepsize, n_iter, n_iter_linesearch,
                                                                            x0, Sig0, xT, SigT, tf)
        
        # ------------------------- 
        # Recover final ltv system
        # ------------------------- 
        zkstar, Skstar = mean_cov_cl(As, Bs, as_, epsilon, x0, Sig0, tf)
    
        zkSk_data_i = {"zk": zkstar, "Sk": Skstar}
        
        exp_data[exp_index] = zkSk_data_i
    
    # save data
    import pickle
    with open(exp_script_dir+'/four_exp_data.pkl', 'wb') as file:
        pickle.dump(exp_data, file)
    