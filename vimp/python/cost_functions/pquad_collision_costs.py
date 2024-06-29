## Defining the planar quadrotor and sdf costs
# Hongzhe Yu, 12/21/2023

import os

file_path = os.path.abspath(__file__)
costfunction_dir = os.path.dirname(file_path)
root_dir = os.path.abspath(os.path.join(os.path.join(os.path.join(costfunction_dir, '..'), '..'), '..'))
col_cost_dir = os.path.abspath(os.path.join(os.path.join(os.path.join(root_dir, '3rdparty'), 'sdf_robot'), 'scripts'))

import sys
sys.path.append(col_cost_dir)
from generate_sdf_2d import *
from collision_costs_2d import *
import numpy as np

from numba.experimental import jitclass
from numba import njit, float64, int64
import numba as nb

# Defines the forward kinematics for collision-checking balls and their gradients to the states.  
def vec_balls(x, L, n_balls, radius):
    nx = x.shape[0]
    v_pts = np.zeros((n_balls, 2), dtype=np.float64)
    v_gradient_ball_states = np.zeros((n_balls, 2, nx), dtype=np.float64)
    v_radius = radius * np.ones(n_balls, dtype=np.float64)
    pos_x = x[0]
    pos_z = x[1]
    phi = x[2]
    
    l_pt_x = pos_x - (L-radius*1.5)*np.cos(phi)/2.0
    l_pt_z = pos_z - (L-radius*1.5)*np.sin(phi)/2.0
    
    for i in range(n_balls):
        pt_xi = l_pt_x + L*np.cos(phi)/n_balls*i
        pt_zi = l_pt_z + L*np.sin(phi)/n_balls*i
        v_pts[i] = np.array([pt_xi, pt_zi])
        v_gradient_ball_states[i, 0] = np.array([1.0, 0.0, (L-radius*1.5)*np.sin(phi)/2.0 - L*np.sin(phi)/n_balls*i, 0.0, 0.0, 0.0])
        v_gradient_ball_states[i, 1] = np.array([0.0, 1.0, -(L-radius*1.5)*np.cos(phi)/2.0 + L*np.cos(phi)/n_balls*i, 0.0, 0.0, 0.0])
    
    return v_pts, v_gradient_ball_states, v_radius


## accepts a planar sdf, a 3-DOF planar quadrotor state, and parameters. 
# Returns a squared hinge loss, and its gradients to the states. 
def planar_quad_hinge_gradient(x, L, n_balls, radius, sig_obs, sdf_2d, eps_obs, slope):
    # dimensions: - gradient_ball_states: (n_balls, 2, nx)
    #             - vec_gradient_col_ball: (n_balls, 2)
    #             - gradient_col_states: (1, states)
    
    v_pts, gradient_ball_states, _ = vec_balls(x, L, n_balls, radius)
    collision_cost, vec_gradient_col_ball = vec_colloss_gradient(sig_obs, v_pts, sdf_2d, eps_obs+radius, slope)
        
    # gradient_col_states = np.max(np.einsum('ij,ijk->ik', vec_gradient_col_ball, gradient_ball_states), axis=0) 
    # print("gradient_col_states")
    # print(gradient_col_states)
    gradient_col_states = np.sum(np.einsum('ij,ijk->ik', vec_gradient_col_ball, gradient_ball_states), axis=0) / n_balls
    
    return collision_cost, gradient_col_states


# spec = [
#     ('eps_obs', float64),       
#     ('slope', float64),   
#     ('sig_obs', float64),   
#     ('n_balls', int64),   
#     ('L', float64),  
#     ('H', float64),        
#     ('radius', float64)  
# ]


class PquadCollisionParams:
    def __init__(self, eps_obs, slope, sig_obs, radius, mapname, L=5.0, H=0.35, n_balls=5):
        self.eps_obs, self.slope, self.sig_obs, self.radius = eps_obs, slope, sig_obs, radius
        self.map_name = mapname
        self._L = L
        self._H = H
        self.n_balls = n_balls


# Define a jitclass
# @jitclass(spec)
class PQuadColSDF:
    def __init__(self, eps_obs, slope, sig_obs, radius, n_balls=5, L=5.0, H=0.35, map_name="SingleObstacleMap"):
        self.eps_obs = eps_obs
        self.slope = slope
        self.sig_obs = sig_obs
        self.n_balls = n_balls
        self._L = L
        self._H = H
        self.radius = radius        
        self.sdf_2d, self.planarmap = generate_2dsdf(map_name, False)

    def collision_loss_gradient(self, x):
        return planar_quad_hinge_gradient(x, self._L, self.n_balls, self.radius, self.sig_obs, self.sdf_2d, self.eps_obs, self.slope)

    def collision_loss_traj(self, xt):
        nt = xt.shape[0]
        costs = np.zeros(nt)
        for i in range(nt):
            costs[i], _ = self.collision_loss_gradient(xt[i])
        return costs
    
    def collision_loss_gradient_traj(self, xt):
        nt, nx = xt.shape
        costs = np.zeros(nt)
        gradients = np.zeros(nt, nx)
        for i in range(xt.shape[0]):
            costs[i], gradients[i] = self.collision_loss_gradient(xt[i])
        return costs, gradients
    
    def L(self):
        return self._L
    
    def H(self):
        return self._H
    
    def nball(self):
        return self.n_balls
        
if __name__ == '__main__':
    # obstacle range: (x: [10.0, 20.0]; y: [10.0, 16.0])
    sdf_2d, planarmap = generate_2dsdf("SingleObstacleMap", False)
    eps_obs = 0.2
    slope = 2.0
    sig_obs = 1.0
    n_balls = 5
    L = 5.0
    H = 0.35
    radius = 0.1
    
    x = np.array([10.0, 10.0, np.pi/4, -2.0, -1.0, 0.0], dtype=np.float64)
    
    collision_cost, gradient_col_states = planar_quad_hinge_gradient(x, L, n_balls, radius, sig_obs, sdf_2d, eps_obs, slope)
    print("collision_cost")
    print(collision_cost)
    print("g_col_states")
    print(gradient_col_states)
    

    instance = PQuadColSDF(eps_obs, slope, sig_obs, n_balls, L, H, radius)
    collision_cost, gradient_col_states = instance.collision_loss_gradient(x)
    print("collision_cost")
    print(collision_cost)
    print("g_col_states")
    print(gradient_col_states)