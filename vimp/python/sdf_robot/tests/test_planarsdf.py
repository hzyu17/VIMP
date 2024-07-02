## Test the planar signed distance field
# Hongzhe Yu, 12/20/2023

import sys
import os

current_path = os.path.abspath(__file__)
current_dir = os.path.dirname(current_path)
root_dir = os.path.dirname(current_dir)
script_dir = os.path.abspath(os.path.join(root_dir, 'scripts'))

sys.path.append(script_dir)


from generate_sdf_2d import *
from collision_costs_2d import *
import numpy as np
import matplotlib.pyplot as plt


def test_signed_distance():
    # obstacle range: (x: [10.0, 20.0]; y: [10.0, 16.0])
    sdf_2d, planarmap = generate_2dsdf("SingleObstacleMap", False)
        
    # ========== test point: the origin ([0, 0]) ========== 
    nearest_dist = np.sqrt(100.0*100.0+100.0*100.0)*0.1
    atan_gradient = np.arctan2(-10.0, -10.0)
    
    t_pt1 = np.zeros(2, dtype=np.float64)
    dist = sdf_2d.getSignedDistance(t_pt1)
    gradient_pt1 = sdf_2d.getGradient(t_pt1)
    
    assert(abs(dist - nearest_dist) < 1e-6)
    assert(abs(np.arctan2(gradient_pt1[1], gradient_pt1[0]) - atan_gradient) < 1e-2)
    
    # ========== test sdf distance and gradient ========== 
    eps_obs = 0.2
    slope = 2
    
    t_pt1 = np.zeros(2, dtype=np.float64)
    h1, g1 = hinge_sdf_loss_gradient(t_pt1, sdf_2d, eps_obs, slope)
    dist_nearest1 = 0.0
    g_nearest1 = np.zeros(2, dtype=np.float64)
    
    assert(abs(h1 - 0.0) < 1e-6)
    assert(np.linalg.norm(g1-dist_nearest1) < 1e-6)
    
    # ========== test hinge loss and gradient ==========     
    t_pt2 = np.array([13.0, 9.8], dtype=np.float64)
    dist_nearest2 = slope * 0.2
    g_nearest2 = -np.array([0, -1], dtype=np.float64)*slope
    
    h2, g2 = hinge_sdf_loss_gradient(t_pt2, sdf_2d, eps_obs, slope)
        
    assert(abs(h2 - dist_nearest2) < 1e-6)
    assert(np.linalg.norm(g2 - g_nearest2) < 1e-6)
    
    # ========== test collision loss gradient ========== 
    vec_pts = np.zeros((2, 2), dtype=np.float64)
    vec_pts[0] = t_pt1
    vec_pts[1] = t_pt2
    
    sig_obs = 0.1
    
    collision_costs = sig_obs * (h1**2 + h2**2)
    
    g_collision_pt = np.zeros(2, dtype=np.float64)
    g_collision_pt[0] = 2*sig_obs*(dist_nearest1*g_nearest1[0] + dist_nearest2*g_nearest2[0])
    g_collision_pt[1] = 2*sig_obs*(dist_nearest1*g_nearest1[1] + dist_nearest2*g_nearest2[1])
    
    col_cost, g_col_cost_pt = collision_lost_gradient(sig_obs, vec_pts, sdf_2d, eps_obs, slope)
        
    assert(abs(col_cost-collision_costs) < 1e-6)
    assert(np.linalg.norm(g_col_cost_pt-g_collision_pt) < 1e-6)
    
if __name__ == '__main__':
    test_signed_distance()