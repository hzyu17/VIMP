import numpy as np
from numba import prange, njit
import matplotlib.pyplot as plt
from matplotlib.patches import PathPatch
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.text import TextPath
from matplotlib.transforms import Affine2D

import sys
import os

file_path = os.path.abspath(__file__)
current_dir = os.path.dirname(file_path)

python_dir = os.path.abspath(os.path.join(current_dir, '..'))
col_lib_dir = os.path.abspath(os.path.join(os.path.join(os.path.join(python_dir, '..'), '..'), 'thirdparty/sdf_robot/scripts'))
col_example_dir = os.path.abspath(os.path.join(os.path.join(os.path.join(python_dir, '..'), '..'), 'thirdparty/sdf_robot/example'))
sys.path.append(python_dir)

# import planar quadrotor dynamics
from dynamics.planar_quad import *

# 2d map plotting 
from sdf_robot.scripts.generate_sdf_2d import * 
from sdf_robot.example.draw_planar_quad_sdf import *

# Defines the forward kinematics for collision-checking balls and their gradients to the states.  
@njit
def vec_balls(x, L, n_balls, radius):
    v_pts = np.zeros((n_balls, 2), dtype=np.float64)
    v_g_states = np.zeros((n_balls, x.shape[0]), dtype=np.float64)
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
        v_g_states[i] = np.array([1.0, 1.0, -L*np.sin(phi)/n_balls, 0.0, 0.0, 0.0])
    
    return v_pts, v_g_states, v_radius


"""
https://matplotlib.org/stable/gallery/mplot3d/pathpatch3d.html
"""
def text3d(ax, xyz, s, zdir="z", size=None, angle=0, usetex=False, **kwargs):
    """
    Plots the string *s* on the axes *ax*, with position *xyz*, size *size*,
    and rotation angle *angle*. *zdir* gives the axis which is to be treated as
    the third dimension. *usetex* is a boolean indicating whether the string
    should be run through a LaTeX subprocess or not.  Any additional keyword
    arguments are forwarded to `.transform_path`.

    Note: zdir affects the interpretation of xyz.
    """
    x, y, z = xyz
    if zdir == "y":
        xy1, z1 = (x, z), y
    elif zdir == "x":
        xy1, z1 = (y, z), x
    else:
        xy1, z1 = (x, y), z

    text_path = TextPath((0, 0), s, size=size, usetex=usetex)
    trans = Affine2D().rotate(angle).translate(xy1[0], xy1[1])

    p1 = PathPatch(trans.transform_path(text_path), **kwargs)
    ax.add_patch(p1)
    art3d.pathpatch_2d_to_3d(p1, z=z1, zdir=zdir)


def draw_planarquad_trj_3d(x_trj, tf, L, H, n_balls, fig, ax, step = 100, draw_ball=False, save_fig=True):
    nt, nx = x_trj.shape
    ts = np.linspace(0.0, tf, nt)
    for i in range(0, nt, step):
        
        x = x_trj[i]
        t_i = ts[i]
        if i == 0:
            rect, l_arrow, r_arrow, v_circles = compute_pqud_patches(x, L, H, n_balls, 'r')
        else:
            rect, l_arrow, r_arrow, v_circles = compute_pqud_patches(x, L, H, n_balls)
        ax.add_patch(rect)
        ax.add_patch(l_arrow)
        ax.add_patch(r_arrow)
        
        art3d.pathpatch_2d_to_3d(rect, z=t_i, zdir="x")
        art3d.pathpatch_2d_to_3d(l_arrow, z=t_i, zdir="x")
        art3d.pathpatch_2d_to_3d(r_arrow, z=t_i, zdir="x")
        
    # =========== plot the goal poses ===========     
    # ---- goal ----
    xT = np.zeros(6, dtype=np.float64) # default goal is the origin
    rect, l_arrow, r_arrow, _ = compute_pqud_patches(xT, L, H, n_balls, 'green')
    ax.add_patch(rect)
    ax.add_patch(l_arrow)
    ax.add_patch(r_arrow)
    
    art3d.pathpatch_2d_to_3d(rect, z=tf+0.01, zdir="x")
    art3d.pathpatch_2d_to_3d(l_arrow, z=tf+0.01, zdir="x")
    art3d.pathpatch_2d_to_3d(r_arrow, z=tf+0.01, zdir="x")

    ax.set_xlabel(r'$Time (s)$', fontsize=10, rotation=150)
    ax.set_ylabel(r'$p_x$', fontsize=10, rotation=150)
    ax.set_zlabel(r'$p_z$', fontsize=10, rotation=60)

    ax.set_xlim(0, tf)
    # ax.set_ylim(-5, 5)
    # ax.set_zlim(-1, 12)
    # ax.view_init(elev=30, azim=30, roll=15)
    
    plt.tight_layout()
    
    plt.show()
    
    if save_fig:
        file_path = os.path.abspath(__file__)
        tools_dir = os.path.dirname(file_path)
        python_dir = os.path.abspath(os.path.join(tools_dir, '..'))
        exp_dir = os.path.abspath(os.path.join(python_dir, 'experiments'))
        fig.savefig(exp_dir+"/figures/planar_quad/landing_trajectory.pdf", dpi=3000)
        
    return fig

def draw_pquad_trj_2d(x_trj, L, H, n_balls, fig, ax, step, draw_ball, save_fig=True, file_name="pquad_trj2d.pdf"):
    nt, nx = x_trj.shape
    for i in range(0, nt, step):
        x = x_trj[i]
        fig, ax = draw_quad_balls(x, L, H, n_balls, fig, ax, draw_ball, patch_color='b')
    
    if save_fig:
        fig.savefig(file_name, dpi=1000, bbox_inches='tight')
    return fig, ax


def draw_map_track(planarmap, track_data, origin_x=0.0, origin_y=0.0, fig=None, ax=None):
    if ax is None:
        fig, ax = plt.subplots()
    
    fig, ax = planarmap.draw_map(fig, ax, plot=False)
    ax = plot_track(track_data, origin_x, origin_y, color='b', ax=ax)
    
    # plt.show()
    
    return fig, ax


def draw_pquad_trj_cov_2d(x_trj, cov_matrix, L, H, n_balls, fig, ax, step, draw_ball, save_fig=True, file_name="pquad_trj2d.pdf"):
    nt, nx = x_trj.shape
    for i in range(0, nt, step):
        x = x_trj[i]
        cov = cov_matrix[i].reshape(nx, nx)
        fig, ax = draw_quad_balls_cov(x, cov, i, L, H, n_balls, fig, ax, draw_ball, patch_color='b')
    
    if save_fig:
        fig.savefig(file_name, dpi=1000, bbox_inches='tight')
    return fig, ax
    

def draw_pquad_map_trj_2d(x0, xT, x_trj, L, H, n_balls, fig, ax, planarmap,
                          step, draw_ball, save_fig=True, file_name="pquad_trj2d.pdf"):

    fig, ax = planarmap.draw_map(fig, ax, plot=False)
        
    fig, ax = draw_pquad_trj_2d(x_trj, L, H, n_balls, fig, ax, step, draw_ball, save_fig)
    
    fig, ax = draw_quad_balls(x0, L, H, n_balls, fig, ax, draw_ball=False, patch_color='r')
    fig, ax = draw_quad_balls(xT, L, H, n_balls, fig, ax, draw_ball=False, patch_color='g')
    
    if save_fig:
        fig.savefig(file_name, dpi=1000, bbox_inches='tight')
        
    return fig, ax

if __name__ == '__main__':
    L = 5.0
    H = 0.35
    n_balls = 5
    
    state = np.array([15.0, 20.0, np.pi/6, 0.0, 0.0, 0.0])
    
    # fig, ax = plt.subplots()
    # fig, ax = draw_quad_balls(state, L, H, n_balls, fig, ax)
    # plt.show()

    file_path = os.path.abspath(__file__)
    tools_dir = os.path.dirname(file_path)
    python_dir = os.path.abspath(os.path.join(tools_dir, '..'))
    data_dir = os.path.abspath(os.path.join(os.path.join(python_dir, 'experiments'), 'data'))

    sys.path.append(data_dir)
    
    # read data
    from numpy import genfromtxt
    example_trj = genfromtxt(data_dir+'/p_quad_sample0.csv', delimiter=',')
    
    ## plot trajecoty in a 3d axis plot, including an axis for the time.
    # fig = plt.figure()
    # ax = fig.add_subplot(projection='3d')
    # tf = 1.0
    # n_balls = 5
    # fig = draw_planarquad_trj_3d(example_trj, tf, L, H, n_balls, fig, ax, step = 1000, draw_ball=False)

    # fig.show()
    
    
    # plot trajecotry in a 2d axis plot, only plot the x-z trajectory
    sdf_2d, planarmap = generate_2dsdf("SingleObstacleMap", savemap=False)
    # fig_2d, ax_2d = plt.subplots()
    # fig_2d, ax_2d = planarmap.draw_map(fig_2d, ax_2d, plot=False)
    
    # fig_2d, ax_2d = draw_pquad_trj_2d(example_trj, L, H, n_balls, fig_2d, ax_2d, step = 2000, draw_ball=False, save_fig=True)

    fig1, ax1 = plt.subplots()
    
    fig1, ax1 = draw_pquad_map_trj_2d(example_trj[0], example_trj[-1], example_trj, 
                                      L, H, n_balls, fig1, ax1, step = 2000, draw_ball=False, planarmap=planarmap)
    
    track_filenme = 'experiments/data/tracks/curve3.csv' 
    fig1, ax1 = draw_map_track(planarmap, track_filenme, fig=fig1, ax=ax1)
    plt.show()
    