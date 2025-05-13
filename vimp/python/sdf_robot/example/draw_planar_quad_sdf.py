import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from scipy.stats import chi2


import sys
import os

file_path = os.path.abspath(__file__)
example_dir = os.path.dirname(file_path)

py_dir = os.path.abspath(os.path.join(example_dir, '..'))
scripts_dir =os.path.abspath(os.path.join(py_dir, 'scripts'))
sys.path.append(scripts_dir)

from generate_sdf_2d import *
from matplotlib.patches import Rectangle, Arrow

# Defines the forward kinematics for collision-checking balls and their gradients to the states.  
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

def compute_pqud_patches(x, L, H, n_balls, patch_color='b'):
    center_location = x[0:2]
    theta = x[2]
    
    center1 = center_location[0]
    center2 = center_location[1]
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]], dtype=np.float64)
    X = np.array([-L/2, L/2, L/2, -L/2], dtype=np.float64)
    Y = np.array([-H/2, -H/2, H/2, H/2], dtype=np.float64)

    T = np.zeros((2, 4), dtype=np.float64)
    for i in range(4):
        T[:, i] = np.dot(R, np.array([X[i], Y[i]]))

    x_lower_left = center1 + T[0, 0]
    y_lower_left = center2 + T[1, 0]
    rect = Rectangle((x_lower_left, y_lower_left), L, H, angle=theta/3.14*180, edgecolor='none', facecolor=patch_color)
    
    arrow_width = 0.1
    # Plot 2 arrows representing the inputs
    # left arrow
    l_arrow_x = center1 - (L-arrow_width)/2*np.cos(theta)
    l_arrow_z = center2 - (L-arrow_width)/2*np.sin(theta) + H/2*np.cos(theta)
    l_arrow = Arrow(l_arrow_x, l_arrow_z, -np.sin(theta), np.cos(theta), width=0.6, edgecolor='none', facecolor='green')
    
    # right arrow
    r_arrow_x = center1 + (L-4*arrow_width)/2*np.cos(theta)
    r_arrow_z = center2 + (L-4*arrow_width)/2*np.sin(theta) + H/2*np.cos(theta)
    r_arrow = Arrow(r_arrow_x, r_arrow_z, -np.sin(theta), np.cos(theta), width=0.6, edgecolor='none', facecolor='green')
    
    radius = L / (n_balls+2.0)
    v_pts, _, v_radius = vec_balls(x, L, n_balls, radius)
    v_circles = []
    for i in range(n_balls):
        v_circles.append(plt.Circle(v_pts[i], v_radius[i], edgecolor='r', facecolor='none', linewidth=2))
        
    return rect, l_arrow, r_arrow, v_circles


def qchisq(p, df):
    return chi2.ppf(p, df)

def getpoints(C, clipping_radius=np.inf):

    n = 100 
    p = np.linspace(0, 2 * np.pi, n)
    
    eigvals, eigvecs = np.linalg.eigh(C)
    
    xy = np.column_stack([np.cos(p), np.sin(p)]) 
    
    xy_transformed = xy @ np.sqrt(np.diag(eigvals)) @ eigvecs.T
    
    x, y = xy_transformed[:, 0], xy_transformed[:, 1]
    
    r = np.sqrt(x**2 + y**2)
    x[r > clipping_radius] = np.nan
    y[r > clipping_radius] = np.nan
    
    return x, y

def plot_cov_ellipse(cov, mu, i, conf=0.997, scale=1, ax=None, style='r-.', linewidth=1, clipping_radius=np.inf):
    
    x0, y0 = mu[0:2]
    cov_2d = cov[0:2, 0:2]
    
    r = cov_2d.shape[0] 
    k = np.sqrt(qchisq(conf, r)) 
    
    if np.any(np.linalg.eigvals(cov_2d) <= 0):
        raise ValueError("The covariance matrix must be positive definite")

    x, y = getpoints(cov_2d, clipping_radius)
    
    if i == 0:
        ax.plot(scale * (x0 + k * x), scale * (y0 + k * y), style, linewidth, label=r"3$\sigma$ contour")
    else:
        ax.plot(scale * (x0 + k * x), scale * (y0 + k * y), style, linewidth)
    ax.set_aspect('equal')

    return ax


# @njit
def draw_quad_balls(x, L, H, n_balls, fig, ax, draw_ball=True, patch_color='b'):
    rect, l_arrow, r_arrow, v_circles = compute_pqud_patches(x, L, H, n_balls, patch_color)
    
    ax.add_patch(l_arrow)
    ax.add_patch(r_arrow)
    ax.add_patch(rect)
    
    if draw_ball:
        for i in range(len(v_circles)):
            ax.add_patch(v_circles[i])
            plt.axis('equal')
            
    return fig, ax


def draw_quad_balls_cov(x, cov, i, L, H, n_balls, fig, ax, draw_ball=True, patch_color='b'):

    plot_cov_ellipse(cov, x, i, conf = 0.997, scale=1, ax=ax, style='r-.', clipping_radius=np.inf)

    rect, l_arrow, r_arrow, v_circles = compute_pqud_patches(x, L, H, n_balls, patch_color)
    
    ax.add_patch(l_arrow)
    ax.add_patch(r_arrow)
    ax.add_patch(rect)
    
    if draw_ball:
        for i in range(len(v_circles)):
            ax.add_patch(v_circles[i])
            plt.axis('equal')
            
    return fig, ax
    
if __name__ == '__main__':
    print("hello")
    L = 5.0
    H = 0.35
    n_balls = 5
    state = np.array([20.0, 15.0, np.pi/6, 0.0, 0.0, 0.0])
    sdf_2d, planarmap = generate_2dsdf("SingleObstacleMap", False)

    fig, ax = plt.subplots()
    
    fig, ax = planarmap.draw_map(fig, ax, plot=False, labels=False)
    fig, ax = draw_quad_balls(state, L, H, n_balls, fig, ax, draw_ball=True, patch_color='b')
    
    font = {'family': 'serif',
        'color':  'black',
        'weight': 'normal',
        'size': 16,
        }
    
    ax.grid(True)
    ax.set_title("")
    ax.set_xlabel(r'$p_x$', fontdict=font)
    ax.set_ylabel(r'$p_z$', fontdict=font)
    ax.set_xlim([7.5, 24])
    ax.set_ylim([10, 22])
    
    fig.savefig(example_dir+"/example_quad2d.pdf", dpi=1100, bbox_inches='tight')
    
    plt.show()