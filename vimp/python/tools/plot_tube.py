## --- Tool for plot covariance time series ---
# Hongzhe Yu
# 12/16/2023

import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from numba import njit, prange, float64, int64

# # @njit
# def plot_cov_tube(trj_mean, trj_cov, tf, color='blue', save_html=False):
#     nt, nx = trj_mean.shape

#     t = np.linspace(0, tf, nt)
    
#     # Generate points to plot the ellipsoid
#     n_ellip_pts = 20
        
#     step = 50
#     samples_t = np.arange(0, nt, step)
#     n_samples_t = len(samples_t)
    
#     tellips = t_ellips_pts(samples_t, trj_mean, trj_cov, n_ellip_pts)

#     X = np.zeros((n_samples_t, 1, n_ellip_pts), dtype=np.float64)
#     Y = np.zeros_like(X, dtype=np.float64)
#     Z = np.zeros_like(X, dtype=np.float64)

#     for i in range(n_samples_t):
#         X[i] = t[samples_t[i]] * np.ones(n_ellip_pts)
#         Y[i] = tellips[i, 0, :]
#         Z[i] = tellips[i, 1, :]

#     # Plot covariance trajectory
#     fig = go.Figure(data=[go.Surface(z=X, x=Y, y=Z, opacity=0.5)])

#     fig.update_layout(title='Covariance Steering for a Linear System', autosize=False,
#                       width=500, height=500,
#                       margin=dict(l=65, r=50, b=65, t=90))

#     fig.update_layout(scene=dict(
#                       xaxis_title='X',
#                       yaxis_title='Y',
#                       zaxis_title='Time'
#                     ))
        
#     # Show image
#     fig.show()
    

    # # save as interactive html file
    # if save_html:
    #     fig.write_html("figures/simple_linCS.html")
        
    # return True

@njit(float64[:,:,:](int64[:], float64[:,:], float64[:,:,:], int64))
def t_ellips_pts(samples_t, trj_mean, trj_cov, n_ellip_pts):
    nx = trj_mean.shape[1]
    n_samples_t = len(samples_t)    
    tellips = np.zeros((n_samples_t, nx, n_ellip_pts), dtype=np.float64)
    
    # Generate points to plot the ellipsoid
    theta = np.linspace(0, 2 * np.pi, n_ellip_pts)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    circle_para = np.zeros((2, n_ellip_pts), dtype=np.float64)
    circle_para[0] = cos_theta
    circle_para[1] = sin_theta
    
    for i in range(n_samples_t):
        t_i = samples_t[i]
        cov_i = trj_cov[t_i]
        eval_Sigi, evec_Sigi = np.linalg.eigh(cov_i)
        sqrtSigi = evec_Sigi @ np.diag(np.sqrt(eval_Sigi)) @ evec_Sigi.T
        
        tellips[i] = 3 * sqrtSigi @ circle_para + trj_mean[t_i][:, np.newaxis]
        
    return tellips


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

font = {'family': 'serif',
        'color':  'black',
        'weight': 'normal',
        'size': 16,
        }

def plot_cov_tube(trj_mean, trj_cov, tf, color='blue', step = 50, fig=None, ax=None):
    nt, nx = trj_mean.shape
    t = np.linspace(0, tf, nt)

    # Generate points to plot the ellipsoid
    n_ellip_pts = 20
    samples_t = np.arange(0, nt, step)
    n_samples_t = len(samples_t)

    # Assuming t_ellips_pts is a function you've defined elsewhere that calculates the ellipsoid points
    tellips = t_ellips_pts(samples_t, trj_mean, trj_cov, n_ellip_pts)

    X = np.zeros((n_samples_t, n_ellip_pts), dtype=np.float64)
    Y = np.zeros_like(X, dtype=np.float64)
    Z = np.zeros_like(X, dtype=np.float64)

    for i in range(n_samples_t):
        X[i, :] = tellips[i, 0, :]
        Y[i, :] = tellips[i, 1, :]
        Z[i, :] = t[samples_t[i]] * np.ones(n_ellip_pts)

    # # Plot covariance trajectory
    if fig is None and ax is None:
        fig, ax = plt.subplots()

    # Plotting each set of ellipsoid points
    for i in range(n_samples_t):
        ax.plot(X[i], Y[i], color=color, linewidth=0.5, alpha=0.5)
    ax.plot(X[-1], Y[-1], color=color, linewidth=0.5, label=r'$3-\sigma$ covariance')
    return fig, ax


def plot_cov_tube_3D(trj_mean, trj_cov, tf, color='blue', step = 50, fig=None, ax=None):
    nt, nx = trj_mean.shape
    t = np.linspace(0, tf, nt)

    # Generate points to plot the ellipsoid
    n_ellip_pts = 20
    samples_t = np.arange(0, nt, step)
    n_samples_t = len(samples_t)

    # Assuming t_ellips_pts is a function you've defined elsewhere that calculates the ellipsoid points
    tellips = t_ellips_pts(samples_t, trj_mean, trj_cov, n_ellip_pts)

    X = np.zeros((n_samples_t, n_ellip_pts), dtype=np.float64)
    Y = np.zeros_like(X, dtype=np.float64)
    Z = np.zeros_like(X, dtype=np.float64)

    for i in range(n_samples_t):
        X[i, :] = tellips[i, 0, :]
        Y[i, :] = tellips[i, 1, :]
        Z[i, :] = t[samples_t[i]] * np.ones(n_ellip_pts)

    # # Plot covariance trajectory
    if fig is None and ax is None:
        fig, ax = plt.subplots()
        # ax = fig.add_subplot(111, projection='3d')

    # Plotting each set of ellipsoid points
    for i in range(n_samples_t):
        ax.plot(Z[i], X[i], Y[i], color=color, linewidth=0.5, alpha=0.5)
    ax.plot(Z[-1], X[-1], Y[-1], color=color, linewidth=0.5, label=r'$3-\sigma$ covariance')
    return fig, ax

def plot_trj_2d(trj_mean, tf):
    nt, nx = trj_mean.shape
    
    # Set figure parameters
    width, height = 12, 4
    plt.figure(figsize=(width, height))
    plt.subplots_adjust(wspace=0.1)
    
    # Generate time vector
    ts = np.linspace(0, tf, nt)

    # --------------- plot x1 ----------------
    plt.subplot(1, 3, 1)
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.scatter(ts, trj_mean[:, 0], color='blue', s=20, marker='.')
    plt.xlabel('t', fontsize=14)
    plt.ylabel('x_1', fontsize=14)

    # --------------- plot x2 ----------------
    plt.subplot(1, 3, 2)
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.scatter(ts, trj_mean[:, 1], color='blue', s=20, marker='.')
    plt.xlabel('t', fontsize=14)
    plt.ylabel('x_2', fontsize=14)