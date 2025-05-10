import yaml
import os, sys
import numpy as np
import torch
import matplotlib.pyplot as plt
from resampling import find_blocks
from conditional_sampling import conditional_sample
from torch.distributions.multivariate_normal import MultivariateNormal

# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))

from vimp.python.sdf_robot.example.draw_planar_quad_sdf import plot_cov_ellipse
from vimp.thirdparty.sensor3D_tools.scripts.SignedDistanceField2D import generate_field2D
import libplanar_sdf

# ------------------------ Load map parameters -------------------------
with open(this_dir + '/PointRobotConfig.yaml') as f:
    cfg = yaml.safe_load(f)

fld       = cfg['Field']
sp        = cfg['Sampling']

cols      = fld['cols']
rows      = fld['rows']
origin_x  = fld['origin']['x']
origin_y  = fld['origin']['y']
cell_size = fld['cell_size']
obstacles = fld['obstacles']
sdf_dir   = vimp_dir + fld['sdf_dir']

# ------------------------ Map generation -------------------------
def world_to_cell(x, y):
    """Return (row, col) index for a world (x,y)."""
    col = int(np.floor((x - origin_x) / cell_size))
    row = int(np.floor((y - origin_y) / cell_size))
    return row, col

def dim_to_cells(w, h):
    """Convert world-size (w, h) to integer cell counts."""
    h_c = int(np.floor(h / cell_size))
    w_c = int(np.floor(w / cell_size))
    return w_c, h_c

fig, ax = plt.subplots(1, 1, figsize=(8, 8))

# Allocate empty map (0 = free, 1 = obstacle)
grid = np.zeros((rows, cols), dtype=int)

for obs in obstacles:
    cx, cy = obs['center']
    w, h   = obs['size']

    r0, c0 = world_to_cell(cx, cy)
    w_c, h_c = dim_to_cells(w, h)
    # compute half‐extents
    dr = (h_c - 1) // 2
    dc = (w_c - 1) // 2
    # slice bounds (clamp to grid)
    r1 = max(0, r0 - dr)
    r2 = min(rows, r0 + dr + (h_c % 2))
    c1 = max(0, c0 - dc)
    c2 = min(cols, c0 + dc + (w_c % 2))
    grid[r1-1:r2, c1-1:c2] = 1   #Compatile with Matlab's 1-based indexing

extent = [
origin_x,
origin_x + cols * cell_size,
origin_y,
origin_y + rows * cell_size,
]

ax.imshow(grid,
          origin='lower',
          cmap='gray_r',       # obstacles in black, free in white
          interpolation='nearest',
          extent=extent)

# ------------------------ Generate PlanarSDF -------------------------
field = generate_field2D(grid, cell_size)
sdf = libplanar_sdf.PlanarSDF(np.array([origin_x, origin_y]), cell_size, field)

# ------------------------ Collision Checking -------------------------
mean_file = data_dir+"/zk_sdf.csv"
cov_file = data_dir+"/Sk_sdf.csv"
joint_precision_file = data_dir+"/joint_precision.csv"

means = np.loadtxt(mean_file, delimiter=",", dtype=np.float32).T
covs = np.loadtxt(cov_file, delimiter=",", dtype=np.float64)
joint_prec = np.loadtxt(joint_precision_file, delimiter=",", dtype=np.float64)

n_states, dim_state = means.shape
dim_conf = dim_state // 2

joint_means = means.flatten()
joint_prec = joint_prec[:,-1].reshape(n_states * dim_state, n_states * dim_state)
joint_cov = np.linalg.inv(joint_prec)

means = means[:,:dim_conf]
means_original = means.copy()
covs = covs.reshape(dim_state, dim_state, n_states)
covs = covs[:dim_conf, :dim_conf, :]

signed_dist = np.zeros(n_states, dtype=np.float32)
for i in range(n_states):
    x = means[i]
    signed_dist[i] = sdf.getSignedDistance(x)

# ------------------------ Sampling -------------------------
threshold = sp['threshold']
max_iters = sp['max_iters']
window_size = sp['window_size']

indices = np.arange(n_states)
collide_idx = indices[(signed_dist < threshold) & (indices != 0) & (indices != n_states - 1)]

if collide_idx.size == 0:
    print("No collision detected.")
else:
    blocks = find_blocks(collide_idx)
    print("Collision detected at indices:", collide_idx)
    print("Collision blocks:", blocks)

for block in blocks:
    start_state = max(block[0] - window_size, 0)
    end_state = min(block[-1] + window_size, n_states - 1)
    start_vector_idx = start_state * dim_state
    end_vector_idx = (end_state + 1) * dim_state

    block_means = joint_means[start_vector_idx:end_vector_idx]
    block_cov = joint_cov[start_vector_idx:end_vector_idx, start_vector_idx:end_vector_idx]

    block_cov += np.eye(block_cov.shape[0]) * 1e-7
    [cond_mean, cond_cov] = conditional_sample(block_means, block_cov, dim_state)

    # # Plot the conditional mean and covariance
    # nt = block.size + 2 * window_size - 2
    # cond_mean = cond_mean.reshape(-1, dim_state)[:, :dim_conf]
    # cond_cov = cond_cov.reshape(nt*dim_state, nt*dim_state)

    # for i in range(window_size, window_size + block.size):
    #     x_i = cond_mean[i]
    #     cov_i = cond_cov[i*dim_state:i*dim_state+dim_state, i*dim_state:i*dim_state+dim_state]
    #     plot_cov_ellipse(cov_i, x_i, 0, conf=0.997, scale=1, ax=ax, style='b-', clipping_radius=np.inf)

    block_iter = 0
    while block_iter < max_iters:
        block_iter += 1
        if block_iter % 50 == 0:
            print("Iteration:", block_iter)
        new_theta = np.random.multivariate_normal(cond_mean, cond_cov)
        means_new = new_theta.reshape(-1, dim_state)[:, :dim_conf]

        signed_dist = np.zeros(means_new.shape[0], dtype=np.float32)
        for i in range(means_new.shape[0]):
            x = means_new[i]
            signed_dist[i] = sdf.getSignedDistance(x)

        # Check if any samples are in collision
        collide_idx = np.where(signed_dist < threshold)[0]
        if collide_idx.size == 0:
            print("No collision detected in block after", block_iter, "iterations.")
            means[start_state + 1:end_state] = means_new
            break


# ---------------------- Plotting -------------------------
df_marginal_cov = np.loadtxt(cov_file, delimiter=",", dtype=np.float32)
df_marginal_mean = np.loadtxt(mean_file, delimiter=",", dtype=np.float32)

mean = df_marginal_mean.T
cov = df_marginal_cov.T

ax.plot(means_original[:, 0], means_original[:, 1], 's', markersize=4, label='Original Mean')
ax.plot(means[:, 0], means[:, 1], 'o', markersize=4, label='Sampled Mean')

plt.legend()

nt, nx = mean.shape
for i in range(0, nt):
    x_i = mean[i]
    cov_i = cov[i].reshape(nx, nx)
    plot_cov_ellipse(cov_i, x_i, 0, conf=0.997, scale=1, ax=ax, style='r-.', linewidth=1, clipping_radius=np.inf)

plt.show()



# def plot_cov(Sigma, mu, k=3, ax=None):
#     from matplotlib.patches import Ellipse
#     eigvals, eigvecs = np.linalg.eigh(Sigma)   
#     order = eigvals.argsort()[::-1]            
#     eigvals, eigvecs = eigvals[order], eigvecs[:,order]

#     a, b = k*np.sqrt(eigvals)                  
#     theta = np.degrees(np.arctan2(*eigvecs[:,0][::-1]))  

#     if ax is None:
#         fig, ax = plt.subplots()
#     ax.scatter(*mu, c='red', s=20, label='mean')
#     ax.add_patch(Ellipse(mu, 2*a, 2*b, theta, fc='none', ec='blue', lw=2, label='3σ contour'))
#     ax.set_aspect('equal')
    
#     plt.show()