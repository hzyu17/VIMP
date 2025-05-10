import yaml
import os, sys
import numpy as np
import matplotlib.pyplot as plt
from resampling import find_blocks
from conditional_sampling import conditional_sample

# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))
build_dir = os.path.dirname(vimp_dir) + "/build/vimp"
third_party_dir = vimp_dir + "/3rdparty"
python_tools_dir = vimp_dir + "/python"

if vimp_dir not in sys.path:            
    sys.path.insert(0, vimp_dir)
if build_dir not in sys.path:            
    sys.path.insert(0, build_dir)
if third_party_dir not in sys.path:            
    sys.path.insert(0, third_party_dir)
if python_tools_dir not in sys.path:
    sys.path.insert(0, python_tools_dir)

from sdf_robot.example.draw_planar_quad_sdf import plot_cov_ellipse
from sensor3D_tools.scripts.SignedDistanceField2D import generate_field2D
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
result_dir = vimp_dir + fld['result_dir']
center_noise = fld['obs_center_noise']
size_noise = fld['obs_size_noise']

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

    cx_noisy = cx + np.random.normal(scale=center_noise)
    cy_noisy = cy + np.random.normal(scale=center_noise)
    w_noisy  = w  + np.random.normal(scale=size_noise)
    h_noisy  = h  + np.random.normal(scale=size_noise)

    r0, c0 = world_to_cell(cx_noisy, cy_noisy)
    w_c, h_c = dim_to_cells(w_noisy, h_noisy)
    # compute half‐extents
    dr = (h_c - 1) // 2
    dc = (w_c - 1) // 2
    # slice bounds (clamp to grid)
    r1 = max(0, r0 - dr)
    r2 = min(rows, r0 + dr + (h_c % 2))
    c1 = max(0, c0 - dc)
    c2 = min(cols, c0 + dc + (w_c % 2))
    grid[r1-1:r2, c1-1:c2] = 1   #Compatile with Matlab's 1-based indexing

width, height = cols * cell_size, rows * cell_size
extent = [origin_x, origin_x + width, origin_y, origin_y + height]

ax.imshow(grid,
          origin='lower',
          cmap='gray_r',       # obstacles in black, free in white
          interpolation='nearest',
          extent=extent)

# ------------------------ Generate PlanarSDF -------------------------
field = generate_field2D(grid, cell_size)
sdf = libplanar_sdf.PlanarSDF(np.array([origin_x, origin_y]), cell_size, field)

# ------------------------ Collision Checking -------------------------
mean_file = result_dir+"/zk_sdf.csv"
cov_file = result_dir+"/Sk_sdf.csv"
joint_precision_file = result_dir+"/joint_precision.csv"

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
    blocks = []  # Initialize blocks as empty list when no collisions
else:
    blocks = find_blocks(collide_idx, gap=2)
    print("Collision detected at indices:", collide_idx)
    print("Collision blocks:", blocks)

for block in blocks:
    start_state = max(block[0] - window_size, 0)
    end_state = min(block[-1] + window_size, n_states - 1)
    start_vector_idx = start_state * dim_state
    end_vector_idx = (end_state + 1) * dim_state

    block_means = joint_means[start_vector_idx:end_vector_idx]
    block_covs = joint_cov[start_vector_idx:end_vector_idx, start_vector_idx:end_vector_idx]

    block_covs += np.eye(block_covs.shape[0]) * 1e-7
    [cond_mean, cond_cov] = conditional_sample(block_means, block_covs, dim_state)

    L = np.linalg.cholesky(cond_cov)  # Cholesky decomposition to speed up sampling

    signed_dist_block = signed_dist[start_state + 1:end_state]
    safety_metric = np.min(signed_dist_block)

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
        if block_iter % 5000 == 0:
            print("Iteration:", block_iter)
        # Fast sampling using Cholesky decomposition
        z = np.random.standard_normal(cond_mean.shape)
        new_theta = cond_mean + L @ z
        
        means_new = new_theta.reshape(-1, dim_state)[:, :dim_conf]

        signed_dist_samples = np.zeros(means_new.shape[0], dtype=np.float32)
        for i in range(means_new.shape[0]):
            x = means_new[i]
            signed_dist_samples[i] = sdf.getSignedDistance(x)

        # Check if any samples are in collision
        collide_idx = np.where(signed_dist_samples < threshold)[0]
        if collide_idx.size == 0:
            print("No collision detected in block after", block_iter, "iterations.")
            means[start_state + 1:end_state] = means_new
            break
        else:
            new_metric = np.min(signed_dist_samples)
            if new_metric > safety_metric:
                safety_metric = new_metric
                signed_dist_block = signed_dist_samples
                means[start_state + 1:end_state] = means_new

# ---------------------- Plotting -------------------------
df_marginal_cov = np.loadtxt(cov_file, delimiter=",", dtype=np.float32)
df_marginal_mean = np.loadtxt(mean_file, delimiter=",", dtype=np.float32)

mean = df_marginal_mean.T
cov = df_marginal_cov.T

ax.plot(means_original[:, 0], means_original[:, 1], 's', markersize=4, label='Original Mean')
ax.plot(means[:, 0], means[:, 1], 'o', markersize=4, label='Sampled Mean')

plt.legend()

# Plot the original covariance ellipses
# nt, nx = mean.shape
# for i in range(0, nt):
#     x_i = mean[i]
#     cov_i = cov[i].reshape(nx, nx)
#     plot_cov_ellipse(cov_i, x_i, 0, conf=0.997, scale=1, ax=ax, style='r-.', linewidth=1, clipping_radius=np.inf)

plt.show()