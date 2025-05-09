import csv
import yaml
import os, sys
import numpy as np
import torch
import matplotlib.pyplot as plt
from conditional_sampling import conditional_sample
from torch.distributions.multivariate_normal import MultivariateNormal

# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))
build_dir = os.path.dirname(vimp_dir) + "/build/vimp"
third_party_dir = vimp_dir + "/3rdparty"
python_tools_dir = vimp_dir + "/python"
data_dir = this_dir + "/../../../matlab_helpers/GVIMP-examples/2d_pR/sparse_gh/map2/case2"

if vimp_dir not in sys.path:            
    sys.path.insert(0, vimp_dir)
if build_dir not in sys.path:            
    sys.path.insert(0, build_dir)
if third_party_dir not in sys.path:            
    sys.path.insert(0, third_party_dir)
if python_tools_dir not in sys.path:
    sys.path.insert(0, python_tools_dir)


from sdf_robot.example.draw_planar_quad_sdf import plot_cov_ellipse
# from bind_SDF import PlanarSDF
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
    col = int(round((x - origin_x) / cell_size))
    row = int(round((y - origin_y) / cell_size))
    return row, col

def dim_to_cells(w, h):
    """Convert world-size (w, h) to integer cell counts."""
    h_c = int(round(h / cell_size))
    w_c = int(round(w / cell_size))
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
    grid[r1:r2, c1:c2] = 1

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
sdf_data = np.loadtxt(sdf_dir, delimiter=",", dtype=np.float64)
sdf = libplanar_sdf.PlanarSDF(np.array([origin_x, origin_y]), cell_size, sdf_data)

# ------------------------ Collision Checking -------------------------
mean_file = data_dir+"/zk_sdf.csv"
cov_file = data_dir+"/Sk_sdf.csv"
joint_precision_file = data_dir+"/joint_precision.csv"

means = np.loadtxt(mean_file, delimiter=",", dtype=np.float32).T
covs = np.loadtxt(cov_file, delimiter=",", dtype=np.float64)
joint_prec = np.loadtxt(joint_precision_file, delimiter=",", dtype=np.float64)

n_states, dim_state = means.shape
dim_conf = dim_state // 2

joint_mean = means.flatten()
joint_prec = joint_prec[:,-1].reshape(n_states * dim_state, n_states * dim_state)
joint_cov = np.linalg.inv(joint_prec)

means = means[:,:dim_conf]
covs = covs.reshape(dim_state, dim_state, n_states)
covs = covs[:dim_conf, :dim_conf, :]

signed_dist = np.zeros(n_states, dtype=np.float32)
for i in range(n_states):
    x = means[i]
    signed_dist[i] = sdf.getSignedDistance(x)

threshold = sp['threshold']
max_iter = sp['max_iter']




# ------------------------ Sampling -------------------------

# [cond_mean, cond_conv] = conditional_sample(mean, joint_cov, state_dim=4)

# # Compute eigenvalues of cond_conv (which is symmetric)
# eigen_vals = torch.linalg.eigvalsh(cond_conv)
# print("Min eigenvalue:", eigen_vals[0].item())
# print("Max eigenvalue:", eigen_vals[-1].item())

# cov = (cond_conv + cond_conv.T) / 2

# sample = MultivariateNormal(cond_mean, covariance_matrix=cov).sample()
# samples = sample.reshape(48, 4)



df_marginal_cov = np.loadtxt(cov_file, delimiter=",", dtype=np.float32)
df_marginal_mean = np.loadtxt(mean_file, delimiter=",", dtype=np.float32)

mean = df_marginal_mean.T
cov = df_marginal_cov.T

# ax.plot(samples[:, 0], samples[:, 1], 'o', markersize=2, label='Conditional Samples')
ax.plot(means[:, 0], means[:, 1], 'o', markersize=2, label='Mean')

nt, nx = mean.shape
for i in range(0, nt):
    x_i = mean[i]
    cov_i = cov[i].reshape(nx, nx)
    plot_cov_ellipse(cov_i, x_i, 0, conf=0.997, scale=1, ax=ax, style='r-.', clipping_radius=np.inf)

plt.show()




# def plot_cov(Sigma, mu, k=3, ax=None):
#     from matplotlib.patches import Ellipse
#     eigvals, eigvecs = np.linalg.eigh(Sigma)   # eigh → guaranteed sorted λ1≤λ2
#     order = eigvals.argsort()[::-1]            # make λ1 ≥ λ2 for nicer handling
#     eigvals, eigvecs = eigvals[order], eigvecs[:,order]

#     a, b = k*np.sqrt(eigvals)                  # semi-axis lengths
#     theta = np.degrees(np.arctan2(*eigvecs[:,0][::-1]))  # principal angle in deg

#     if ax is None:
#         fig, ax = plt.subplots()
#     ax.scatter(*mu, c='red', s=20, label='mean')
#     ax.add_patch(Ellipse(mu, 2*a, 2*b, theta, fc='none', ec='blue', lw=2, label='3σ contour'))
#     ax.set_aspect('equal')
    
#     plt.show()