import yaml
import os, sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Patch
from scipy.stats import chi2
from resampling import find_blocks
from conditional_sampling import conditional_sample

# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))

from vimp.thirdparty.sensor3D_tools.scripts.SignedDistanceField2D import generate_field2D
from vimp.thirdparty.sensor3D_tools import PlanarSDF

# ------------------------ Load parameters ---------------------------
with open(this_dir + '/config/PointRobotConfig.yaml') as f:
    cfg = yaml.safe_load(f)

fld       = cfg['Field']
sp        = cfg['Sampling']
sf        = cfg['Fig_save']

cols      = fld['cols']
rows      = fld['rows']
origin_x  = fld['origin']['x']
origin_y  = fld['origin']['y']
cell_size = fld['cell_size']
obstacles = fld['obstacles']
result_dir = vimp_dir + fld['result_dir']
center_noise = fld['obs_center_noise']
size_noise = fld['obs_size_noise']

safe_threshold = sp['safe_threshold']
collision_threshold = sp['collision_threshold']
max_iters = sp['max_iters']
window_size = sp['window_size']

figure_dpi = sf['fig_dpi']
num_figs = sf['num_figs']

mean_file = result_dir+"/zk_sdf.csv"
cov_file = result_dir+"/Sk_sdf.csv"
joint_precision_file = result_dir+"/joint_precision.csv"

# -------------------- Load and process data ----------------------
means = np.loadtxt(mean_file, delimiter=",", dtype=np.float32).T
covs = np.loadtxt(cov_file, delimiter=",", dtype=np.float64)
joint_prec = np.loadtxt(joint_precision_file, delimiter=",", dtype=np.float64)

n_states, dim_state = means.shape
dim_conf = dim_state // 2

joint_means = means.flatten()
joint_prec = joint_prec[:,-1].reshape(n_states * dim_state, n_states * dim_state)
joint_cov = np.linalg.inv(joint_prec)

means = means[:,:dim_conf]
covs = covs.reshape(dim_state, dim_state, n_states)
covs = covs[:dim_conf, :dim_conf, :]

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

def plot_cov_shaded(cov, mu, ax=None, conf=0.997, facecolor='r', edgecolor='none', alpha=0.2, label=None):
    # Calculate semi-axis lengths and angle
    eigvals, eigvecs = np.linalg.eigh(cov[0:2,0:2])
    order = eigvals.argsort()[::-1]
    eigvals, eigvecs = eigvals[order], eigvecs[:,order]
    # Major and minor semi-axes
    k = np.sqrt(chi2.ppf(conf, 2))
    width, height = 2 * k * np.sqrt(eigvals)  # in matplotlib width,height = 2a,2b
    angle = np.degrees(np.arctan2(eigvecs[1,0], eigvecs[0,0]))
    
    ell = Ellipse(xy=mu[0:2],
                  width=width, height=height,
                  angle=angle,
                  facecolor=facecolor,
                  edgecolor=edgecolor,
                  alpha=alpha,
                  lw=0,
                  label=label)
    ax.add_patch(ell)
    return ax

def build_sdf(noise=True, return_sdf=True):
    # Allocate empty map (0 = free, 1 = obstacle)
    grid = np.zeros((rows, cols), dtype=int)

    for obs in obstacles:
        cx, cy = obs['center']
        w, h = obs['size']

        # Add noise
        cx_noisy = cx + (np.random.normal(scale=fld['obs_center_noise']) if noise else 0)
        cy_noisy = cy + (np.random.normal(scale=fld['obs_center_noise']) if noise else 0)
        w_noisy  = w  + (np.random.normal(scale=fld['obs_size_noise']) if noise else 0)
        h_noisy  = h  + (np.random.normal(scale=fld['obs_size_noise']) if noise else 0)

        # World coordinates to grid indices
        r0, c0 = world_to_cell(cx_noisy, cy_noisy)
        w_c, h_c = dim_to_cells(w_noisy, h_noisy)

        dr = (h_c - 1) // 2
        dc = (w_c - 1) // 2

        r1 = max(0, r0 - dr)
        r2 = min(rows, r0 + dr + (h_c % 2))
        c1 = max(0, c0 - dc)
        c2 = min(cols, c0 + dc + (w_c % 2))

        grid[r1-1:r2, c1-1:c2] = 1 # Compatible with Matlab's 1-based indexing
    
    if return_sdf:
        field = generate_field2D(grid, cell_size)
        sdf = PlanarSDF(np.array([origin_x, origin_y]), cell_size, field)
        return grid, sdf
    else:
        return grid


def sample_and_save(save_dir: str, file_name: str, show_fig = True, save_fig = True):
    # ------------------------ Collision Checking -------------------------
    grid, sdf = build_sdf(noise=True)
    resample_means = means.copy()
    resample_joint_means = joint_means.copy()
    
    signed_dist = sdf.getSignedDistanceBatched(resample_means)

    # Check if start or goal positions are in collision
    if signed_dist[0] < collision_threshold or signed_dist[n_states - 1] < collision_threshold:
        print("Start or goal position is in collision. Cannot resample.")
        return False

    # Check if any waypoints are in collision
    indices = np.arange(n_states)
    collide_idx = indices[(signed_dist < collision_threshold) & (indices != 0) & (indices != n_states - 1)]

    if collide_idx.size == 0:
        print("No collision detected. Resampling not needed.")
        return False # No collision, no need for resampling, jump to the next iteration
    else:
        blocks = find_blocks(collide_idx, gap=2)
        print("Number of blocks:", len(blocks))

    # Container to store the block distribution for plotting
    block_distribution = []

    # ------------------------ Sampling -------------------------
    Success = True  # Track overall success for all blocks
    for block in blocks:
        block_success = False
        start_state = max(block[0] - window_size, 0)
        end_state = min(block[-1] + window_size, n_states - 1)
        start_vector_idx = start_state * dim_state
        end_vector_idx = (end_state + 1) * dim_state

        block_means = resample_joint_means[start_vector_idx:end_vector_idx]
        block_covs = joint_cov[start_vector_idx:end_vector_idx, start_vector_idx:end_vector_idx]

        block_covs += np.eye(block_covs.shape[0]) * 1e-7
        [cond_mean, cond_cov] = conditional_sample(block_means, block_covs, dim_state)

        block_distribution.append({
            'cond_mean': cond_mean.copy(),
            'cond_cov': cond_cov.copy(),
        })

        L = np.linalg.cholesky(cond_cov)  # Cholesky decomposition to speed up sampling

        signed_dist_block = signed_dist[start_state + 1:end_state]
        safety_metric = np.min(signed_dist_block)
        safe_block_means = resample_joint_means[start_vector_idx + dim_state: end_vector_idx - dim_state]

        block_iter = 0
        while block_iter < max_iters:
            block_iter += 1
            if block_iter % 10000 == 0:
                print("Iteration:", block_iter)
            # Fast sampling using Cholesky decomposition
            z = np.random.standard_normal(cond_mean.shape)
            new_theta = cond_mean + L @ z
            
            # Collision checking for the new samples
            means_new = new_theta.reshape(-1, dim_state)[:, :dim_conf]
            signed_dist_samples = sdf.getSignedDistanceBatched(means_new)

            # Check if any samples are in collision
            collide_idx = np.where(signed_dist_samples < safe_threshold)[0]
            if collide_idx.size == 0:
                # print("No collision detected in block after", block_iter, "iterations.")
                resample_means[start_state + 1:end_state] = means_new
                safe_block_means = new_theta
                block_success = True
                break
            else:
                new_metric = np.min(signed_dist_samples)
                if new_metric > safety_metric:
                    safety_metric = new_metric
                    signed_dist_block = signed_dist_samples
                    resample_means[start_state + 1:end_state] = means_new
                    safe_block_means = new_theta
        
        # If this block failed, the overall success is False
        if not block_success:
            Success = False
        resample_joint_means[start_vector_idx + dim_state: end_vector_idx - dim_state] = safe_block_means

    # ------------------------ Plotting -------------------------
    if Success:
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        grid_fixed = build_sdf(noise=False, return_sdf=False)

        extent = [origin_x, origin_x + cols * cell_size, 
                  origin_y, origin_y + rows * cell_size]
        
        ax.imshow(grid_fixed, origin='lower', cmap='gray_r', # obstacles in black, free in white
                  interpolation='nearest', extent=extent, alpha=0.8)

        grid_noisy_masked = np.ma.masked_where(grid == 0, grid)
        ax.imshow(grid_noisy_masked, origin='lower', cmap='Reds',
                  interpolation='nearest', extent=extent, alpha=0.8, vmin=0, vmax=1)
        
        fixed_patch = Patch(color='lightgrey', label='Original obstacles')
        noisy_patch = Patch(color='red', alpha=0.5, label='Noisy obstacles')

        # Plot the original covariance ellipses with more subtle styling
        for i in range(0, n_states):
            x_i = means[i]
            cov_i = covs[:, :, i]
            plot_cov_shaded(cov_i, x_i, ax=ax, conf=0.997, facecolor='salmon', alpha=0.1)
        
        for j, res in enumerate(block_distribution):
            cond_mean = res['cond_mean'].reshape(-1, dim_state)[:, :dim_conf]
            nt = cond_mean.shape[0]
            cond_cov = res['cond_cov'].reshape(nt*dim_state, nt*dim_state)

            # Plot the conditional covariance ellipses
            for i in range(nt):
                x_i = cond_mean[i]
                cov_i = cond_cov[i*dim_state:i*dim_state+dim_state, i*dim_state:i*dim_state+dim_state]
                plot_cov_shaded(cov_i, x_i, ax=ax, conf=0.997, facecolor='blue', alpha=0.15)
        
        traj_cov_patch = Patch(facecolor='salmon', alpha=0.2, label='Trajectory covariance')
        resamp_cov_patch = Patch(facecolor='blue', alpha=0.25, label='Resample covariance')

        # Plot trajectory means with improved style and more descriptive labels
        original_line, = ax.plot(means[:, 0], means[:, 1], 'ro', markersize=5, markeredgecolor='firebrick',
                markeredgewidth=0.8, label='Original trajectory', alpha=0.7)
                
        resample_line, = ax.plot(resample_means[:, 0], resample_means[:, 1], 'b*', markersize=6, markeredgecolor='darkblue',
                markeredgewidth=0.8, label='Resampled trajectory', alpha=0.9)

        # Add start and end markers for clarity
        start_point, = ax.plot(means[0, 0], means[0, 1], 'go', markersize=7, markeredgecolor='darkgreen', 
                markeredgewidth=1, label='Start')
                
        goal_point, = ax.plot(means[-1, 0], means[-1, 1], 'mo', markersize=7, markeredgecolor='darkmagenta',
                markeredgewidth=1, label='Goal')

        # Connect dots with lines to better visualize the path
        for i in range(len(resample_means)-1):
            ax.arrow(resample_means[i, 0], resample_means[i, 1], 
                    resample_means[i+1, 0] - resample_means[i, 0], resample_means[i+1, 1] - resample_means[i, 1],
                    head_width=0, head_length=0, fc='b', ec='b', alpha=0.3, linewidth=1)
            
        handles = [fixed_patch, noisy_patch, traj_cov_patch, resamp_cov_patch,
                   original_line, resample_line, start_point, goal_point]
        labels = [h.get_label() for h in handles]
        ax.legend(handles, labels, loc='lower right')
        ax.set_axis_off()

        if save_fig:
            if not os.path.isdir(save_dir):
                print(f"Error: '{save_dir}' is not a valid directory.")

            full_path = os.path.join(save_dir, file_name)

            try:
                fig.savefig(full_path, dpi=300, bbox_inches='tight')
                print("Figure saved to:", full_path)

            except Exception as e:
                print("Error saving figure:", e)
                return False

        # ------------------- Save and show figure ---------------
        if show_fig:
            plt.show()
        else:
            plt.close(fig)

        return True
    
    else:
        print("Resampling failed for this block.")
        return False

if __name__ == "__main__":
    success_count = 0
    map_iter = 0
    figure_dir = this_dir + "/figures/case2"
    while success_count < 30:
        map_iter += 1
        print("Map iteration:", map_iter)
        filename = f"trajectories_{success_count}.png"

        success = sample_and_save(save_dir=figure_dir, file_name=filename, save_fig=True, show_fig=False)
        if success:
            success_count += 1
            print("Resampling successful.")