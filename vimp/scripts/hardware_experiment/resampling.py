import os, sys
import yaml
import torch
import time
from pathlib import Path
import numpy as np
from pathlib import Path
from conditional_sampling import conditional_sample


# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))

from vimp.pybinds.bind_FK import ForwardKinematics, DHType
from vimp.thirdparty.sensor3D_tools import SignedDistanceField

def find_blocks(bad_idx, gap = 2):
    """Split a sorted array of collision indices into contiguous runs."""
    # where the gap between consecutive indices is >1
    gaps = np.where(np.diff(bad_idx) > gap)[0]
    # split at those gap positions (+1 for the right-side start)
    blocks = np.split(bad_idx, gaps + 1)
    return blocks

def collision_checking(sdf: SignedDistanceField,
                        fk: ForwardKinematics,
                        means: np.ndarray):
    n_states = means.shape[0]
    n_spheres = fk.num_spheres

    pts = fk.compute_sphere_centers_batched(means)
    signed_distances = sdf.getSignedDistanceBatched(pts)
    margins_matrix = signed_distances.reshape(n_spheres, n_states, order='F') - fk.radii[:, None]
    min_margin = np.min(margins_matrix, axis=0)

    return min_margin


def read_forwardkinematic_from_config(config_file):
    # ---------------- Forward Kinematics and Parameters----------------
    with config_file.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    dh_type = getattr(DHType, cfg["dh_type"])

    dh = cfg["dh_params"]
    a          = np.array(dh["a"],          dtype=np.float64)
    alpha      = np.array(dh["alpha"],      dtype=np.float64)
    d          = np.array(dh["d"],          dtype=np.float64)
    theta_bias = np.array(dh["theta_bias"], dtype=np.float64)
    radii      = np.array(dh["radii"],      dtype=np.float64)
    frames     = np.array(dh["frames"],     dtype=np.int32)
    centers    = np.array(dh["centers"],    dtype=np.float64)  # shape (20,3)
    
    # Create FK object
    return ForwardKinematics(a, alpha, d, theta_bias, frames, centers, radii, dh_type)


def collision_checking_and_resampling(config_file: str,
                              result_dir: str,
                              sdf: SignedDistanceField,
                              sampling_cfg: dict,
                              output: bool = False):
    
    fk = read_forwardkinematic_from_config(config_file)
    
    max_iters = sampling_cfg["max_iters"]
    collision_threshold = sampling_cfg["collision_threshold"]
    safety_threshold = sampling_cfg["safety_threshold"]
    window_size = sampling_cfg["window_size"]

    # Load the mean and covariance matrices
    mean_path = os.path.join(result_dir, "zk_sdf.csv")
    prec_path = os.path.join(result_dir, "joint_precision.csv")

    # ---------------- Load VIMP Results ----------------
    zk_matrix = np.loadtxt(mean_path, delimiter=',').T
    joint_prec = np.loadtxt(prec_path, delimiter=',')

    n_states, dim_state = zk_matrix.shape
    dim_conf = dim_state // 2
    
    joint_means = zk_matrix.flatten()
    joint_cov = torch.inverse(torch.from_numpy(joint_prec)).numpy()

    means = zk_matrix[:, :dim_conf]
    means_sample = means.copy()

    # ----------------- Collision Checking ----------------
    min_margin = collision_checking(sdf, fk, means)
    print(f"Minimum margin: {min(min_margin)}")
    # print(f"Index of the minimum margin: {np.argmin(min_margin)}")
    original_signed_distance = min(min_margin)

    # ----------------- Sampling --------------------------
    # Only consider the intermediate states (exclude the first and last points).
    indices = np.arange(n_states)
    bad_idx = indices[(min_margin < collision_threshold) & (indices != 0) & (indices != n_states - 1)]

    if bad_idx.size == 0:
        print("All intermediate states are collision-free, ending resampling.")
        blocks = []
    else:
        blocks = find_blocks(bad_idx)
        # print("Collision detected at indices:", bad_idx)
        print("Collision blocks:", blocks)

    for block in blocks:
        # Resample the block of states
        start_state = max(block[0] - window_size, 0)
        end_state = min(block[-1] + window_size, n_states - 1)
        start_vector_idx = start_state * dim_state
        end_vector_idx = (end_state + 1) * dim_state
        
        block_means = joint_means[start_vector_idx:end_vector_idx]
        block_covs = joint_cov[start_vector_idx:end_vector_idx, start_vector_idx:end_vector_idx]

        # Ensure the covariance is positive-definite by adding a small jitter.
        block_covs += 1e-7 * np.eye(block_covs.shape[0])

        # Sample from the multivariate normal distribution
        [cond_mean, cond_cov] = conditional_sample(block_means, block_covs, dim_state)
        L = np.linalg.cholesky(cond_cov)  # Cholesky decomposition to speed up sampling

        signed_dist_block = min_margin[start_state + 1:end_state]
        safety_metric = np.min(signed_dist_block)
        safe_block_means = joint_means[start_vector_idx + dim_state: end_vector_idx - dim_state] # The means of the block without condition states

        block_iter = 0

        while block_iter < max_iters:
            # Check if the new sample is valid
            block_iter += 1
            if block_iter % 10000 == 0:
                print(f"Block is being resampled, iteration {block_iter}.")

            z = np.random.standard_normal(cond_mean.shape)
            new_theta = cond_mean + L @ z

            # Compute the signed distances for the new sample
            means_new = new_theta.reshape(-1, dim_state)[:, :dim_conf]
            min_block_margin = collision_checking(sdf, fk, means_new)

            if np.all(min_block_margin > safety_threshold):
                print(f"Block {block} is valid after {block_iter} iterations.")
                means_sample[start_state + 1:end_state] = means_new
                safe_block_means = new_theta
                break
            elif np.min(min_block_margin) > safety_metric:
                safety_metric = np.min(min_block_margin)
                means_sample[start_state + 1:end_state] = means_new
                safe_block_means = new_theta

        # Update trajectory with best means found
        joint_means[start_vector_idx + dim_state: end_vector_idx - dim_state] = safe_block_means

    if output:
        output_file = os.path.join(result_dir, "good_zk.csv")
        np.savetxt(output_file, means_sample.T, delimiter=",")
        print(f"Saved good mean to {output_file}")

    result_distance = collision_checking(sdf, fk, means_sample)
    print(f"Minimum margin after resampling: {min(result_distance)}")
    # print(f"Index of the minimum margin after resampling: {np.argmin(result_distance)}")
    return original_signed_distance, min(result_distance)


if __name__ == "__main__":

    experiment_config = this_dir + "/config/config.yaml"
    path_to_yaml = Path(experiment_config)
    start = time.time()

    with path_to_yaml.open("r") as f:
        cfg = yaml.safe_load(f)            # cfg is now a nested dict

    planning_cfg_path   = Path(cfg["Planning"]["config_file"])
    sdf_path            = cfg["Planning"]["disturbed_sdf"]
    result_dir = Path(vimp_dir + "/" + cfg["Planning"]["saving_prefix"])

    # Load the signed distance field
    sdf = SignedDistanceField()
    sdf.loadSDF(sdf_path)

    collision_checking_and_resampling(planning_cfg_path, result_dir, sdf, cfg["Sampling"], output=True)

    end = time.time()
    print(f"Time taken: {end - start:.2f} seconds")