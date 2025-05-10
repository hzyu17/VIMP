import os, sys
import yaml
import torch
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from pathlib import Path
from conditional_sampling import conditional_sample

# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))
build_dir = os.path.dirname(vimp_dir) + "/build/vimp"
third_party_dir = vimp_dir + "/3rdparty"

if vimp_dir not in sys.path:            
    sys.path.insert(0, vimp_dir)
if build_dir not in sys.path:            
    sys.path.insert(0, build_dir)
if third_party_dir not in sys.path:            
    sys.path.insert(0, third_party_dir)

from bind_FK import ForwardKinematics, DHType
from sensor3D_tools import SignedDistanceField


def find_blocks(bad_idx, gap = 2):
    """Split a sorted array of collision indices into contiguous runs."""
    # where the gap between consecutive indices is >1
    gaps = np.where(np.diff(bad_idx) > gap)[0]
    # split at those gap positions (+1 for the right-side start)
    blocks = np.split(bad_idx, gaps + 1)
    return blocks

def collision_checking(sdf: SignedDistanceField,
                        fk: ForwardKinematics,
                        joint_means: np.ndarray):
    n_states, dim_conf = joint_means.shape
    n_spheres = fk.num_spheres

    margins_matrix = np.zeros((n_spheres, n_states))
    for i in range(n_states):
        pts = fk.compute_sphere_centers(joint_means[i]).T
        signed_distances = np.array([sdf.getSignedDistance(p) for p in pts])
        margins_matrix[:, i] = signed_distances
    min_margin = np.min(margins_matrix, axis=0)

    return min_margin


def collision_checking_and_resampling(config_file: str,
                                      sdf: SignedDistanceField,
                                      max_iters: int = 1000,
                                      threshold: float = 0.0):
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

    result_dir = Path(vimp_dir + "/" + cfg["saving_prefix"])

    # Load the mean and covariance matrices
    mean_path = os.path.join(result_dir, "zk_sdf.csv")
    cov_path = os.path.join(result_dir, "Sk_sdf.csv")

    # Create FK object
    fk = ForwardKinematics(a, alpha, d, theta_bias, frames, centers.T, dh_type)

    # ---------------- Load VIMP Results ----------------
    zk_matrix = np.loadtxt(mean_path, delimiter=',')
    joint_means = zk_matrix[:7, :].T

    n_states = joint_means.shape[0]
    n_spheres = frames.size

    Sk_matrix = np.loadtxt(cov_path, delimiter=',')
    Sk_matrix = Sk_matrix.reshape(14, 14, n_states)
    covariance_matrix = Sk_matrix[:7, :7, :]

    # ----------------- Collision Checking ----------------
    margins_matrix = np.zeros((n_spheres, n_states))

    for i in range(n_states):
        pts = fk.compute_sphere_centers(joint_means[i]).T
        signed_distances = np.array([sdf.getSignedDistance(point) for point in pts])
        # Compute margins as the difference between the signed distances and the radii
        margins_matrix[:, i] = signed_distances - radii
    min_margin = np.min(margins_matrix, axis=0)
    # print(f"Minimum margin for each state: {min_margin}")

    means = joint_means
    covs = covariance_matrix

    # ----------------- Resampling ------------------------
    it = 0

    # Resample until all intermediate states are collision-free
    # or until max iterations is reached
    while True:
        # Only consider the intermediate states (exclude the first and last points).
        indices = np.arange(n_states)
        bad_idx = indices[(min_margin < threshold) & (indices != 0) & (indices != n_states - 1)]
        if bad_idx.size == 0:
            print("All intermediate states are collision-free, ending resampling.")
            break
        if it >= max_iters:
            print(f"Reached maximum iterations {max_iters}, and some states still have collisions: {bad_idx}")
            break
        it += 1
        # print(f"Iteration {it}: {bad_idx.size} states need resampling -> indices {bad_idx}")

        # Individual resampling
        for i in bad_idx:
            state_margins = margins_matrix[:, i]
            first_collision = np.where(state_margins < threshold)[0][0]
            frame = frames[first_collision]
            joint_idx = max(frame - 1, 0)
            # joint_idx = 0

            # Sample from the original distribution
            mean_i = joint_means[i, joint_idx].flatten()
            cov_i  = covs[joint_idx, joint_idx, i]

            # Ensure the covariance is positive-definite by adding a small jitter.
            cov_i += 1e-6 * np.eye(mean_i.size)

            # eig_vals = np.linalg.eigvals(cov_i)
            # max_eig = np.max(eig_vals)
            # print(f"Largest eigenvalue for state {i}: {max_eig}")

            new_theta = np.random.multivariate_normal(mean_i, cov_i)
            
            print("diff norm: ", np.linalg.norm(means[i, joint_idx] - new_theta))
            
            if np.linalg.norm(means[i, joint_idx] - new_theta) < 0.01:
                # Update the mean for this state.
                means[i, joint_idx] = new_theta

                # Recompute the collision cost for the new sample.
                pts = fk.compute_sphere_centers(means[i]).T
                raw_dists = np.array([sdf.getSignedDistance(pt) for pt in pts])
                margins_matrix[:, i] = raw_dists - radii
                min_margin[i] = np.min(margins_matrix[:, i])

        # print(f"  End of iteration {it}, remaining collision costs: {costs[bad_idx]}")

    # Save the valid mean trajectory to a csv file.
    output_file = os.path.join(result_dir, "good_zk.csv")
    np.savetxt(output_file, means.T, delimiter=",")
    print(f"Saved good mean to {output_file}")


def resample_block_trajectory(config_file: str,
                                sdf: SignedDistanceField,
                                max_iters: int = 1000,
                                threshold: float = 0.1,
                                window_size: int = 2):
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

    result_dir = Path(vimp_dir + "/" + cfg["saving_prefix"])

    # Load the mean and covariance matrices
    mean_path = os.path.join(result_dir, "zk_sdf.csv")
    prec_path = os.path.join(result_dir, "joint_precision.csv")

    # Create FK object
    fk = ForwardKinematics(a, alpha, d, theta_bias, frames, centers.T, dh_type)

    # ---------------- Load VIMP Results ----------------
    zk_matrix = np.loadtxt(mean_path, delimiter=',').T
    joint_prec = np.loadtxt(prec_path, delimiter=',')

    n_states, dim_state = zk_matrix.shape
    dim_conf = dim_state // 2
    n_spheres = frames.size
    
    joint_means = zk_matrix.flatten()
    joint_cov = np.linalg.inv(joint_prec)    

    means = zk_matrix[:, :dim_conf]
    means_sample = means.copy()

    # ----------------- Collision Checking ----------------
    margins_matrix = np.zeros((n_spheres, n_states))

    for i in range(n_states):
        pts = fk.compute_sphere_centers(means[i]).T
        signed_distances = np.array([sdf.getSignedDistance(p) for p in pts])
        # Compute margins as the difference between the signed distances and the radii
        margins_matrix[:, i] = signed_distances - radii
    min_margin = np.min(margins_matrix, axis=0)
    # print(f"Minimum margin for each state: {min_margin}")


    # ----------------- Sampling --------------------------
    # Only consider the intermediate states (exclude the first and last points).
    indices = np.arange(n_states)
    bad_idx = indices[(min_margin < threshold) & (indices != 0) & (indices != n_states - 1)]

    if bad_idx.size == 0:
        print("All intermediate states are collision-free, ending resampling.")
    else:
        blocks = find_blocks(bad_idx)
        print("Collision detected at indices:", bad_idx)
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

        engenvalue = np.max(np.linalg.eigvals(cond_cov))
        print(f"Largest eigenvalue for block {block}: {engenvalue}")

        engenvalue = np.max(np.linalg.eigvals(block_covs))
        print(f"Largest eigenvalue for joint covariance: {engenvalue}")

        block_iter = 0

        while block_iter < max_iters:
            # Check if the new sample is valid
            block_iter += 1
            if block_iter % 10 == 0:
                print(f"Block is being resampled, iteration {block_iter}.")

            z = np.random.standard_normal(cond_mean.shape)
            new_theta = cond_mean + L @ z

            # Compute the signed distances for the new sample
            means_new = new_theta.reshape(-1, dim_state)[:, :dim_conf]
            block_margins = np.zeros((n_spheres, means_new.shape[0]))

            for i in range(means_new.shape[0]):
                pts = fk.compute_sphere_centers(means_new[i]).T
                signed_distances = np.array([sdf.getSignedDistance(p) for p in pts])
                block_margins[:, i] = signed_distances - radii
            min_block_margin = np.min(block_margins, axis=0)
            print(f"Minimum margin: {min_block_margin.min()}")

            # Check if the new sample is valid
            if np.all(min_block_margin > threshold):
                print(f"Block {block} is valid after {block_iter} iterations.")
                means_sample[start_state + 1:end_state] = means_new
                break

    # Save the valid mean trajectory to a csv file.
    output_file = os.path.join(result_dir, "good_zk.csv")
    np.savetxt(output_file, means_sample.T, delimiter=",")
    print(f"Saved good mean to {output_file}")


if __name__ == "__main__":

    experiment_config = this_dir + "/config/config.yaml"
    path_to_yaml = Path(experiment_config)

    with path_to_yaml.open("r") as f:
        cfg = yaml.safe_load(f)            # cfg is now a nested dict

    planning_cfg_path   = Path(cfg["Planning"]["config_file"])
    max_iters           = cfg["Sampling"]["max_iters"]
    threshold          = cfg["Sampling"]["threshold"]
    window_size        = cfg["Sampling"]["window_size"]

    # Load the signed distance field
    sdf = SignedDistanceField()
    sdf.loadSDF(os.path.join(this_dir, "../../maps/WAM/FrankaBoxDataset_cereal.bin"))

    # Perform collision checking and resampling
    # collision_checking_and_resampling(planning_cfg_path, sdf, max_iters, threshold)

    resample_block_trajectory(planning_cfg_path, sdf, max_iters, threshold, window_size)