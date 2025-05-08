import os, sys
import yaml
import torch
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from pathlib import Path

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
    num_states = joint_means.shape[0]
    
    covariance_matrix = np.loadtxt(cov_path, delimiter=',')
    covariance_matrix = covariance_matrix.reshape(14, 14, num_states)
    covariance_matrix = covariance_matrix[:7, :7, :]

    # ----------------- Collision Checking ----------------
    n_states = joint_means.shape[0]
    costs = np.zeros(n_states)

    for i in range(n_states):
        pts = fk.compute_sphere_centers(joint_means[i]).T
        signed_distances = np.array([sdf.getSignedDistance(point) for point in pts])
        # Compute difference between the signed distance and the sum of epsilon and radius
        margins = signed_distances - radii
        costs[i] = np.min(margins)
    costs = np.array(costs)
    # print(f"Collision costs: {costs}")

    means = joint_means
    covs = covariance_matrix

    # ----------------- Resampling ------------------------
    it = 0

    # Resample until all intermediate states are collision-free
    # or until max iterations is reached
    while True:
        # Only consider the intermediate states (exclude the first and last points).
        indices = np.arange(costs.shape[0])
        bad_idx = indices[(costs < threshold) & (indices != 0) & (indices != costs.shape[0] - 1)]
        if bad_idx.size == 0:
            print("All intermediate states are collision-free, ending resampling.")
            break
        if it >= max_iters:
            print(f"Reached maximum iterations {max_iters}, and some states still have collisions: {bad_idx}")
            break
        it += 1
        # print(f"Iteration {it}: {bad_idx.size} states need resampling -> indices {bad_idx}")

        for i in bad_idx:
            # Use the original mean from joint_means for sampling
            mean_i = joint_means[i]
            cov_i  = covs[:, :, i]
            # Ensure the covariance is positive-definite by adding a small jitter.
            cov_i += 1e-6 * np.eye(mean_i.size)

            eig_vals = np.linalg.eigvals(cov_i)
            max_eig = np.max(eig_vals)
            print(f"Largest eigenvalue for state {i}: {max_eig}")

            new_theta = np.random.multivariate_normal(mean_i, cov_i)

            # Update the mean for this state.
            means[i] = new_theta

            # Recompute the collision cost for the new sample.
            poses = fk.compute_sphere_centers(new_theta)
            raw_dists = np.array([sdf.getSignedDistance(pt) for pt in poses.T])
            # If SignedDistanceField returns voxel units, convert them to meters:
            # dists_m = raw_dists * voxel_grid.voxel_size
            dists_m = raw_dists  # assume it's already in meters
            margins = dists_m - radii
            costs[i] = np.min(margins)

        # print(f"  End of iteration {it}, remaining collision costs: {costs[bad_idx]}")

    # Save the valid mean trajectory to a csv file.
    output_file = os.path.join(result_dir, "good_zk.csv")
    np.savetxt(output_file, means.T, delimiter=",")
    print(f"Saved good mean to {output_file}")


if __name__ == "__main__":

    experiment_config = this_dir + "/config.yaml"
    path_to_yaml = Path(experiment_config)

    with path_to_yaml.open("r") as f:
        cfg = yaml.safe_load(f)            # cfg is now a nested dict

    planning_cfg_path   = Path(cfg["Planning"]["config_file"])
    max_iters           = cfg["Sampling"]["max_iters"]
    threshold          = cfg["Sampling"]["threshold"]

    # Load the signed distance field
    sdf = SignedDistanceField()
    sdf.loadSDF(os.path.join(this_dir, "../../maps/WAM/FrankaBoxDataset_cereal.bin"))

    # Perform collision checking and resampling
    collision_checking_and_resampling(planning_cfg_path, sdf, max_iters, threshold)

