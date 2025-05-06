import torch
import numpy as np
import open3d as o3d
import os, sys
# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))
build_dir = os.path.dirname(vimp_dir) + "/build/vimp"
third_party_dir = vimp_dir + "/3rdparty"

result_dir = os.path.join(this_dir, "../../../matlab_helpers/GVIMP-examples/Franka/sparse_gh/case1")

print("build_dir:", build_dir)
if vimp_dir not in sys.path:            
    sys.path.insert(0, vimp_dir)
if build_dir not in sys.path:            
    sys.path.insert(0, build_dir)
if third_party_dir not in sys.path:            
    sys.path.insert(0, third_party_dir)
    
# from matlabengine_readsave_sdf import read_and_save_sdf
# from python.sdf_robot import save_occmap_for_matlab

from bind_FK import ForwardKinematics, DHType
from sensor3D_tools import OccpuancyGrid, PointCloud, SignedDistanceField, generate_field3D
import json
from pathlib import Path

# -----------------------------------------
#    Read from camera configuration file 
# -----------------------------------------
transform_camera = False

if transform_camera:
    d435_json_file = this_dir + '/UTF-8realsense_high_d435_no_table_tuned_p2048_w_icp.json'
    with open(d435_json_file, "r", encoding="utf-8") as f:
        data = json.load(f)        
    camera_pose = torch.tensor(data['pose'], dtype=torch.float32)
else:
    R_cmr = np.array([[ 1,  0,  0],
                      [ 0,  1,  0],
                      [ 0,  0,  1]])
    t_cmr = np.array([0, 0, 0])

    camera_pose = np.eye(4)
    camera_pose[:3, :3] = R_cmr
    camera_pose[:3,  3] = t_cmr

pcd = PointCloud()

# ----------------
#  Read from file
# ----------------
pcd.read_from_file(this_dir+"/UTF-800000001.ply")
pcd.register_camera_pose(torch.tensor(camera_pose, dtype=torch.float32))

R_obj = np.array([[ 1, 0,  0],
                    [ 0,  1,  0],
                    [ 0,  0,  1]])
t_obj = np.array([0.56, 0, -0.912])

T_CO = np.eye(4)
T_CO[:3, :3] = R_obj
T_CO[:3,  3] = t_obj

pcd.update_obstacle_pose(torch.tensor(T_CO, dtype=torch.float32))
pcd.draw_pcd_frame()

# ----------- Convert to voxel and visualize -----------
voxel_grid = pcd.to_voxel_grid(voxel_size=0.05)
o3d.visualization.draw_geometries([voxel_grid, pcd.camera_frame, pcd.base_frame])
center = voxel_grid.get_center()
min_corner = voxel_grid.get_min_bound()
max_corner = voxel_grid.get_max_bound()

voxels = voxel_grid.get_voxels()

print("Voxel grid center:", center)
print("min corner:", min_corner)
print("max corner:", max_corner)
# print("voxels:", voxels)

# ---------------- Forward Kinematics ----------------
dh_type = DHType.Modified

a           = np.array([0.0, 0.0, 0.0,  0.0825, -0.0825, 0.0,   0.088, 0.0  ], dtype=np.float64)
alpha       = np.array([0,    -np.pi/2,  np.pi/2,  np.pi/2, -np.pi/2, np.pi/2, np.pi/2, 0.0], dtype=np.float64)
d           = np.array([0.333, 0.0,      0.316,    0.0,      0.384,    0.0,     0.0,     0.107], dtype=np.float64)
theta_bias  = np.array([0,     0,        0,        0,        0,        0,       0,      -np.pi/4], dtype=np.float64)
radii       = np.array([
    0.1,  0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 
    0.05, 0.03, 0.05, 0.05, 0.05, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03
], dtype=np.float64)
frames      = np.array([
    0, 0, 1, 1, 2, 2, 2, 3, 3, 4,
    4, 4, 4, 6, 7, 7, 7, 7, 7, 7
], dtype=np.int32)
centers     = np.array([
    [ 0.0,  0.0,  -0.3],
    [ 0.0,  0.0,  -0.1],
    [ 0.0,  0.0,  -0.07],
    [ 0.0,  0.0,   0.07],
    [ 0.0,  0.0,  -0.10],
    [ 0.0,  0.0,  -0.175],
    [ 0.0,  0.0,  -0.25],
    [ 0.0,  0.0,  -0.07],
    [ 0.0,  0.0,   0.07],
    [ 0.0,  0.0,  -0.30],
    [ 0.0,  0.08, -0.15],
    [ 0.0,  0.10,  0.00],
    [ 0.0,  0.00,  0.00],
    [ 0.0,  0.0,  -0.05],
    [ 0.0,  0.0,   0.00],
    [ 0.0,  0.06,  0.00],
    [ 0.0, -0.06,  0.00],
    [ 0.0,  0.0,   0.04],
    [ 0.0,  0.07,  0.04],
    [ 0.0, -0.07,  0.04]
], dtype=np.float64)

fk = ForwardKinematics(a, alpha, d, theta_bias, frames, centers.T, dh_type)

# start_confs = np.array([-0.351, -0.521, 2.436, -1.402, 0.315, 1.783, -1.824], dtype=np.float64)
# poses = fk.compute_sphere_centers(start_confs)

# -------------------------------
#  From voxels to occupancy grid
# -------------------------------
rows, cols, z = 100, 100, 100
cell_size = 0.05
occup_map = OccpuancyGrid(rows, cols, z, cell_size)
occup_map.from_voxel_grid(voxel_grid)
field3D = generate_field3D(occup_map.map.detach().numpy(), cell_size=voxel_grid.voxel_size)

# Construct SDF class
sdf = SignedDistanceField(voxel_grid.origin, voxel_grid.voxel_size, 
                          field3D.shape[0], field3D.shape[1], field3D.shape[2])
for z in range(field3D.shape[2]):
        sdf.initFieldData(z, field3D[:,:,z])

sdf = SignedDistanceField()
sdf.loadSDF(os.path.join(this_dir, "../../maps/WAM/FrankaDeskDataset_cereal.bin"))

# # An example signed distance 
# test_pt = np.array([0.5, 0.5, 0.5])
# sdf_val = sdf.getSignedDistance(test_pt)
# print("Signed distance value at point {}: {}".format(test_pt, sdf_val))


# # An example signed distance for forward kinematics
# distances = np.array([sdf.getSignedDistance(point) for point in poses.T])
# print("Signed distance values at poses:\n", distances)



# =========================================================================
#  Alternatively: We can use files to save, read, and construct the SDF.
# =========================================================================
# print("occupancy map shape:", occup_map.map.shape)
# save_occmap_for_matlab(occup_map, this_dir+"/occupancy_map.mat")

# # -----------------------------------------------------------------
# # Read the occupancy map, convert to SDF and save to binary file
# read_and_save_sdf(this_dir+"/occupancy_map.mat", this_dir+"/sdf.bin")

# ------------------------------------------------------
# Read the SDF into the C++ SignedDistanceField class 
# from bind_SDF import SignedDistanceField
# sdf = SignedDistanceField()
# sdf.loadSDF(this_dir+"/sdf.bin")

# ==============================================
# python bindings for GVIMP Planner C++ classes
from bind_Params.core import GVIMPParams
# from bind_WamSDF import GVIMPWAMArm # Does not work yet, figuring out why

config_file = Path(vimp_dir + "/configs/vimp/sparse_gh/franka.yaml")
import yaml
with config_file.open("r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

total_time = cfg["total_time"]
n_states = cfg["n_states"]
map_name = str(cfg["map_name"])
coeff_Qc = cfg["coeff_Qc"]
GH_deg = cfg["GH_deg"]
sig_obs = cfg["sig_obs"]
eps_sdf = cfg["eps_sdf"]
radius = cfg["radius"]
step_size = cfg["step_size"]
init_precision_factor = cfg["init_precision_factor"]
boundary_penalties = cfg["boundary_penalties"]
temperature = cfg["temperature"]
high_temperature = cfg["high_temperature"]
low_temp_iterations = cfg["low_temp_iterations"]
stop_err = float(cfg["stop_err"])
max_iterations = cfg["max_iterations"]
max_n_backtracking = cfg["max_n_backtracking"]
sdf_file = str(cfg["sdf_file"])
saving_prefix = str(cfg["saving_prefix"])

start_pos = np.array(cfg["start_pos"], dtype=float)
goal_pos  = np.array(cfg["goal_pos"],  dtype=float)

print("map_name :", repr(map_name))
print("sdf_file :", repr(sdf_file), "exists:", Path(sdf_file).is_file())
print("start:", start_pos)
print("goal :", goal_pos)

nx = 7
nu = 7

map_name = map_name.strip()
sdf_file = sdf_file.strip()
alpha = 1

gvimp_params = GVIMPParams(nx, nu, total_time, 
                            n_states, coeff_Qc, GH_deg, 
                            sig_obs, eps_sdf, radius, 
                            step_size, max_iterations,
                            init_precision_factor, 
                            boundary_penalties, temperature,
                            high_temperature, low_temp_iterations, 
                            stop_err, max_n_backtracking, 
                            map_name, sdf_file, alpha)


# # Generate the optimizer
# verbose = True
# wam_optimizer = GVIMPWAMArm(gvimp_params)
# runtime = wam_optimizer.run_optimization_withtime(gvimp_params, verbose)


mean_file = os.path.join(result_dir, "zk_sdf.csv")
zk_matrix = np.loadtxt(mean_file, delimiter=',')
zk_selected_transposed = zk_matrix[:7, :].T
print("zk_selected_transposed:", zk_selected_transposed.shape)


covariance_file = os.path.join(result_dir, "Sk_sdf.csv")
covariance_matrix = np.loadtxt(covariance_file, delimiter=',')

# Reshape the covariance matrix from (14*14, 30) to (14, 14, 30)
covariance_matrix = covariance_matrix.reshape(14, 14, 30)
covariance_matrix = covariance_matrix[:7, :7, :]
print("Updated covariance_matrix shape:", covariance_matrix.shape)

costs = []
epsilon = eps_sdf  # _epsilon parameter from config
sigma = sig_obs    # _sigma parameter


for i in range(zk_selected_transposed.shape[0]):
    poses = fk.compute_sphere_centers(zk_selected_transposed[i])
    signed_distances = np.array([sdf.getSignedDistance(point) for point in poses.T])
    n_balls = signed_distances.shape[0]
    distances = np.maximum(0, (epsilon + radii[:n_balls]) - signed_distances)
    identity = np.eye(distances.shape[0])
    collision_cost = distances @ (sig_obs * identity) @ distances.T
    print("Collision cost:", collision_cost)
    costs.append(collision_cost)
costs = np.array(costs)

means = zk_selected_transposed
covs = covariance_matrix


max_iters = 1000
it = 0

while True:
    # Only consider the intermediate states (exclude the first and last points).
    indices = np.arange(costs.shape[0])
    bad_idx = indices[(costs > 0) & (indices != 0) & (indices != costs.shape[0] - 1)]
    if bad_idx.size == 0:
        print("All intermediate states are collision-free, ending resampling.")
        break
    if it >= max_iters:
        print(f"Reached maximum iterations {max_iters}, and some states still have collisions: {bad_idx}")
        break
    it += 1
    print(f"Iteration {it}: {bad_idx.size} states need resampling -> indices {bad_idx}")

    for i in bad_idx:
        # Use the original mean from zk_selected_transposed for sampling
        mean_i = zk_selected_transposed[i]
        cov_i  = covs[:, :, i]
        # Ensure the covariance is positive-definite by adding a small jitter.
        cov_i += 1e-6 * np.eye(mean_i.size)
        new_theta = np.random.multivariate_normal(mean_i, cov_i)

        # Update the mean for this state.
        means[i] = new_theta

        # Recompute the collision cost for the new sample.
        poses = fk.compute_sphere_centers(new_theta)
        raw_dists = np.array([sdf.getSignedDistance(pt) for pt in poses.T])
        # If SignedDistanceField returns voxel units, convert them to meters:
        # dists_m = raw_dists * voxel_grid.voxel_size
        dists_m = raw_dists  # assume it's already in meters
        dists = np.maximum(0, (eps_sdf + radii[:dists_m.size]) - dists_m)
        # Compute the collision cost as a quadratic form.
        costs[i] = dists @ (sig_obs * dists)

    print(f"  End of iteration {it}, remaining collision costs: {costs[bad_idx]}")




# # Example of computing collision cost
# identity = np.eye(distances.shape[0])
# collision_cost = distances @ (sig_obs * identity) @ distances.T
# print("Collision cost:", collision_cost)

