import torch
import numpy as np
import open3d as o3d
import os, sys

# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))
build_dir = os.path.dirname(vimp_dir) + "/build/vimp"

print("build_dir:", build_dir)
if vimp_dir not in sys.path:            
    sys.path.insert(0, vimp_dir)
if build_dir not in sys.path:            
    sys.path.insert(0, build_dir)

from python.sdf_robot import OccpuancyGrid, PointCloud, save_occmap_for_matlab
import json
from pathlib import Path


import pybind_expparams.core as pybind_params
import pybind_WamSDF 
from pybind_WamSDF import GVIMPWAMArm

config_file = Path(vimp_dir + "/configs/vimp/sparse_gh/franka.yaml")
import yaml, numpy as np
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
                
gvimp_params = pybind_params.GVIMPParams(nx, nu, total_time, 
                                         n_states, coeff_Qc, GH_deg, 
                                         sig_obs, eps_sdf, radius, 
                                         step_size, max_iterations,
                                         init_precision_factor, 
                                         boundary_penalties, temperature,
                                         high_temperature, low_temp_iterations, 
                                         stop_err, max_n_backtracking, 
                                         map_name, sdf_file, alpha)


# Generate the optimizer
verbose = True
wam_optimizer = GVIMPWAMArm(gvimp_params)
runtime = wam_optimizer.run_optimization_withtime(gvimp_params, verbose)

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
print("voxels:", voxels)

# -------------------------------
#  From voxels to occupancy grid
# -------------------------------
rows, cols, z = 100, 100, 100
cell_size = 0.05
occup_map = OccpuancyGrid(rows, cols, z, cell_size)
occup_map.from_voxel_grid(voxel_grid)
occup_map.set_origin(center[0], center[1], center[2])

print("occupancy map shape:", occup_map.map.shape)
save_occmap_for_matlab(occup_map, this_dir+"/occupancy_map.mat")

