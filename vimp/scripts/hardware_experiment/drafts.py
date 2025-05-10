import torch
import numpy as np
import open3d as o3d
import os, sys
# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))

result_dir = os.path.join(this_dir, "../../../matlab_helpers/GVIMP-examples/Franka/sparse_gh/case1")
    
# from matlabengine_readsave_sdf import read_and_save_sdf
# from python.sdf_robot import save_occmap_for_matlab

from vimp.pybinds.bind_FK import ForwardKinematics, DHType
from vimp.thirdparty.sensor3D_tools import OccpuancyGrid, PointCloud, SignedDistanceField, generate_field3D
import json, yaml
from pathlib import Path
from resampling import collision_checking_and_resampling


# ------------------
#   Configurations
# ------------------
experiment_config = this_dir + "/config/config.yaml"

path_to_yaml = Path(experiment_config)

with path_to_yaml.open("r") as f:
    cfg = yaml.safe_load(f)            # cfg is now a nested dict

# Accessing values -----------------------------------------------------
field               = cfg["Field"]
origin_xyz          = field["field_origin"]      # [0, 0, 0]
field_size_xyz      = field["field_size"]        # [500, 500, 500]
voxel_size_m        = field["cell_size"]         # 0.01

obs_center_rc       = field["obstacle"]["center"]  # [200, 200]
obs_size_rcz        = field["obstacle"]["size"]    # [50, 50, 50]

planning_cfg_path   = Path(cfg["Planning"]["config_file"])

cam_cfg             = cfg["Camera"]["config_file"]
apriltag_topic      = cfg["Camera"]["apriltag_lcm_topic"]
apriltag_families   = cfg["Camera"]["apriltag_families"]


# -----------------------------------------
#    Read from camera configuration file 
# -----------------------------------------
transform_camera = False

if transform_camera:
    with open(cam_cfg, "r", encoding="utf-8") as f:
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

# ----------- Convert point cloud to voxel and visualize -----------
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
fk_config = Path(vimp_dir + "/configs/vimp/sparse_gh/franka.yaml")


# -------------------------------
#  From voxels to occupancy grid
# -------------------------------
rows, cols, z = field_size_xyz[0], field_size_xyz[1], field_size_xyz[2]

occup_map = OccpuancyGrid(rows, cols, z, voxel_size_m)
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
