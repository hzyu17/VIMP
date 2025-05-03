import torch
import numpy as np
import open3d as o3d
import os, sys
# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))
if vimp_dir not in sys.path:            # avoid duplicates
    sys.path.insert(0, vimp_dir)

from python.sdf_robot import OccpuancyGrid, PointCloud, save_occmap_for_matlab
import json

# -----------------------------------------
#   Read from camera configuration file 
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