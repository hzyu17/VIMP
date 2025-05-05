# Read point cloud from a .ply file, and save the point cloud as an occupancy map to a .mat file.
import torch
import open3d as o3d
import numpy as np
import os

from pointcloud import PointCloud, OccpuancyGrid, save_occmap_for_matlab


cur_file = os.path.abspath(__file__)
cur_dir  = os.path.dirname(cur_file)


def ply_to_mat(filename: str, mat_filename:str, cam2world:np.array, T_CO:np.array):
    pcd = PointCloud()

    # ----------------
    #  Read from file
    # ----------------
    pcd.read_from_file(cur_dir+'/'+filename)

    pcd.register_camera_pose(torch.tensor(cam2world, dtype=torch.float32))
    pcd.update_obstacle_pose(torch.tensor(T_CO, dtype=torch.float32))
    pcd.draw_pcd_frame()

    # ----------- Convert point cloud to voxel and visualize -----------
    voxel_grid = pcd.to_voxel_grid(voxel_size=0.03)
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

    save_occmap_for_matlab(occup_map, cur_dir + '/' + mat_filename)
    
    
# ---------- Example Usage ----------
if __name__ == '__main__':
    pcd_filename = "UTF-800000001.ply"
    occmap_savefilename = "example_occupancy_map.mat"
    
    # Example of a camera pose
    R = np.array([[ 0, -1,  0],
                [ 1,  0,  0],
                [ 0,  0,  1]])
    t = np.array([-0.5, -0.5, -0.5])

    # build the 4×4 matrix
    cam2world = np.eye(4)
    cam2world[:3, :3] = R
    cam2world[:3,  3] = t
    
    # Example of an obstacle pose in the camera frame
    R_obj = np.array([[ 0, -1,  0],
                    [ 0,  0,  1],
                    [ 1,  0,  0]])
    t_obj = np.array([-1, -1, -1])

    T_CO = np.eye(4)
    T_CO[:3, :3] = R_obj
    T_CO[:3,  3] = t_obj
    
    ply_to_mat(pcd_filename, occmap_savefilename, cam2world, T_CO)