import torch
from types import SimpleNamespace
import open3d as o3d
import numpy as np

import os

this_file = os.path.abspath(__file__)
this_dir  = os.path.dirname(this_file)


class OccpuancyGrid:
    def __init__(self, rows, cols, z, cell_size):
        self.rows = rows
        self.cols = cols
        self.z = z
        self.cell_size = cell_size
        self.map = torch.zeros((rows, cols, z), dtype=torch.uint8)
        self.corner_idx = torch.empty((0, 6), dtype=torch.long)
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0
        self.cell_size = cell_size     
        
    
    def set_origin(self, origin_x, origin_y, origin_z):
        """
        Set the origin of the occupancy grid in the world frame.
        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_z = origin_z

    
    def add_obstacle(self, center, size):
        """
        Insert a rectangular obstacle into an occupancy grid and
        append its bounding box to the `corner` list.

        Parameters
        ----------
        center : (3,) Tensor-like [row, col, depth] – grid-cell centre
        size     : (3,) Tensor-like [height, width, length] – side lengths (odd)
        occ_map  : (H, W, D) torch.Tensor – occupancy grid (modified in place)
        corner   : (N, 6) torch.Tensor – existing list of bounding boxes

        Returns
        -------
        occ_map  : updated occupancy grid (same object, for convenience)
        corner   : updated (N+1, 6) tensor with bounding box [r0, r1, c0, c1, z0, z1]
        """

        # Ensure integer tensors
        pos  = torch.as_tensor(center, dtype=torch.long)
        size = torch.as_tensor(size, dtype=torch.long)

        # Half-sizes 
        half = (size - 1) // 2

        # Inclusive index bounds in each dimension
        r0, r1 = (pos[0] - half[0]).item(), (pos[0] + half[0]).item()
        c0, c1 = (pos[1] - half[1]).item(), (pos[1] + half[1]).item()
        z0, z1 = (pos[2] - half[2]).item(), (pos[2] + half[2]).item()

        # Mark the cells as occupied (slice upper bound is exclusive in Python ⇒ +1)
        self.map[r0:r1 + 1, c0:c1 + 1, z0:z1 + 1] = 1

        # Append the axis-aligned bounding box to the running list
        new_box  = torch.tensor([r0, r1, c0, c1, z0, z1],
                                dtype=self.corner_idx.dtype, 
                                device=self.corner_idx.device).unsqueeze(0)
        self.corner_idx   = torch.cat((self.corner_idx, new_box), dim=0)
        
        
    def from_voxel_grid(self, voxel_grid):
        idx_xyz = np.asarray([v.grid_index for v in voxel_grid.get_voxels()], dtype=np.int64)
        
        self.map[idx_xyz[:, 0], idx_xyz[:, 1], idx_xyz[:, 2]] = 1
        
        print("occupancy map:")
        print(self.map)


def save_occmap_for_matlab(occup_map, filename):
    import scipy.io as sio
    class SimpleNamespace: pass
    dataset = SimpleNamespace()
    dataset.cols      = occup_map.cols
    dataset.rows      = occup_map.rows
    dataset.z         = occup_map.z
    dataset.origin_x  = occup_map.origin_x
    dataset.origin_y  = occup_map.origin_y
    dataset.origin_z  = occup_map.origin_z
    dataset.cell_size = occup_map.cell_size
    dataset.map = occup_map.map.detach().cpu().numpy().astype(np.float64)

    # --- pack into a dict, then save ------------------------------------
    mat_dict = {
        'cols'      : dataset.cols,
        'rows'      : dataset.rows,
        'z'         : dataset.z,
        'origin_x'  : dataset.origin_x,
        'origin_y'  : dataset.origin_y,
        'origin_z'  : dataset.origin_z,
        'cell_size' : dataset.cell_size,
        'map'       : dataset.map,
    }

    sio.savemat(filename, {'dataset': mat_dict}, do_compression=True)


class PointCloud:
    def __init__(self, occup_map=None):
        self.occup_map = occup_map 
        
        self.base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3) 
        
        self.camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1) 
        
        # Default camera pose in the robot base frame
        self.T_BC = torch.eye(4, dtype=torch.float32)
        # Default obstacle pose in the camera frame
        self.T_CO = torch.eye(4, dtype=torch.float32)
        # Default obstacle pose in the robot base frame
        self.T_BO = self.T_BC @ self.T_CO
        
        
    def register_camera_pose(self, T_BC):
        self.T_BC = T_BC
        self.T_BO = self.T_BC @ self.T_CO
        self.camera_frame.transform(T_BC)
        
        self.pcd.transform(self.T_BO)
    
    
    def update_obstacle_pose(self, T_CO):
        """
        Update the obstacle pose of the occupancy grid in the camera frame.
        """
        self.T_CO = T_CO
        self.T_BO = self.T_BC @ self.T_CO
        
        self.pcd.transform(self.T_BO)
        
    
    def read_from_file(self, filename):
        self.pcd = o3d.io.read_point_cloud(filename)
        
    
    def transform_frame(self, T):
        T[:3, 3] = [0.0, 0.0, 0.0]
        self.camera_frame = self.frame.transform(T.copy())   
    
    
    def construct_pcd(self):
        coords = torch.nonzero(self.occup_map.map).float()
        coords_metric = coords * self.occup_map.cell_size
        coords_metric[:,0] += self.occup_map.origin_x
        coords_metric[:,1] += self.occup_map.origin_y
        coords_metric[:,2] += self.occup_map.origin_z

        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(coords_metric.numpy())

    
    def draw_pcd_frame(self):
        self.pcd.paint_uniform_color([0.3, 0.4, 0.5])  
        self.pcd = self.pcd.voxel_down_sample(0.05)

        o3d.visualization.draw_geometries([self.pcd, 
                                           self.camera_frame, 
                                           self.base_frame],
                                            window_name="PCD + Camera Frame + Base Frame",
                                            width=800,
                                            height=600,
                                            point_show_normal=False)
        
        
    def to_occmap(self, center, size_xyz, cell_size):
        """Converting a point cloud into an occupancy map."""
        dims = (size_xyz / cell_size).long()   
        
        self.occ_map = OccpuancyGrid(*dims, cell_size)           
        # occ_map  = torch.zeros(dims.tolist(), dtype=torch.uint8)            

        # --- point cloud  -----------------------------------------------------
        pts = torch.from_numpy(np.asarray(self.pcd.points))    # (N,3) float32

        # --- world → grid -----------------------------------------------------
        grid_pts = ((pts - center) / cell_size).floor().long()  # (N,3) integer indices
        mask = ((grid_pts >= 0) & (grid_pts < dims)).all(dim=1)
        grid_pts = grid_pts[mask]                         # keep inside bounds
        
        self.occ_map.map[grid_pts] = 1
        
    def to_voxel_grid(self, voxel_size=0.01):
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(self.pcd, voxel_size=voxel_size)
        return voxel_grid
    

# ==================
#   Example Usage
# ==================
# --- Create an occupancy map with one obstacle ----------------------
rows, cols, z = 100, 100, 100
cell_size = 0.05
occup_map = OccpuancyGrid(rows, cols, z, cell_size)   

# --- Add a single rectangular obstacle -------------------------------
center = (50, 50, 50)
size   = (50, 100, 50)

occup_map.add_obstacle(center, size) 

# ----------------------
#   Quick sanity check
# ----------------------
print("Number of occupied voxels:", int(occup_map.map.sum()))
print("First obstacle AABB:", occup_map.corner_idx[0].tolist())

pcd = PointCloud()
# pcd.construct_pcd()


# ----------------
#  Read from file
# ----------------
pcd.read_from_file(this_dir+"/UTF-800000001.ply")

# center_camera = torch.tensor([0.0, 0.0, 0.0])
# size_camera   = torch.tensor([rows, cols, z])
# cell_size = 0.05
# pcd.to_occmap(center_camera, size_camera, cell_size)


# Example of a camera pose
R = np.array([[ 0, -1,  0],
              [ 1,  0,  0],
              [ 0,  0,  1]])
t = np.array([0.1, 0.1, 0.5])

# build the 4×4 matrix
cam2world = np.eye(4)
cam2world[:3, :3] = R
cam2world[:3,  3] = t

pcd.register_camera_pose(torch.tensor(cam2world, dtype=torch.float32))

R_obj = np.array([[ 0, -1,  0],
                [ 0,  0,  1],
                [ 1,  0,  0]])
t_obj = np.array([0, 0, 0])

T_CO = np.eye(4)
T_CO[:3, :3] = R_obj
T_CO[:3,  3] = t_obj

pcd.update_obstacle_pose(torch.tensor(T_CO, dtype=torch.float32))
pcd.draw_pcd_frame()

# ----------- Convert to voxel and visualize -----------
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

save_occmap_for_matlab(occup_map, this_dir+"/occupancy_map.mat")

# ------------------------------------------------
#   Transform the point cloud under a given pose
# ------------------------------------------------

