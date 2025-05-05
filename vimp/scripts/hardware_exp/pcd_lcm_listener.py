import lcm
from exlcm import pcd_t
import open3d as o3d
import numpy as np
import rospy, geometry_msgs.msg
from visualization_msgs.msg import Marker

import os, sys
this_file = os.path.abspath(__file__)
this_dir  = os.path.dirname(this_file)
vimp_dir = os.path.dirname(os.path.dirname(this_dir))
build_dir = os.path.dirname(vimp_dir) + "/build/vimp"

if build_dir not in sys.path:            
    sys.path.insert(0, build_dir)
if vimp_dir not in sys.path:
    sys.path.insert(0, vimp_dir)

from python.sdf_robot import OccpuancyGrid
from python.sdf_robot import SignedDistanceField3D
from bind_SDF import SignedDistanceField

from scripts.vimp_ros.voxel_to_rviz import publish_voxel_grid

# optional RGB in range [0, 1]
rgb = np.asarray([
    [1, 0, 0],      
    [0, 1, 0],      
    [0, 0, 1],      
    [1, 1, 0],      
], dtype=np.float64)


def voxel_to_occmap(rows, cols, z, cell_size, voxel_grid):
    occmap = OccpuancyGrid(rows, cols, z, cell_size)
    occmap.from_voxel_grid(voxel_grid)
    return occmap


rospy.init_node("voxel_to_rviz")

def pcd_handler(channel, data):
    msg = pcd_t.decode(data)
    print("Received pcd message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   pose    = %s" % str(msg.pose))
    print("   num_points = %s" % str(msg.num_points))
    print("")
    
    # Construct the pcd class object from the message
    points_arr = np.asarray(msg.points, dtype=np.float64).reshape(-1, 3).copy()
    pcd = o3d.geometry.PointCloud()
    pcd.points  = o3d.utility.Vector3dVector(points_arr)   
    pcd.colors  = o3d.utility.Vector3dVector(rgb) 
    
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
    radius=0.05, max_nn=30))
    pcd.normalize_normals()
    
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.01)
    o3d.visualization.draw_geometries([voxel_grid])    
    
    occmap = voxel_to_occmap(200, 200, 200, 0.01, voxel_grid)
    field3D = SignedDistanceField3D.generate_field3D(occmap.map.detach().numpy(), cell_size=0.01)
    
    sdf = SignedDistanceField(voxel_grid.origin, voxel_grid.voxel_size, 
                              field3D.shape[0], field3D.shape[1], field3D.shape[2])
    
    for z in range(field3D.shape[2]):
        sdf.initFieldData(z, field3D[:,:,z])
    
    # An example signed distance 
    test_pt = np.array([0.5, 0.5, 0.5])
    sdf_val = sdf.getSignedDistance(test_pt)
    print("Signed distance value at point {}: {}".format(test_pt, sdf_val))
    
    print("SDF Constructed!")
    
    publish_voxel_grid(voxel_grid, topic="/obstacle_voxel", frame="world")
    rospy.spin()
    
lc = lcm.LCM()
subscription = lc.subscribe("EXAMPLE", pcd_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass
