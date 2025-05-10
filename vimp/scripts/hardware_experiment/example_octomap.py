#!/usr/bin/env python
import rospy, numpy as np
import open3d as o3d
from geometry_msgs.msg   import Pose
from octomap_msgs.msg    import Octomap, OctomapWithPose
from moveit_msgs.msg     import PlanningScene
from moveit_msgs.srv     import ApplyPlanningScene, ApplyPlanningSceneRequest

import os, sys
import rospy, time
    
from vimp.thirdparty.sensor3D_tools import transform_sdf, o3d_voxelgrid_to_ros, rosvoxel_to_octomap


def make_small_voxelgrid(voxel_size=0.005):
    # create a tiny pointcloud of a 3×3×3 cube
    coords = np.linspace(2.0, 2.0+voxel_size*2, 30)
    pts = np.array([[x,y,z] for x in coords for y in coords for z in coords])
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    # voxelize it
    return o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)


def publish_small_octomap():
    rospy.init_node("tiny_octomap_test")

    # 1) build a tiny VoxelGrid
    vg = make_small_voxelgrid(voxel_size=0.05)

    # 2) convert to ROS VoxelGrid
    vg_msg = o3d_voxelgrid_to_ros(vg, frame_id="world")

    # 3) convert that to an Octomap
    octo = rosvoxel_to_octomap(vg_msg)   # gives you an octomap_msgs/Octomap

    # 5) build a PlanningScene diff
    ps = PlanningScene()
    ps.is_diff       = True
    ps.world.octomap = octo

    # 6) apply via service
    rospy.wait_for_service("/apply_planning_scene")
    svc = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)
    req = ApplyPlanningSceneRequest(scene=ps)
    res = svc(req)
    if not res.success:
        rospy.logerr("Failed to apply tiny octomap: %d", res.error_code.val)
    else:
        rospy.loginfo("Tiny octomap applied successfully")

    rospy.spin()



if __name__=="__main__":
    publish_small_octomap()