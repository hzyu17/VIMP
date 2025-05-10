#!/usr/bin/env python

# Python 2/3 compatibility import
from __future__ import print_function

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest


import numpy as np
import open3d as o3d
import os, sys
import rospy, time

# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))
vimp_ros_dir = os.path.dirname(this_dir) + "/vimp_ros"
build_dir = os.path.dirname(vimp_dir) + "/build/vimp"
third_party_dir = vimp_dir + "/3rdparty"

if vimp_dir not in sys.path:            
    sys.path.insert(0, vimp_dir)
if build_dir not in sys.path:            
    sys.path.insert(0, build_dir)
if third_party_dir not in sys.path:            
    sys.path.insert(0, third_party_dir)
if vimp_ros_dir not in sys.path:            
    sys.path.insert(0, vimp_ros_dir)


from sensor3D_tools import transform_sdf, o3d_voxelgrid_to_ros, rosvoxel_to_octomap
from resampling import collision_checking_and_resampling
from add_bookshelf import CollisionSceneExample
from moveit_msgs.msg import PlanningScene


def publish_transformed_obstacle(sdf_json_file, pose, planning_scene_topic="/planning_scene"):
    
    vg_T, occmap_T = transform_sdf(sdf_json_file, pose, visualize=True)
    
    rospy.init_node("collision_scene_example_cluttered")
    while (
        not rospy.search_param("robot_description_semantic") and not rospy.is_shutdown()
    ):
        time.sleep(0.5)
    
    load_scene = CollisionSceneExample()
    load_scene.clear_all_objects()
    
    voxel_rosmsg = o3d_voxelgrid_to_ros(vg_T, frame_id="world")
    octomap = rosvoxel_to_octomap(voxel_rosmsg)
    
    scene_pub = rospy.Publisher(
        planning_scene_topic, PlanningScene, queue_size=1, latch=True
    )
    rospy.sleep(0.5)
    
    ps = PlanningScene()
    ps.is_diff = True
    
    # attach the OctoMap under the “world” field:
    ps.world.octomap = octomap
    
    # rospy.wait_for_service('/apply_planning_scene')
    # try:
    #     apply_ps = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
    #     req = ApplyPlanningSceneRequest(scene=ps)
    #     apply_ps(req)
    #     rospy.loginfo("Octomap inserted into planning scene")
    # except rospy.ServiceException as e:
    #     rospy.logerr("Failed to apply planning scene: %s", e)
        
    scene_pub.publish(ps)



if __name__ == "__main__":
    # Load the SDF file
    sdf_json_file = third_party_dir+"/sensor3D_tools/scripts/WAMDeskDataset.json"
    
    # Define the transformation pose
    T_example = np.eye(4)
    T_example[:3, 3] = [0.0, 0.0, 0.0]
    T_example[:3, [0,1]] = T_example[:3, [1,0]]
    
    # Publish the transformed obstacle
    planning_scene_topic = "/planning_scene"
    publish_transformed_obstacle(sdf_json_file, T_example, planning_scene_topic)
    
