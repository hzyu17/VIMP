#!/usr/bin/env python

# Python 2/3 compatibility import
from __future__ import print_function

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
import time
import sys


class CollisionSceneExample(object):
    def __init__(self):
        self._scene = PlanningSceneInterface()

        # clear the scene
        self._scene.remove_world_object()

        self.robot = RobotCommander()

        # pause to wait for rviz to load
        print("============ Waiting while RVIZ displays the scene with obstacles...")

        # TODO: need to replace this sleep by explicitly waiting for the scene to be updated.
        rospy.sleep(2)

    def add_one_box(self):
        box1_pose = [0.25, 0.25, 0.0, 0, 0, 0, 1]
        box1_dimensions = [0.25, 0.25, 0.75]

        self.add_box_object("box1", box1_dimensions, box1_pose)

        print("============ Added one obstacle to RViz!!")

    def add_bookshelf(self):
        from operator import add

        offset = [-1.5, -1.5, -0.15, 0, 0, 0, 0]
        
        box1_pose = [1.7, 2.2, 1.3, 0, 0, 0, 1]
        box1_pose = [box1_pose[i] + offset[i] for i in range(7)]
        box1_dimensions = [1.4, 0.6, 0.05]
        
        box2_pose = [1.05, 1.95, 0.9, 0, 0, 0, 1]
        box2_pose = [box2_pose[i] + offset[i] for i in range(7)]
        box2_dimensions = [0.1, 0.1, 0.8]

        box3_pose = [2.35, 1.95, 0.9, 0, 0, 0, 1]
        box3_pose = [box3_pose[i] + offset[i] for i in range(7)]
        box3_dimensions = [0.1, 0.1, 0.8]

        box4_pose = [1.05, 2.45, 0.9, 0, 0, 0, 1]
        box4_pose = [box4_pose[i] + offset[i] for i in range(7)]
        box4_dimensions = [0.1, 0.1, 0.8]
        
        box5_pose = [2.35, 2.45, 0.9, 0, 0, 0, 1]
        box5_pose = [box5_pose[i] + offset[i] for i in range(7)]
        box5_dimensions = [0.1, 0.1, 0.8]
        
        
        
        box6_pose = [2.5, 1.9, 1.45, 0, 0, 0, 1]
        box6_pose = [box6_pose[i] + offset[i] for i in range(7)]
        box6_dimensions = [0.6, 0.05, 1.9]
        
        box7_pose = [2.5, 0.9, 1.45, 0, 0, 0, 1]
        box7_pose = [box7_pose[i] + offset[i] for i in range(7)]
        box7_dimensions = [0.6, 0.05, 1.9]
        
        
        box8_pose = [2, 1.9, 1.45, 0, 0, 0, 1]
        box8_pose = [box8_pose[i] + offset[i] for i in range(7)]
        box8_dimensions = [0.4, 0.05, 1.9]
        
        # box9_pose = [1.3, 0.4, 0.95, 0, 0, 0, 1]
        # box9_pose = [box9_pose[i] + offset[i] for i in range(7)]
        # box9_dimensions = [0.6, 0.05, 1.9]
        
        
        
        box10_pose = [2.5, 1.4, 2.4, 0, 0, 0, 1]
        box10_pose = [box10_pose[i] + offset[i] for i in range(7)]
        box10_dimensions = [0.6, 1.0, 0.05]
        
        box11_pose = [2.5, 1.4, 1.9, 0, 0, 0, 1]
        box11_pose = [box11_pose[i] + offset[i] for i in range(7)]
        box11_dimensions = [0.6, 1.0, 0.05]
        
        box12_pose = [2.5, 1.4, 1.4, 0, 0, 0, 1]
        box12_pose = [box12_pose[i] + offset[i] for i in range(7)]
        box12_dimensions = [0.6, 1.0, 0.05]
        
        box13_pose = [2.5, 1.4, 0.9, 0, 0, 0, 1]
        box13_pose = [box13_pose[i] + offset[i] for i in range(7)]
        box13_dimensions = [0.6, 1.0, 0.05]

        self.add_box_object("box1", box1_dimensions, box1_pose)
        self.add_box_object("box2", box2_dimensions, box2_pose)
        self.add_box_object("box3", box3_dimensions, box3_pose)
        self.add_box_object("box4", box4_dimensions, box4_pose)
        self.add_box_object("box5", box5_dimensions, box5_pose)
        self.add_box_object("box6", box6_dimensions, box6_pose)
        self.add_box_object("box7", box7_dimensions, box7_pose)
        self.add_box_object("box8", box8_dimensions, box8_pose)
        # self.add_box_object("box9", box9_dimensions, box9_pose)
        self.add_box_object("box10", box10_dimensions, box10_pose)
        self.add_box_object("box11", box11_dimensions, box11_pose)
        self.add_box_object("box12", box12_dimensions, box12_pose)
        self.add_box_object("box13", box13_dimensions, box13_pose)
    
    def add_four_boxes(self):
        box1_pose = [0.20, 0.50, 0.25, 0, 0, 0, 1]
        box1_dimensions = [0.2, 0.2, 0.5]

        box2_pose = [-0.55, -0.55, 0, 0, 0, 0, 1]
        box2_dimensions = [0.25, 0.25, 1.75]

        box3_pose = [0.5, -0.55, 0.14, 0, 0, 0, 1]
        box3_dimensions = [0.28, 0.28, 0.22]

        box4_pose = [-0.4, 0.4, 0.5, 0, 0, 0, 1]
        box4_dimensions = [0.25, 0.25, 1.1]

        self.add_box_object("box1", box1_dimensions, box1_pose)
        self.add_box_object("box2", box2_dimensions, box2_pose)
        self.add_box_object("box3", box3_dimensions, box3_pose)
        self.add_box_object("box4", box4_dimensions, box4_pose)

        print("========== Added 4 obstacles to the scene!!")

    def add_box_object(self, name, dimensions, pose):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]
        p.pose.orientation.x = pose[3]
        p.pose.orientation.y = pose[4]
        p.pose.orientation.z = pose[5]
        p.pose.orientation.w = pose[6]

        self._scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))


if __name__ == "__main__":
    rospy.init_node("collision_scene_example_cluttered")
    while (
        not rospy.search_param("robot_description_semantic") and not rospy.is_shutdown()
    ):
        time.sleep(0.5)
    load_scene = CollisionSceneExample()

    if len(sys.argv) != 2:
        print(
            'Correct usage:: \n"rosrun moveit_tutorials collision_scene_example.py cluttered" OR \n"rosrun moveit_tutorials collision_scene_example.py sparse"'
        )
        sys.exit()
    if sys.argv[1] == "cluttered":
        load_scene.add_four_boxes()
    elif sys.argv[1] == "sparse":
        load_scene.add_one_box()
    elif sys.argv[1] == "bookshelf":
        load_scene.add_bookshelf()
    else:
        print("Please specify correct type of scene as cluttered or sparse")
        sys.exit()
