#!/usr/bin/env python

import rospy
import csv
import moveit_commander
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def read_trajectory_from_csv(file_path):
    # Read the trajectory from a CSV file
    trajectory = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            trajectory.append([float(val) for val in row[0:7]])
    return trajectory

if __name__ == '__main__':
    # moveit_commander.roscpp_initialize()
    # rospy.init_node('wam_motion_planner')
    rospy.init_node('display_trajectory_node')
    
    # Connect to the MoveIt! API
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("panda_arm")

    # Read the trajectory from a CSV file
    trajectory = read_trajectory_from_csv('zk_sdf.csv')

    # Create a RobotTrajectory message
    robot_trajectory = RobotTrajectory()
    joint_trajectory = JointTrajectory()
    # joint_trajectory.joint_names = ['wam/base_yaw_joint', 
    #                                 'wam/shoulder_pitch_joint', 
    #                                 'wam/shoulder_yaw_joint', 
    #                                 'wam/elbow_pitch_joint', 
    #                                 'wam/wrist_yaw_joint', 
    #                                 'wam/wrist_pitch_joint',
    #                                 'wam/palm_yaw_joint']
    joint_trajectory.joint_names = ['panda_link1', 
                                    'panda_link2', 
                                    'panda_link3', 
                                    'panda_link4', 
                                    'panda_link5', 
                                    'panda_link6',
                                    'panda_link7']
    for point in trajectory:
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = point
        joint_trajectory.points.append(joint_trajectory_point)
    robot_trajectory.joint_trajectory = joint_trajectory

    # Create a DisplayTrajectory message and publish it
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory.append(robot_trajectory)
    print("============ Publishing trajectory ============")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)
    display_trajectory_publisher.publish(display_trajectory)

    rospy.spin()
