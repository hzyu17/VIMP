from pathlib import Path
from fake_bodyframe_publisher import PosePublisher
import numpy as np
from geometry_msgs.msg import Pose
from pose_helpers import matrix_to_pose, read_poses_from_yaml


box_pose_file = Path(__file__).parent / "Data" / "box_poses_disturbed.yaml"
pose_mats = read_poses_from_yaml(box_pose_file)
# print(pose_mats)

pose_publisher = PosePublisher('/B1_pose', wait_key=True, output_file='Data/Worst_poses.yaml')
pose_publisher.run_with_given_poses(pose_mats)