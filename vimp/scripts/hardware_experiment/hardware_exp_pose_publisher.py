from pathlib import Path
from apriltag_d435 import read_poses_from_yaml
from fake_bodyframe_publisher import PosePublisher
import numpy as np
from geometry_msgs.msg import Pose
from bodyframe_pose_listener import matrix_to_pose


box_pose_file = Path(__file__).parent / "Data" / "box_poses_hardware.yaml"
# pose_mats = read_poses_from_yaml(box_pose_file)

pose_mats = np.array([[0.9977, -0.0267, 0.0616, 0.0037], [0.0666, 0.5159, -0.8540, 0.025], [-0.009, 0.8562, 0.5165, 0.4501], [0, 0, 0, 1]])
print(pose_mats)

pose_publisher = PosePublisher('/B1_pose', wait_key=True, output_file='Data/Worst_poses.yaml')
pose_publisher.run_with_given_poses([pose_mats])