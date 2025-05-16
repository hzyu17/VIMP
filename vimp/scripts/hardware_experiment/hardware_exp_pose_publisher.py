from pathlib import Path
from apriltag_d435 import read_poses_from_yaml
from fake_bodyframe_publisher import PosePublisher


box_pose_file = Path(__file__).parent / "Data" / "box_poses.yaml"
pose_mats = read_poses_from_yaml(box_pose_file)

pose_publisher = PosePublisher('/B1_pose', wait_key=True, output_file='Data/Exp_poses.yaml')
pose_publisher.run_with_given_poses(pose_mats)