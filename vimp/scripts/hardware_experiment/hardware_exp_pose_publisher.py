from pathlib import Path
from fake_bodyframe_publisher import PosePublisher
from pose_helpers import read_poses_from_yaml, read_poses_from_yaml_single_obstacle


# box_pose_file = Path(__file__).parent / "Data" / "box_poses_disturbed.yaml"
box_pose_file = Path(__file__).parent / "Data" / "box_poses_d435_disturb.yaml"
cfg_path = Path(__file__).parent / "config" / "config.yaml"

pose_dict, timestamps = read_poses_from_yaml(box_pose_file)

pose_publisher = PosePublisher('/pose_publish', wait_key=True)
pose_publisher.run_with_given_poses(pose_dict, timestamps)