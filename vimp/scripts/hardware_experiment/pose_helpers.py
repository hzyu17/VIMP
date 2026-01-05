from geometry_msgs.msg import Pose
from pathlib import Path
import yaml
import numpy as np
import tf

def pose_to_matrix(p: Pose):
    """geometry_msgs/Pose ➜ 4x4 homogeneous matrix."""
    q = p.orientation
    t = p.position
    # quaternion (x,y,z,w)  ->  3×3 rotation matrix
    R = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [t.x, t.y, t.z]
    return T


def matrix_to_pose(T: np.ndarray):
    """4x4 homogeneous matrix ➜ geometry_msgs/Pose."""
    T = np.asarray(T, dtype=float)
    t = T[:3,  3]
    q = tf.transformations.quaternion_from_matrix(T)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = t
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def pose_to_dict(pose: Pose):
    """Convert a geometry_msgs/Pose to a dictionary."""
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        }
    }


def dict_to_pose(d: dict):
    """Convert a dictionary to a geometry_msgs/Pose."""
    pose = Pose()
    pose.position.x = d["position"]["x"]
    pose.position.y = d["position"]["y"]
    pose.position.z = d["position"]["z"]
    pose.orientation.x = d["orientation"]["x"]
    pose.orientation.y = d["orientation"]["y"]
    pose.orientation.z = d["orientation"]["z"]
    pose.orientation.w = d["orientation"]["w"]
    return pose


def read_poses_from_yaml(yaml_path):
    frame_poses = {}
    timestamps = []
    with open(yaml_path, 'r') as f:
        # safe_load_all will iterate over each document separated by '---'
        for doc in yaml.safe_load_all(f):
            if doc is None:
                continue
            timestamp = doc.get('timestamp')
            for frame_id, pose in doc.items():
                if frame_id == 'timestamp':
                    continue
                if isinstance(pose, (list, tuple, np.ndarray)):
                    pose_mat = np.array(pose, dtype=float)
                    if pose_mat.shape != (4, 4):
                        raise ValueError(f"Unexpected pose shape: {pose_mat.shape}")
                    pose = matrix_to_pose(pose_mat)
                elif isinstance(pose, dict):
                    pose = dict_to_pose(pose)
                
                frame_poses.setdefault(frame_id, []).append(pose)
                
            timestamps.append(timestamp)

    return frame_poses, timestamps


def read_poses_from_yaml_single_obstacle(yaml_path):
    frame_id = 'B1'
    frame_poses = {}
    timestamps = []
    with open(yaml_path, 'r') as f:
        for doc in yaml.safe_load_all(f):
            if doc is None:
                continue
            timestamp = doc.get('timestamp')
            pose = dict_to_pose(doc)
            frame_poses.setdefault(frame_id, []).append(pose)
            timestamps.append(timestamp)

    return frame_poses, timestamps



def get_average_pose(file_path: Path, return_type: str = "matrix"):
    from scipy.spatial.transform import Rotation as R
    from geometry_msgs.msg import Pose

    # Read the box poses from the yaml file
    frame_poses = read_poses_from_yaml(file_path)

    average_poses = {}
    for frame_id, poses in frame_poses.items():
        pos_array = np.array([[p.position.x, p.position.y, p.position.z] for p in poses])
        mean_pos = pos_array.mean(axis=0)
    
        quat_array = np.array([
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            for p in poses
        ])
    
        rots = R.from_quat(quat_array)   # SciPy expects [x, y, z, w]
        mean_rot = rots.mean()
        mean_q_scipy = mean_rot.as_quat()

        mean_pose = Pose()
        mean_pose.position.x, mean_pose.position.y, mean_pose.position.z = mean_pos
        mean_pose.orientation.x, mean_pose.orientation.y, mean_pose.orientation.z, mean_pose.orientation.w = mean_q_scipy

        if return_type.lower() == "pose":
            average_poses[frame_id] = mean_pose
        elif return_type.lower() == "matrix":
            average_poses[frame_id] = pose_to_matrix(mean_pose)
        else:
            raise ValueError("return_type must be 'pose' or 'matrix'.")

    return average_poses

if __name__ == "__main__":
    box_pose_file = Path(__file__).parent / "Data" / "disturbed_poses.yaml"
    mat = get_average_pose(box_pose_file, "pose")
    print("Mean pose matrix from camera:")
    print(mat)