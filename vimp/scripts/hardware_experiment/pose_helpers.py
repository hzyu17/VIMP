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
    R = T[:3, :3]
    t = T[:3,  3]
    q = tf.transformations.quaternion_from_matrix(T)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = t
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def read_poses_from_yaml(yaml_path):
    """
    Reads a multi-document YAML file where each document has a 'pose'
    (a 4-by-4 nested list) and a 'timestamp' field.

    Returns a list of (pose_matrix, timestamp) tuples:
      - pose_matrix: a (4,4) numpy array
      - timestamp: the integer timestamp
    """
    poses = []
    with open(yaml_path, 'r') as f:
        # safe_load_all will iterate over each document separated by '---'
        for doc in yaml.safe_load_all(f):
            if doc is None:
                continue
            pose_list = doc.get('pose')
            timestamp = doc.get('timestamp')
            if pose_list is None or timestamp is None:
                continue  # skip docs without the expected fields

            # convert nested Python lists to a NumPy array
            pose_mat = np.array(pose_list, dtype=float)
            if pose_mat.shape != (4,4):
                raise ValueError(f"Unexpected pose shape: {pose_mat.shape}")

            poses.append(pose_mat)

    return poses


def get_average_pose(file_path: Path, return_type: str = "matrix"):
    from scipy.spatial.transform import Rotation as R
    from geometry_msgs.msg import Pose

    # Read the box poses from the yaml file
    pose_mats = read_poses_from_yaml(file_path)
    
    poses = []
    for pose_mat in pose_mats:
        poses.append(matrix_to_pose(pose_mat))

    pos_array = np.array([
        [p.position.x, p.position.y, p.position.z]
        for p in poses
    ])
    mean_pos = pos_array.mean(axis=0)
    
    quat_array = np.array([
        [p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w]
        for p in poses
    ])
    
    rots = R.from_quat(quat_array)   # SciPy expects [x, y, z, w]
    mean_rot = rots.mean()
    mean_q_scipy = mean_rot.as_quat()

    mean_pose = Pose()
    mean_pose.position.x, mean_pose.position.y, mean_pose.position.z = mean_pos
    mean_pose.orientation.x, mean_pose.orientation.y, mean_pose.orientation.z, mean_pose.orientation.w = mean_q_scipy

    if return_type.lower() == "pose":
        return mean_pose
    elif return_type.lower() == "matrix":
        # Convert the mean pose to a 4x4 matrix
        return pose_to_matrix(mean_pose)
    else:
        raise ValueError("return_type must be 'pose' or 'matrix'.")

if __name__ == "__main__":
    box_pose_file = Path(__file__).parent / "Data" / "box_poses.yaml"
    mat = get_average_pose(box_pose_file, "pose")
    print("Mean pose matrix from camera:")
    print(mat)