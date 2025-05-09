import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
import json, yaml
from pathlib import Path
from geometry_msgs.msg import Pose
import numpy as np
import tf

# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------
def pose_to_matrix(p: Pose) -> np.ndarray:
    """geometry_msgs/Pose ➜ 4×4 homogeneous matrix."""
    q = p.orientation
    t = p.position
    # quaternion (x,y,z,w)  ➜  3×3 rotation matrix
    R = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [t.x, t.y, t.z]
    return T

def matrix_to_pose(T: np.ndarray) -> Pose:
    """4×4 homogeneous matrix ➜ geometry_msgs/Pose."""
    R = T[:3, :3]
    t = T[:3,  3]
    q = tf.transformations.quaternion_from_matrix(T)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = t
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def boxes_world_poses(T_B_W: Pose, boxes_in_B: list[Pose]) -> list[Pose]:
    """
    Args
    ----
    T_B_W       Pose of frame B in world (W).
    boxes_in_B  List of box-centre poses wrt frame B.

    Returns
    -------
    List of poses of the same boxes in world frame.
    """
    TBW = pose_to_matrix(T_B_W)
    return [matrix_to_pose(TBW @ pose_to_matrix(pB)) for pB in boxes_in_B]


def load_per_box_yaml(path: Path):
    """
    Returns a list of (name, frame_id, Pose) for each box.
    """
    data = yaml.safe_load(path.read_text())
    result = []
    for entry in data["SDF_boxe_relative_poses"]["boxes"]:
        p = Pose()
        x, y, z = entry["center"]
        p.position.x, p.position.y, p.position.z = x, y, z
        qx, qy, qz, qw = entry["orientation"]
        p.orientation.x, p.orientation.y = qx, qy
        p.orientation.z, p.orientation.w = qz, qw
        result.append((entry["name"], entry["frame_id"], p))
    return result


def idx_to_world(idx: int, origin: float, cell_size: float) -> float:
    return origin + cell_size * (idx + 0.5)


def add_boxes_from_sdf_json(json_file: Path,
                            scene: PlanningSceneInterface) -> None:

    data = json.loads(json_file.read_text())

    cell   = float(data["cell_size"])
    origin = data["origin"]                

    for i, obs in enumerate(data["obstacles"], start=1):
        x0, x1, y0, y1, z0, z1 = obs["corner_idx"]  

        # --- geometric size -----------------
        size_x = (x1 - x0 + 1) * cell 
        size_y = (y1 - y0 + 1) * cell
        size_z = (z1 - z0 + 1) * cell

        # --- centre position ----------------
        cx = idx_to_world((x0 + x1) / 2.0, origin[0], cell)
        cy = idx_to_world((y0 + y1) / 2.0, origin[1], cell)
        cz = idx_to_world((z0 + z1) / 2.0, origin[2], cell)
        
        # Original pose of the boxes
        pose = PoseStamped()
        frame="world"
        pose.header.frame_id = frame
        pose.pose.position.x = cx
        pose.pose.position.y = cy
        pose.pose.position.z = cz
        pose.pose.orientation.w = 1.0 

        name = f"sdf_box_{i}"
        scene.add_box(name, pose, size=(size_x, size_y, size_z))
        scene.wait_for_known_object(name, timeout=0.5)
        rospy.loginfo(f"Added {name} at ({cx:.3f}, {cy:.3f}, {cz:.3f}) " 
                      f"size=({size_x:.3f}, {size_y:.3f}, {size_z:.3f})")
        

def add_box_with_pose(pose):
    rospy.init_node("scene_box_demo")
    planning_scene = PlanningSceneInterface(synchronous=True)
    rospy.sleep(1.0)
    
    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0.50      # metres
    box_pose.pose.position.y = 0.10
    box_pose.pose.position.z = 0.25
    box_pose.pose.orientation.w = 1.0
    
    box_size = (0.30, 0.20, 0.50)
    box_name = "demo_box"

    planning_scene.add_box(box_name, box_pose, size=box_size)
    
    planning_scene.wait_for_known_object(box_name, timeout=1.0)
    

if __name__ == "__main__":
    config_file = Path(__file__).parent.parent / "hardware_experiment" / "config" / "franka_hardware.yaml"
    print("config_file", config_file)
    boxes = load_per_box_yaml(config_file)
    
    print(boxes)