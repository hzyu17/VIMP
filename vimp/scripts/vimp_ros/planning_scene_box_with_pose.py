import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
import json, yaml
from pathlib import Path
from geometry_msgs.msg import Pose
import numpy as np
import tf


# -----------
#   Helpers
# -----------
class BoxObstcle:
    def __init__(self):
        self.geometry = None
        self.bodypose = None
        self.name = None
    
    def update_geometry(self, box_geometry):
        self.geometry = box_geometry
        
    def update_bodypose(self, box_bodypose):
        self.bodypose = box_bodypose


class BoxGeometry:
    def __init__(self, size=None):
        self.size = size


class BodyFramePose:
    def __init__(self, position=None, orientation=None):
        self.position = position
        self.orientation = orientation


def pose_to_matrix(p: Pose):
    """geometry_msgs/Pose ➜ 4×4 homogeneous matrix."""
    q = p.orientation
    t = p.position
    # quaternion (x,y,z,w)  ➜  3×3 rotation matrix
    R = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [t.x, t.y, t.z]
    return T


def matrix_to_pose(T: np.ndarray):
    """4×4 homogeneous matrix ➜ geometry_msgs/Pose."""
    R = T[:3, :3]
    t = T[:3,  3]
    q = tf.transformations.quaternion_from_matrix(T)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = t
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def load_box_bodyposes_yaml(path: Path):
    """
    Returns a list of (name, frame_id, Pose) for each box, relative to the body frame attached to the box obstacle. This pose is fixed throughout.
    """
    data = yaml.safe_load(path.read_text())
    result = []
    for entry in data["SDF_box_bodyframes"]["boxes"]:
        p = Pose()
        x, y, z = entry["center"]
        p.position.x, p.position.y, p.position.z = x, y, z
        qx, qy, qz, qw = entry["orientation"]
        p.orientation.x, p.orientation.y = qx, qy
        p.orientation.z, p.orientation.w = qz, qw
        entry["pose"] = p
        result.append(entry)
    return result


def read_boxes(geometry_json, bodyframe_config):
    boxes = []    
    
    boxes_bodyframe = load_box_bodyposes_yaml(bodyframe_config)
    
    geo_data = json.loads(geometry_json.read_text())
    cell   = float(geo_data["cell_size"])        
       
    # Assert the geormetry data and body frame data are of the same length
    assert(len(geo_data["obstacles"]) == len(boxes_bodyframe))

    for i, obs in enumerate(geo_data["obstacles"], start=1):
        box_i = BoxObstcle()
        
        x0, x1, y0, y1, z0, z1 = obs["corner_idx"]  

        # --- geometric size -----------------
        size_x = (x1 - x0 + 1) * cell 
        size_y = (y1 - y0 + 1) * cell
        size_z = (z1 - z0 + 1) * cell
        
        size = np.array([size_x, size_y, size_z])
        geo_i = BoxGeometry(size=size)
        
        box_i.update_geometry(geo_i)
        
        # --- body frame pose ----------------
        pose_body = boxes_bodyframe[i-1]["pose"]
        box_i.update_bodypose(pose_body)
        
        
        boxes.append(box_i)
        
    return boxes
        

def box_world_pose(T_WB: Pose, boxe_in_B: Pose):
    """
    Args
    ----
    T_WB       Pose of frame B in world (W).
    boxe_in_B   box-centre poses wrt frame B.

    Returns
    -------
    Pose of the same boxe in world frame.
    """
    
    TBW = pose_to_matrix(T_WB)
    return matrix_to_pose(TBW @ pose_to_matrix(boxe_in_B))


# def add_box_world_poses(boxes_in_B: list, dict_T_WB: dict, scene: PlanningSceneInterface):
#     for box_pose in boxes_in_B:
#         B_frame_name = box_pose["frame_id"]
#         T_WB = dict_T_WB[B_frame_name]
#         box_pose_world = box_world_pose(T_WB, box_pose["pose"])
        
        
#         size_x = (x1 - x0 + 1) * cell 
#         size_y = (y1 - y0 + 1) * cell
#         size_z = (z1 - z0 + 1) * cell
        
#         pose = PoseStamped()
#         frame="world"
#         pose.header.frame_id = frame
#         pose.pose.position.x = box_pose_world[0]
#         pose.pose.position.y = box_pose_world[1]
#         pose.pose.position.z = box_pose_world[2]
        
#         pose.pose.orientation.w = 1.0 

#         name = f"sdf_box_{i}"
#         scene.add_box(name, pose, size=(size_x, size_y, size_z))
#         scene.wait_for_known_object(name, timeout=0.5)
#         rospy.loginfo(f"Added {name} at ({cx:.3f}, {cy:.3f}, {cz:.3f}) " 
#                       f"size=({size_x:.3f}, {size_y:.3f}, {size_z:.3f})")
        

def idx_to_world(idx: int, origin: float, cell_size: float):
    return origin + cell_size * (idx + 0.5)


def add_box_with_pose(pose):
    rospy.init_node("scene_box_demo")
    planning_scene = PlanningSceneInterface(synchronous=True)
    rospy.sleep(1.0)
    
    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0.50
    box_pose.pose.position.y = 0.10
    box_pose.pose.position.z = 0.25
    box_pose.pose.orientation.w = 1.0
    
    box_size = (0.30, 0.20, 0.50)
    box_name = "demo_box"

    planning_scene.add_box(box_name, box_pose, size=box_size)
    
    planning_scene.wait_for_known_object(box_name, timeout=1.0)
    


if __name__ == "__main__":
    config_body_pose = Path(__file__).parent.parent / "hardware_experiment" / "config" / "franka_hardware.yaml"
    config_geometry = Path(__file__).parent.parent.parent / "scripts" / "FrankaBoxDataset.json"
    
    boxes = read_boxes(config_geometry, config_body_pose)
    
    print(boxes)