import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
import yaml
from pathlib import Path
from geometry_msgs.msg import Pose
import numpy as np
import tf

import threading
from collections import deque

# import lcm
# from vimp.thirdparty.sensor3D_tools.lcm import construct_poselcm_msg

# -----------
#   Helpers
# -----------

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
        

class RealTimeBoxUpdater:
    def __init__(self,
                 body_box: dict,
                 body_topic: dict,
                 box_sizes: dict,
                 box_rel_poses: dict,
                 buffer_size: int = 5,
                 update_hz: float = 1.0):
        """
        box_body:       {box_name: frame_name} mapping box name to its body frame
        body_topic:     {frame_name: topic_name} for incoming PoseStamped
        box_sizes:      {box_name: (sx,sy,sz)} in metres
        box_rel_poses:  {box_name: Pose()} of the box in the body frame
        buffer_size:    how many old poses to keep
        update_hz:      how often to re-publish boxes into MoveIt!
        """
        # ring buffers for each body frame
        self._buffers = {
            name: deque(maxlen=buffer_size)
            for name in body_topic
        }
        self._lock = threading.Lock()

        # store relative poses & sizes
        self._rel_poses = box_rel_poses
        self._sizes     = box_sizes
        self._body_box  = body_box
        
        self._body_lcm_topic = body_topic

        # set up MoveIt planning scene
        rospy.init_node("scene_box_demo")
        self._scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(1.0)

        # subscribe to topics
        for name, topic in body_topic.items():
            rospy.Subscriber(topic,
                             PoseStamped,
                             callback=self._make_cb(name),
                             queue_size=1)

        # timer to push updates at fixed rate
        self._timer = rospy.Timer(rospy.Duration(1.0/update_hz), callback=self._on_timer)

    def _make_cb(self, name):
        def _cb(msg: PoseStamped):
            with self._lock:
                self._buffers[name].append(msg)
        return _cb

    def _on_timer(self, event):
        with self._lock:
            for name in self._scene.get_known_object_names():
                self._scene.remove_world_object(name)
                
            for name, buf in self._buffers.items():
                if not buf:
                    continue
                latest: PoseStamped = buf[-1]
                
                bname = self._body_box[name]

                # compose: T_world_box = T_world_body × T_body_box
                T_w_b = pose_to_matrix(latest.pose)
                T_b_box = pose_to_matrix(self._rel_poses[bname])
                T_w_box = T_w_b @ T_b_box
                
                # Send out the lcm message for sdf construction
                # lcm = lcm.LCM()
                # msg = construct_poselcm_msg(name, T_w_b)
                # lcm.publish(self._body_lcm_topic[name], msg)

                # make a stamped pose in world
                out = PoseStamped()
                out.header.frame_id = "world"
                out.header.stamp = rospy.Time.now()
                out.pose = matrix_to_pose(T_w_box)

                # update MoveIt!
                self._scene.add_box(bname,
                                    out,
                                    size=self._sizes[bname])

    def spin(self):
        rospy.spin()


def load_body_frames_config(path: Path):
    """
    Reads a YAML of the form:

    Field:
        field_origin: [0, 0, 0] # [x, y, z] The origin of the field in the april tag's frame
        field_size: [500, 500, 500]
        cell_size: 0.01
        obstacles:
            - name: sdf_box_1
            bodyframe: B1
            center: [0.120, -0.300, 0.050]    # [x, y, z] in metres (wrt frame B)
            orientation: [0.0, 0.0, 0.0, 1.0] # quaternion (qx, qy, qz, qw)
            size: [ 0.18, 0.18, 0.38 ]
      
    body_frames:
      - frame_id: B1
        position:    [x, y, z]
        orientation: [qx, qy, qz, qw]
        monitor_topic: "/B1_pose"
        box:
          name: sdf_box_1
          center: [cx, cy, cz]
          orientation: [qx, qy, qz, qw]
          size: [sx, sy, sz]

    Returns three dicts:
        box_bodyframe: {box_name: bodyframe_id}
        body_topic:    { bodyframe_id: monitor_topic }
        box_rel_poses:  { box_name: Pose() }            # Pose in its body frame
        box_sizes:      { box_name: (sx, sy, sz) }
        body_default_pose: {bodyframe_id: Pose() }
    """
    data = yaml.safe_load(path.read_text())
    body_box   = {}
    body_topic = {}
    box_rel_poses = {}
    box_sizes     = {}
    body_default_pose = {}

    for entry in data["Body_frames"]:
        topic = entry["monitor_topic"]
        fname = entry["frame_id"]
        bname = entry["box"]["name"]
        
        # map box : topic
        body_topic[fname] = topic
        
        body_box[fname] = bname
        
        # bodyframe : default pose
        p = Pose()
        cx, cy, cz = entry["position"]
        p.position.x, p.position.y, p.position.z = cx, cy, cz
        qx, qy, qz, qw = entry["orientation"]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
            qx, qy, qz, qw
        )
        
        body_default_pose[fname] = p

        
    for entry in data["Field"]["obstacles"]:
        # build a Pose for the box centre in its body frame
        bname = entry["name"]
        frame_id = entry["bodyframe"]
        
        # box_bodyframe[bname] = frame_id
        
        p = Pose()
        cx, cy, cz = entry["center"]
        p.position.x, p.position.y, p.position.z = cx, cy, cz
        qx, qy, qz, qw = entry["orientation"]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
            qx, qy, qz, qw
        )
        box_rel_poses[bname] = p

        # 3) read out the size (you’ll need to add this in your YAML!)
        sx, sy, sz = entry["size"]
        box_sizes[bname] = (sx, sy, sz)
        
    print("body_box")
    print(body_box)
    print("body_topic")
    print(body_topic)
    print("box_rel_poses")
    print(box_rel_poses)
    print("box_sizes")
    print(box_sizes)
    print("body_default_pose")
    print(body_default_pose)

    return body_box, body_topic, box_rel_poses, box_sizes, body_default_pose


if __name__ == "__main__":
    cfg_path = Path(__file__).parent / "config" / "config.yaml"
    body_box, body_topic, rel_poses, sizes, body_default_pose = load_body_frames_config(cfg_path)

    # 2) Spin up the updater (from the previous example)
    updater = RealTimeBoxUpdater(
        body_box = body_box,
        body_topic=body_topic,
        box_sizes=sizes,
        box_rel_poses=rel_poses,
        buffer_size=10,
        update_hz=2.0,
    )
    updater.spin()
    
    