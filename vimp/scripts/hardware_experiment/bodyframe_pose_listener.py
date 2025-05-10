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

# -----------
#   Helpers
# -----------

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
        

class RealTimeBoxUpdater:
    def __init__(self,
                 body_topics: dict,
                 box_sizes: dict,
                 box_rel_poses: dict,
                 buffer_size: int = 5,
                 update_hz: float = 1.0):
        """
        body_topics:    {frame_name: topic_name} for incoming PoseStamped
        box_sizes:      {frame_name: (sx,sy,sz)} in metres
        box_rel_poses:  {frame_name: Pose()} of the box in the body frame
        buffer_size:    how many old poses to keep
        update_hz:      how often to re-publish boxes into MoveIt!
        """
        # ring buffers for each body frame
        self._buffers = {
            name: deque(maxlen=buffer_size)
            for name in body_topics
        }
        self._lock = threading.Lock()

        # store relative poses & sizes
        self._rel_poses = box_rel_poses
        self._sizes     = box_sizes

        # set up MoveIt planning scene
        rospy.init_node("scene_box_demo")
        self._scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(1.0)

        # subscribe to topics
        for name, topic in body_topics.items():
            rospy.Subscriber(topic,
                             PoseStamped,
                             callback=self._make_cb(name),
                             queue_size=1)

        # timer to push updates at fixed rate
        self._timer = rospy.Timer(rospy.Duration(1.0/update_hz),
                                  callback=self._on_timer)

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

                # compose: T_world_box = T_world_body × T_body_box
                T_w_b = pose_to_matrix(latest.pose)
                T_b_box = pose_to_matrix(self._rel_poses[name])
                T_w_box = T_w_b @ T_b_box

                # make a stamped pose in world
                out = PoseStamped()
                out.header.frame_id = "world"
                out.header.stamp = rospy.Time.now()
                out.pose = matrix_to_pose(T_w_box)

                # update MoveIt!
                self._scene.add_box(name,
                                    out,
                                    size=self._sizes[name])

    def spin(self):
        rospy.spin()


def load_body_frames_config(path: Path):
    """
    Reads a YAML of the form:

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
      • body_topics:    { box_name: monitor_topic }
      • box_rel_poses:  { box_name: Pose() }            # Pose in its body frame
      • box_sizes:      { box_name: (sx, sy, sz) }
    """
    data = yaml.safe_load(path.read_text())
    body_topics   = {}
    box_rel_poses = {}
    box_sizes     = {}

    for entry in data["body_frames"]:
        topic = entry["monitor_topic"]
        bname = entry["box"]["name"]

        # 1) map box → topic
        body_topics[bname] = topic

        # 2) build a Pose for the box centre in its body frame
        p = Pose()
        cx, cy, cz = entry["box"]["center"]
        p.position.x, p.position.y, p.position.z = cx, cy, cz
        qx, qy, qz, qw = entry["box"]["orientation"]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
            qx, qy, qz, qw
        )
        box_rel_poses[bname] = p

        # 3) read out the size (you’ll need to add this in your YAML!)
        sx, sy, sz = entry["box"]["size"]
        box_sizes[bname] = (sx, sy, sz)

    return body_topics, box_rel_poses, box_sizes


if __name__ == "__main__":
    cfg_path = Path(__file__).parent / "config" / "body_frames.yaml"
    body_topics, rel_poses, sizes = load_body_frames_config(cfg_path)

    # 2) Spin up the updater (from the previous example)
    updater = RealTimeBoxUpdater(
        body_topics=body_topics,
        box_sizes=sizes,
        box_rel_poses=rel_poses,
        buffer_size=10,
        update_hz=2.0,
    )
    updater.spin()
    
    