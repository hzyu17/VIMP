import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
import numpy as np
import tf

import threading
from collections import deque

from read_defaul_poses import *

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
                 config_file: str,
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
        
        # store relative poses & sizes
        (self._body_box, self._body_topic, 
         self._rel_poses, self._sizes, self._body_default_pose, _) = load_body_frames_config(config_file)
        
        self._T_cam = read_cam_pose(config_file)
        
        self._body_lcm_topic = self._body_topic
        
        
        # ring buffers for each body frame
        self._buffers = {
            name: deque(maxlen=buffer_size)
            for name in self._body_topic
        }
        self._lock = threading.Lock()

        # set up MoveIt planning scene
        rospy.init_node("scene_box_demo")
        self._scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(1.0)

        # subscribe to topics
        for name, topic in self._body_topic.items():
            rospy.Subscriber(topic,
                             PoseStamped,
                             callback=self._make_cb(name),
                             queue_size=1)
            
        self._received_newpose = False

        # timer to push updates at fixed rate
        self._timer = rospy.Timer(rospy.Duration(1.0/update_hz), callback=self._on_timer)
        

    def _make_cb(self, name):
        def _cb(msg: PoseStamped):
            rospy.loginfo("Received pose message in the planning scene listener!")
            print("Received pose: ", msg.pose)
            with self._lock:
                self._buffers[name].append(msg)
                self._received_newpose = True
        return _cb


    def _on_timer(self, event):
        with self._lock:
            if self._received_newpose:
                self._received_newpose = False
                
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
                    T_w_box = self._T_cam @ T_w_b @ T_b_box
                    
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



if __name__ == "__main__":
    cfg_path = Path(__file__).parent / "config" / "config.yaml"
    body_box, body_topic, rel_poses, sizes, body_default_pose, _ = load_body_frames_config(cfg_path)

    # 2) Spin up the updater (from the previous example)
    updater = RealTimeBoxUpdater(
        cfg_path,
        buffer_size=10,
        update_hz=2.0,
    )
    updater.spin()
    
    