import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
import yaml
from pathlib import Path
from geometry_msgs.msg import Pose
from pose_helpers import pose_to_matrix, matrix_to_pose
from read_default_poses import *
import numpy as np

import threading
from collections import deque

# import lcm
# from vimp.thirdparty.sensor3D_tools.lcm import construct_poselcm_msg

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


if __name__ == "__main__":
    cfg_path = Path(__file__).parent / "config" / "config.yaml"
    body_box, body_topic, rel_poses, sizes, body_default_pose, world_size = load_body_frames_config(cfg_path)

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
    
    