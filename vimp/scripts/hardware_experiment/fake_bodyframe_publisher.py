import rospy
from geometry_msgs.msg import PoseStamped
import random
from vimp.thirdparty.sensor3D_tools.lcm.pcd_lcm_sender import construct_poselcm_msg, publish_lcm_msg
import tf.transformations as tft
import time, select, sys, os
import numpy as np
import yaml
from pathlib import Path
from ack_lcm.acknowledgment.ack_t import ack_t
import lcm
from apriltag_d435 import read_poses_from_yaml
from read_defaul_poses import read_cam_pose, read_body_pose


def make_random_msg():
    msg = PoseStamped()
    msg.header.frame_id = "world"          # or whatever frame you need
    msg.pose.position.x    = random.random()
    msg.pose.position.y    = 1.0
    msg.pose.position.z    = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    
    return msg


class PosePublisher:
    def __init__(self, topic_name, wait_key=False, output_file: str = 'Data/poses.yaml'):
        self._wait_key = wait_key
        self._topic_name = topic_name

        cfg_path = Path(__file__).parent / "config" / "config.yaml"
        self.output_file = str(Path(__file__).parent / output_file)

        # Load camera pose
        self._T_world_cam = read_cam_pose(cfg_path)

        # Initial body frame pose
        self._pose = np.eye(4, dtype=np.float32)
        pose = read_body_pose(cfg_path)
        self._pose[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        self._pose[:3, :3] = tft.quaternion_matrix(quat)[:3, :3]
        
        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        self._output_yaml = open(self.output_file, 'a')
                
        rospy.init_node('apriltag_webcam', anonymous=True)
        self._pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
        
        rospy.loginfo(f"[{rospy.get_name()}] listening to {topic_name},"
                  " hit ENTER to pop & publish on {output_topic}")
        
        # Acknowledgment
        self._lc = lcm.LCM()
        ack_topic = "ACK_" + self._topic_name
        self._lc.subscribe(ack_topic, self.ack_handler)
        
        self._ready_to_publish = False
        self._first_time = True
            
    def publish_pose(self, pose):
        lcm_msg = construct_poselcm_msg(pose, name='B1')
        publish_lcm_msg(lcm_msg, topic=self._topic_name)
        
        print("LCM stamped pose message")
        print("   frame name  = %s" % str(lcm_msg.frame_name))
        print("   timestamp   = %s" % str(lcm_msg.timestamp))
        print("   pose    = %s" % str(lcm_msg.pose))
        
        # Publish ROS message for RViz
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "camera_frame"
        
        t = pose[:3, 3]
        q = tft.quaternion_from_matrix(pose)

        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = t
        
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        
        self._pub.publish(pose_msg)
        print("Published pose to ROS topic ", self._topic_name)
        
        # Record the poses in the json file
        entry = {
            "timestamp": lcm_msg.timestamp, # Use lcm message timestamp
            "position": {
                "x": float(pose_msg.pose.position.x),
                "y": float(pose_msg.pose.position.y),
                "z": float(pose_msg.pose.position.z),
            },
            "orientation": {
                "x": float(pose_msg.pose.orientation.x),
                "y": float(pose_msg.pose.orientation.y),
                "z": float(pose_msg.pose.orientation.z),
                "w": float(pose_msg.pose.orientation.w),
            },
        }
        # write exactly one JSON object per line
        yaml.safe_dump(entry, self._output_yaml)
        # add document separator
        self._output_yaml.write('---\n')
        self._output_yaml.flush()
        
    
    def _on_shutdown(self):
        self._json.close()
        
        
    def run(self):
        """
        This function captures video from the webcam, detects AprilTags in the frames,
        and publishes the pose of the detected AprilTags to LCM and ROS.
        """
                    
        while True:
            self._lc.handle_timeout(0)
            
            if self._wait_key:
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    ch = sys.stdin.read(1)
                    if ch == '\n': # if 'Enter' key is pressed
                        rospy.loginfo("Enter pressed — publishing one pose")
                        self._ready_to_publish = True
            
            if self._ready_to_publish:
                if self._first_time:
                    x_rand, y_rand, theta_rand = 0, 0, 0
                    self._first_time = False
                else:
                    x_rand = random.random() * 0.05  # x_range = [0, 0.05]
                    y_rand = (random.random() - 0.5) * 0.06 # y_range = [-0.03, 0.03]
                    theta_rand = random.uniform(-np.pi/12, np.pi/12)

                # Get object's coordinate in the world frame from self._T_world_cam and self._pose
                object_pose = self._T_world_cam @ self._pose
                
                # Translation matrix to move the object back to the origin 
                T_to_origin = np.eye(4, dtype=np.float32)
                T_to_origin[:3, 3] = -object_pose[:3, 3]
                
                # Rotation around the z axis (in radians)
                    # random rotation between -30° and 30°
                R_z = np.array([
                    [np.cos(theta_rand), -np.sin(theta_rand), 0, 0],
                    [np.sin(theta_rand),  np.cos(theta_rand), 0, 0],
                    [0,              0,             1, 0],
                    [0,              0,             0, 1]
                ], dtype=np.float32)
                
                # New desired translation for the object (to be applied after rotation)
                T_back = np.eye(4, dtype=np.float32)
                T_back[:3, 3] = object_pose[:3, 3]
                T_back[0, 3] += x_rand
                T_back[1, 3] += y_rand
                
                # Combine: translate to origin, rotate, then translate back
                pose = T_back @ R_z @ T_to_origin
                new_body_pose = np.linalg.inv(self._T_world_cam) @ pose @ self._T_world_cam @ self._pose

                self.publish_pose(new_body_pose)
                self._ready_to_publish = False
                        
            time.sleep(0.1)  # Add a small delay to control the loop speed
    

    def run_with_given_poses(self, poses):
        idx = 0
        total_poses = len(poses)

        while idx < total_poses and not rospy.is_shutdown():
            self._lc.handle_timeout(0)
            
            if self._wait_key:
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    ch = sys.stdin.read(1)
                    if ch == '\n':
                        rospy.loginfo("Enter pressed — publishing one pose")
                        self._ready_to_publish = True
                
            if self._ready_to_publish:
                pose = poses[idx]
                self.publish_pose(pose)
                idx += 1
                self._ready_to_publish = False
                
            time.sleep(0.2)  # Add a small delay to control the loop speed


    def obtain_box_pose(self):
        from bodyframe_pose_listener import matrix_to_pose, pose_to_matrix
        from scipy.spatial.transform import Rotation as R
        from geometry_msgs.msg import Pose

        # Read the box poses from the yaml file
        box_pose_file = Path(__file__).parent / "Data" / "box_poses_hardware.yaml"
        pose_mats = read_poses_from_yaml(box_pose_file)
        
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

        mean_pose_mat = pose_to_matrix(mean_pose)
        
        return mean_pose_mat
    

    def ack_handler(self, channel, data):
        ack = ack_t.decode(data)
        print(f"Publisher got Ack for msg #{ack.msg_id} (sent at {ack.timestamp})")
        self._ready_to_publish = True
        

if __name__ == '__main__':
    pose_publisher = PosePublisher('/B1_pose', wait_key=True)
    pose_publisher.run()
    # pose_publisher.obtain_box_pose()