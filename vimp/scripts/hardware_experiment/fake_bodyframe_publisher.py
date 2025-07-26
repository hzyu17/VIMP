import rospy
from geometry_msgs.msg import PoseStamped
import random
from vimp.thirdparty.sensor3D_tools.lcm.pcd_lcm_sender import construct_poselcm_msg, publish_lcm_msg
import time, select, sys, os
import numpy as np
import yaml
from pathlib import Path
from ack_lcm.acknowledgment.ack_t import ack_t
import lcm
from read_default_poses import read_cam_pose, read_body_pose
from pose_helpers import matrix_to_pose, pose_to_matrix


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
    def __init__(self, topic_name, wait_key=False, output_file: str = 'Data/poses_111.yaml'):
        self._wait_key = wait_key
        self._topic_name = topic_name  # Send message to this topic when the pose publishing is done

        cfg_path = Path(__file__).parent / "config" / "config.yaml"
        self.output_file = str(Path(__file__).parent / output_file)

        # Load camera pose
        self._T_world_cam = read_cam_pose(cfg_path)

        # Initial body frame pose
        body_poses, body_topics = read_body_pose(cfg_path) # Read body frame pose from config / Body_frames
        # Convert all body poses to matrix form
        self._default_poses = {k: pose_to_matrix(v) for k, v in body_poses.items()}
        self._new_poses = {k: np.eye(4) for k in body_poses.keys()}  # Initialize new poses as identity matrices
        self._topics = body_topics

        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        self._output_yaml = open(self.output_file, 'a')

        rospy.init_node('apriltag_webcam', anonymous=True)
        # Create a ROS publisher for every topic in self._topics
        self._pubs = {frame_id: rospy.Publisher(topic, PoseStamped, queue_size=10) 
                      for frame_id, topic in self._topics.items()}

        rospy.loginfo(f"[{rospy.get_name()}] listening to topics {list(self._topics.values())},"
                  " hit ENTER to pop & publish on {output_topic}")
        
        # box_pose = self._default_poses['B1']
        # B1_world_body = self._T_world_cam @ box_pose
        # transform = np.eye(4, dtype=np.float32)
        # transform[:3, 3] = [0.04, -0.33, 0.0]
        # B2_world_body = transform @ B1_world_body
        # print(f"B1 in world frame: {B1_world_body}")
        # print(f"B2 in world frame: {B2_world_body}")
        # B2_cam_body = np.linalg.inv(self._T_world_cam) @ B2_world_body
        # print(f"B2 in pose: {matrix_to_pose(B2_cam_body)}")

        # Acknowledgment
        self._lc = lcm.LCM()
        ack_topic = "ACK_" + self._topic_name
        self._lc.subscribe(ack_topic, self.ack_handler)

        self._ready_to_publish = False
        self._first_time = True
            
    def publish_pose(self, pose, frame_id, topic):
        lcm_msg = construct_poselcm_msg(pose, name=frame_id)
        publish_lcm_msg(lcm_msg, topic)

        print("LCM stamped pose message")
        print("   frame name  = %s" % str(lcm_msg.frame_name))
        print("   timestamp   = %s" % str(lcm_msg.timestamp))
        print("   pose    = %s" % str(lcm_msg.pose))

        # Publish ROS message for RViz
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "camera_frame"
        pose_msg.pose = matrix_to_pose(pose)

        self._pubs[frame_id].publish(pose_msg)
        print("Published pose to ROS topic ", self._topics[frame_id])
        
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
                
                for frame_id, default_pose in self._default_poses.items():
                    if self._first_time:
                        x_rand, y_rand, theta_rand = 0, 0, 0
                    else:
                        x_rand = random.random() * 0.05  # x_range = [0, 0.05]
                        y_rand = (random.random() - 0.5) * 0.06 # y_range = [-0.03, 0.03]
                        theta_rand = random.uniform(-np.pi/2, np.pi/2)

                    # Get object's coordinate in the world frame
                    object_pose = self._T_world_cam @ default_pose
                    topic = self._topics[frame_id]

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
                    transformation = T_back @ R_z @ T_to_origin
                
                    # Transform to world frame, apply the transformation, then convert back to body frame
                    new_body_pose = np.linalg.inv(self._T_world_cam) @ transformation @ self._T_world_cam @ default_pose

                    self._new_poses[frame_id] = new_body_pose
                    self.publish_pose(new_body_pose, frame_id, topic)
                    time.sleep(0.1)  # Add a small delay to control the loop speed

                info = {
                    "timestamp": int(time.time()),
                    "statues": "Done",
                    # encode the new poses in the message
                }
                done_msg = yaml.safe_dump(info).encode('utf-8')
                self._lc.publish(self._topic_name, done_msg)
                print(f"Published done message to topic {self._topic_name}")

                self._ready_to_publish = False
                self._first_time = False
            
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

    def ack_handler(self, channel, data):
        ack = ack_t.decode(data)
        print(f"Publisher got Ack for msg #{ack.msg_id} (sent at {ack.timestamp})")
        self._ready_to_publish = True
        

if __name__ == '__main__':
    pose_publisher = PosePublisher('/pose_publish', wait_key=True)
    pose_publisher.run()