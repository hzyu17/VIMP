from D435_receive import CameraRedisPubInterface, CameraRedisSubInterface

# Read and show the video stream from the webcam using OpenCV
# Detect the april tag in the video stream

import sys

from vimp.thirdparty.AprilTag.scripts.apriltag_image import apriltag_image2pose
from vimp.thirdparty.sensor3D_tools.lcm.pcd_lcm_sender import construct_poselcm_msg, publish_lcm_msg

import cv2
import time
import threading 
from collections import deque

import rospy, select
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf
import numpy as np
import yaml
from pathlib import Path

from scipy.spatial.transform import Rotation as R

class CameraAprilTagReader:
    def __init__(self, buffer_size, topic_name, wait_key=False):
        self._buffer = deque(maxlen=buffer_size)
        self._lock = threading.Lock()
        self._wait_key = wait_key
        self._topic_name = topic_name
        self._cam_receiver = CameraRedisSubInterface()

        self._output = []
        self._count_output = 0
        output_file = Path(__file__).parent / "Data" / "box_poses.yaml"
        self._output_yaml = open(output_file, 'a')
        
        rospy.init_node('apriltag_D435', anonymous=True)
        self._pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
        
        rospy.loginfo(f"[{rospy.get_name()}] listening to {topic_name},"
                  " hit ENTER to pop & publish on {output_topic}")
        
        import json
        json_path = Path(__file__).parent / "UTF-8realsense_high_d435_no_table_tuned_p2048_w_icp.json"          # change the filename if needed
        with json_path.open("r") as f:
            cfg = json.load(f) 
        K = np.asarray(cfg["K"], dtype=np.float32)
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        self._d435_params = (fx, fy, cx, cy)
        
        self._tag_size = 0.075

        # self._cap = cv2.VideoCapture(0)         
        # if not self._cap.isOpened():
        #     raise IOError("Cannot open webcam")
    
    
    def read_april_tag(self):
        json_str = self._cam_receiver.info_redis.get(f"{self._cam_receiver.camera_name}::last_img_info")
        if json_str is not None:
            for _ in range(5):
                self._cam_receiver.save_img(flag=True)
                time.sleep(0.02)

        raw_frame = self._cam_receiver.get_img()
        rgb_img = cv2.cvtColor(np.array(raw_frame["color"]), cv2.COLOR_BGR2RGB)

        cv2.imshow("live", rgb_img)     
        if cv2.waitKey(1) == 27:      
            return False
        
        pose = apriltag_image2pose(input_img_numpy=rgb_img,
                                    display_images=False,
                                    detection_window_name='AprilTag',
                                    tag_size=self._tag_size,
                                    camera_params=self._d435_params)
                
        if pose is not None:
            with self._lock:
                self._buffer.append(pose)
        
        return True
                
                
    def publish_pose(self):
        with self._lock:
            if self._buffer:
                pose = self._buffer.popleft()

                msg = construct_poselcm_msg(pose, name='B1')
                publish_lcm_msg(msg, topic=self._topic_name)
                
                # Publish ROS message for RViz
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "camera_frame"
                
                t = pose[:3, 3]
                q = tf.quaternion_from_matrix(pose)

                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = t
                
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]
                
                self._pub.publish(pose_msg)
                print("Published pose to ROS topic ", self._topic_name)

                # save pose
                save_pose = True
                if save_pose and self._count_output < 100:
                    entry = {
                            "timestamp": int(pose_msg.header.stamp.to_sec()),
                            "pose": pose.tolist()
                            }
                    print("Saving box pose!")
                    yaml.safe_dump(entry, self._output_yaml)
                    # add document separator
                    self._output_yaml.write('---\n')
                    self._output_yaml.flush()
                    
                    self._count_output += 1
                    
                else:
                    self._output_yaml.close()
                    return 


    def run(self):
        """
        This function captures video from the webcam, detects AprilTags in the frames,
        and publishes the pose of the detected AprilTags to LCM and ROS.
        """
        
        while True:
            result = self.read_april_tag()
            
            if not result:
                break
            
            if self._wait_key:
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    ch = sys.stdin.read(1)
                    if ch == '\n': # if 'Enter' key is pressed
                        rospy.loginfo("Enter pressed — publishing one pose")
                        self.publish_pose()
                        
            time.sleep(0.1)  # Add a small delay to control the loop speed
            
        # self._cap.release()
        cv2.destroyAllWindows()

    
if __name__ == "__main__":
    # --- Obtain and save box poses ---
    reader = CameraAprilTagReader(buffer_size=10, topic_name='/B1_pose', wait_key=True)
    reader.run()
    
    # # --- Read box poses in D435 coordinate ---
    # box_pose_file = Path(__file__).parent / "Data" / "box_poses.yaml"
    # pose_mats = read_poses_from_yaml(box_pose_file)
    
    # # from pose_helpers import matrix_to_pose
    # poses = []
    # for pose_mat in pose_mats:
    #     poses.append(matrix_to_pose(pose_mat))
    
    # pos_array = np.array([
    #     [p.position.x, p.position.y, p.position.z]
    #     for p in poses
    # ])
    # mean_pos = pos_array.mean(axis=0)
    
    
    # quat_array = np.array([
    #     [p.orientation.x,
    #     p.orientation.y,
    #     p.orientation.z,
    #     p.orientation.w]
    #     for p in poses
    # ])
    # mean_q_lin = quat_array.mean(axis=0)
    # mean_q_lin /= np.linalg.norm(mean_q_lin)
    
    # rots = R.from_quat(quat_array)   # SciPy expects [x, y, z, w]
    # mean_rot = rots.mean()
    # mean_q_scipy = mean_rot.as_quat()
    
    # print("mean position: ", mean_pos)
    # print("mean quaternion: ", mean_q_lin)
    # print("mean q_scipy: ", mean_q_lin)