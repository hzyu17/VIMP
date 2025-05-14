from vimp.thirdparty.AprilTag.scripts.apriltag_image import apriltag_image2pose
from vimp.thirdparty.sensor3D_tools.lcm.pcd_lcm_sender import construct_poselcm_msg, publish_lcm_msg
import sys, select, rospy
import cv2
import time
import threading 
from collections import deque
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf
import json 
from pathlib import Path
import pyrealsense2 as rs
import numpy as np


def read_camera_params(json_path: str):
    """
    Read camera intrinsics from a JSON file with a 3*3 'K' entry.
    
    Returns [fx, fy, cx, cy].
    """
    with open(json_path, 'r') as f:
        data = json.load(f)

    # grab the K matrix
    K = data.get('K')
    if K is None or len(K) < 2 or len(K[0]) < 3:
        raise ValueError(f"Invalid or missing 'K' matrix in {json_path}")

    fx = K[0][0]
    fy = K[1][1]
    cx = K[0][2]
    cy = K[1][2]

    return (fx, fy, cx, cy)


camera_config = str(Path(__file__).parent / "config" / "UTF-8realsense_high_d435_no_table_tuned_p2048_w_icp.json")

class D435AprilTagReader:
    def __init__(self, buffer_size, topic_name, wait_key=False):
        self._buffer = deque(maxlen=buffer_size)
        self._lock = threading.Lock()
        self._wait_key = wait_key
        self._topic_name = topic_name
        
        rospy.init_node('apriltag_webcam', anonymous=True)
        self._pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
        
        rospy.loginfo(f"[{rospy.get_name()}] listening to {topic_name},"
                  " hit ENTER to pop & publish on {output_topic}")

        self._cap = cv2.VideoCapture(2, cv2.CAP_V4L2)    
        
        # RealSense camera
        self._pipeline = rs.pipeline()
        self._rs_cfg = rs.config()
        self._rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self._pipeline.start(self._rs_cfg)
        
        # set desired resolution
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  
          
        if not self._cap.isOpened():
            raise IOError("Cannot open D435 camera")
        
        self._camera_params = read_camera_params(camera_config)
        print("Camera params: ", self._camera_params)
    
    
    def read_april_tag(self):
        frames = self._pipeline.wait_for_frames()
        
        color_frame = frames.get_color_frame()
        frame = np.asanyarray(color_frame.get_data())

        cv2.imshow("D435 Color (via V4L2)", frame)     
        if cv2.waitKey(1) == 27:      
            return False
        
        pose = apriltag_image2pose(input_img_numpy=frame,
                                    display_images=False,
                                    detection_window_name='AprilTag',
                                    camera_params=self._camera_params)
        
        print("pose: ", pose)
        
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
            
        self._pipeline.stop()
        cv2.destroyAllWindows()
    

if __name__ == '__main__':
    reader = D435AprilTagReader(buffer_size=10, topic_name='/B1_pose', wait_key=True)