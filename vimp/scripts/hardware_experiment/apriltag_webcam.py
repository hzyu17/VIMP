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


class CameraAprilTagReader:
    def __init__(self, buffer_size, topic_name, wait_key=False):
        self._buffer = deque(maxlen=buffer_size)
        self._lock = threading.Lock()
        self._wait_key = wait_key
        self._topic_name = topic_name
        
        rospy.init_node('apriltag_webcam', anonymous=True)
        self._pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
        
        rospy.loginfo(f"[{rospy.get_name()}] listening to {topic_name},"
                  " hit ENTER to pop & publish on {output_topic}")

        self._cap = cv2.VideoCapture(0)         
        if not self._cap.isOpened():
            raise IOError("Cannot open webcam")
    
    
    def read_april_tag(self):
        ret, frame = self._cap.read()        
        if not ret:
            return False

        cv2.imshow("live", frame)     
        if cv2.waitKey(1) == 27:      
            return False
        
        pose = apriltag_image2pose(input_img_numpy=frame,
                                    display_images=False,
                                    detection_window_name='AprilTag',
                                    camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909))
        
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
            
        self._cap.release()
        cv2.destroyAllWindows()
    
    
    
if __name__ == "__main__":
    reader = CameraAprilTagReader(buffer_size=10, topic_name='/B1_pose', wait_key=True)
    reader.run()
    
    