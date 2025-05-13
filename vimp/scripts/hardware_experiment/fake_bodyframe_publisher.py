import rospy
from geometry_msgs.msg import PoseStamped
import random
from vimp.thirdparty.sensor3D_tools.lcm.pcd_lcm_sender import construct_poselcm_msg, publish_lcm_msg
import tf.transformations as tf
import time, select, sys
import numpy as np


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
    def __init__(self, topic_name, wait_key=False):
        self._wait_key = wait_key
        self._topic_name = topic_name
        
        self._pose = np.eye(4, dtype=np.float32)
        
        rospy.init_node('apriltag_webcam', anonymous=True)
        self._pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
        
        rospy.loginfo(f"[{rospy.get_name()}] listening to {topic_name},"
                  " hit ENTER to pop & publish on {output_topic}")
            
            
    def publish_pose(self, pose):

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
                            
                if self._wait_key:
                    if select.select([sys.stdin], [], [], 0.0)[0]:
                        ch = sys.stdin.read(1)
                        if ch == '\n': # if 'Enter' key is pressed
                            rospy.loginfo("Enter pressed — publishing one pose")
                            pose = np.eye(4, dtype=np.float32)
                            pose[0, 3] = 0.5
                            pose[1, 3] = 0.5
                            pose[2, 3] = 0.5
                            self.publish_pose(pose)
                            
                time.sleep(0.1)  # Add a small delay to control the loop speed
            


if __name__ == '__main__':
    pose_publisher = PosePublisher('/B1_pose', True)
    pose_publisher.run()