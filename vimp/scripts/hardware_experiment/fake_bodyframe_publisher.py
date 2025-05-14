import rospy
from geometry_msgs.msg import PoseStamped
import random
from vimp.thirdparty.sensor3D_tools.lcm.pcd_lcm_sender import construct_poselcm_msg, publish_lcm_msg
import tf.transformations as tf
import time, select, sys, os
import numpy as np
import yaml
from pathlib import Path
from ack_lcm.acknowledgment.ack_t import ack_t
import lcm


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

        self.output_file = str(Path(__file__).parent / output_file)
        self._pose = np.eye(4, dtype=np.float32)
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
        q = tf.quaternion_from_matrix(pose)

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
                        x_rand, y_rand = 0, 0
                        self._first_time = False
                    else:
                        x_rand = (random.random()-0.5)*2*0.05
                        y_rand = (random.random()-0.5)*2*0.05
                    pose = np.eye(4, dtype=np.float32)
                    pose[0, 3] = 0.25 + x_rand
                    pose[1, 3] = 0.0 + y_rand
                    pose[2, 3] = 0.0
                    self.publish_pose(pose)
                    
                    self._ready_to_publish = False
                            
                time.sleep(0.1)  # Add a small delay to control the loop speed
            
            
    def ack_handler(self, channel, data):
        ack = ack_t.decode(data)
        print(f"Publisher got Ack for msg #{ack.msg_id} (sent at {ack.timestamp})")
        self._ready_to_publish = True
        

if __name__ == '__main__':
    pose_publisher = PosePublisher('/B1_pose', wait_key=True)
    pose_publisher.run()