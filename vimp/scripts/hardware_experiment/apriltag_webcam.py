# Read and show the video stream from the webcam using OpenCV
# Detect the april tag in the video stream

from pathlib import Path
import sys

from vimp.thirdparty.AprilTag.scripts.apriltag_image import apriltag_image2poses
from vimp.thirdparty.sensor3D_tools.lcm.pcd_lcm_sender import construct_poselcm_msg, publish_lcm_msg
from read_default_poses import read_body_tags
from pose_helpers import matrix_to_pose, pose_to_dict

import cv2
import time
import threading
import lcm
from collections import deque

import rospy, select, yaml
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf


class CameraAprilTagReader:
    def __init__(self, 
                 config_file: str, 
                 buffer_size: int, 
                 topic_name, 
                 wait_key=False):

        self._body_topic, self._tag_body = read_body_tags(config_file) # {body_frame: topic}, {tag_id: body_frame}
        self._buffers = {
            name: deque(maxlen=buffer_size)
            for name in self._body_topic
        }
        # self._buffer = deque(maxlen=buffer_size)
        self._lock = threading.Lock()
        self._wait_key = wait_key
        self._topic_name = topic_name
        self._lc = lcm.LCM()
        
        self._count_output = 0
        output_file = Path(__file__).parent / "Data" / "box_poses_webcam.yaml"
        self._output_yaml = open(output_file, 'a')

        rospy.init_node('apriltag_webcam', anonymous=True)
        self._pubs = {frame_id: rospy.Publisher(topic, PoseStamped, queue_size=10) 
                      for frame_id, topic in self._body_topic.items()}
        
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
        
        poses = apriltag_image2poses(input_img_numpy=frame,
                                    display_images=False,
                                    detection_window_name='AprilTag',
                                    camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909))

        if poses is not None:
            with self._lock:
                for tid, pose in poses.items():
                    name = self._tag_body.get(tid)
                    if name in self._buffers:
                        self._buffers[name].append(pose)
        
        return True


    def publish_pose(self):
        with self._lock:
            # save pose
            save_pose = True
            timestamp = int(rospy.Time.now().to_sec())
            entry = {"timestamp": timestamp}
            print(self._buffers)

            for frame_id, buffer in self._buffers.items():
                if not buffer:
                    continue

                pose = buffer.popleft()
                msg = construct_poselcm_msg(pose, name=frame_id)
                publish_lcm_msg(msg, topic=self._body_topic[frame_id])
                
                # Publish ROS message for RViz
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "camera_frame"
                pose_msg.pose = matrix_to_pose(pose)

                self._pubs[frame_id].publish(pose_msg)
                entry[frame_id] = pose_to_dict(pose_msg.pose)
                print("Published pose to ROS topic ", self._body_topic[frame_id])
            
            info = {
                    "timestamp": int(time.time()),
                    "status": "Done",
                }
            done_msg = yaml.safe_dump(info).encode('utf-8')
            self._lc.publish(self._topic_name, done_msg)

            if save_pose and self._count_output < 10:
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
            
        self._cap.release()
        cv2.destroyAllWindows()
    
    
if __name__ == "__main__":
    cfg_path = Path(__file__).parent / "config" / "config.yaml"
    reader = CameraAprilTagReader(cfg_path, buffer_size=10, topic_name='/pose_publish', wait_key=True)
    reader.run()
    
    