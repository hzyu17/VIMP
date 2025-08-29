from D435_receive import CameraRedisPubInterface, CameraRedisSubInterface

# Read and show the video stream from the webcam using OpenCV
# Detect the april tag in the video stream

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

import rospy, select
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf
import numpy as np
import yaml
from pathlib import Path

from scipy.spatial.transform import Rotation as R

import pyrealsense2 as rs
import numpy as np
import cv2
from pupil_apriltags import Detector

# Create pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

class CameraAprilTagReader:
    def __init__(self, config_file, buffer_size, topic_name, wait_key=False):
        self._body_topic, self._tag_body = read_body_tags(config_file)
        self._buffers = {
            name: deque(maxlen=buffer_size)
            for name in self._body_topic
        }
        self._lock = threading.Lock()
        self._wait_key = wait_key
        self._topic_name = topic_name
        self._lc = lcm.LCM()
        self._cam_receiver = CameraRedisSubInterface()

        self._output = []
        self._count_output = 0
        output_file = Path(__file__).parent / "Data" / "box_poses_d435_disturb.yaml"
        self._output_yaml = open(output_file, 'a')

        self._dector = Detector(
            families='tag25h9',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        
        rospy.init_node('apriltag_D435', anonymous=True)
        self._pubs = {frame_id: rospy.Publisher(topic, PoseStamped, queue_size=10) 
                      for frame_id, topic in self._body_topic.items()}
        
        rospy.loginfo(f"[{rospy.get_name()}] listening to {topic_name},"
                  " hit ENTER to pop & publish on {output_topic}")
        
        import json
        json_path = Path(__file__).parent / "UTF-8realsense_high_d435_no_table_tuned_p2048_w_icp.json"    # change the filename if needed
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
        # json_str = self._cam_receiver.info_redis.get(f"{self._cam_receiver.camera_name}::last_img_info")
        # if json_str is not None:
        #     for _ in range(5):
        #         self._cam_receiver.save_img(flag=True)
        #         time.sleep(0.02)

        # raw_frame = self._cam_receiver.get_img()
        # rgb_img = cv2.cvtColor(np.array(raw_frame["color"]), cv2.COLOR_BGR2RGB)

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # depth_frame = frames.get_depth_frame()

        # Convert images to numpy arrays
        rgb_img = np.asanyarray(color_frame.get_data())
        # depth_image = np.asanyarray(depth_frame.get_data())

        cv2.imshow("live", rgb_img)     
        if cv2.waitKey(1) == 27:      
            return False
        
        poses = apriltag_image2poses(detector=self._dector,
                                    input_img_numpy=rgb_img,
                                    display_images=False,
                                    detection_window_name='AprilTag',
                                    tag_size=self._tag_size,
                                    camera_params=self._d435_params)
        
        # print(poses)
        
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

            if save_pose and self._count_output < 100:
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
    cfg_path = Path(__file__).parent / "config" / "config.yaml"
    reader = CameraAprilTagReader(cfg_path, buffer_size=10, topic_name='/pose_publish', wait_key=True)
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