# Read and show the video stream from the webcam using OpenCV
# Detect the april tag in the video stream

import os, sys
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))
thirdparty_dir = vimp_dir + "/3rdparty"

if thirdparty_dir not in sys.path:            
    sys.path.insert(0, thirdparty_dir)

from AprilTag.scripts.apriltag_image import apriltag_image, apriltag_image2pose
from sensor3D_tools.lcm.pcd_lcm_sender import construct_poselcm_msg, publish_lcm_msg

import cv2
import time

def publish_apriltag_pose(topic_name='/apriltag_pose'):
    """
    This function captures video from the webcam, detects AprilTags in the frames,
    and publishes the pose of the detected AprilTags to LCM and ROS.
    """
    
    # Initialize webcam

    cap = cv2.VideoCapture(0)         
    if not cap.isOpened():
        raise IOError("Cannot open webcam")


    publish_ros = True

    while True:
        ret, frame = cap.read()        
        if not ret:
            break

        cv2.imshow("live", frame)     
        if cv2.waitKey(1) == 27:      
            break
        
        pose = apriltag_image2pose(input_img_numpy=frame,
                                    display_images=False,
                                    detection_window_name='AprilTag',
                                    camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909))
        
        
        print("pose: ", pose)
        
        if pose is not None:
            msg = construct_poselcm_msg(pose)
            publish_lcm_msg(msg)
            
            if publish_ros:
                import rospy
                from geometry_msgs.msg import PoseStamped
                import tf.transformations as tf
                rospy.init_node('apriltag_webcam', anonymous=True)
                pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
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
                pub.publish(pose_msg)
                print("Published pose to ROS topic /apriltag_pose")
            
        time.sleep(0.1)  # Add a small delay to control the loop speed
        

    cap.release()
    cv2.destroyAllWindows()
    
    
if __name__ == "__main__":
    publish_apriltag_pose(topic_name='/B1_pose')