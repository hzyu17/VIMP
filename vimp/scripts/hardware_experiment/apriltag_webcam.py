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


cap = cv2.VideoCapture(0)         
if not cap.isOpened():
    raise IOError("Cannot open webcam")

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
    

cap.release()
cv2.destroyAllWindows()