import os, sys
this_dir = os.path.dirname(os.path.abspath(__file__))
vimp_dir = os.path.dirname(os.path.dirname(this_dir))
thirdparty_dir = vimp_dir + "/3rdparty"

if thirdparty_dir not in sys.path:            
    sys.path.insert(0, thirdparty_dir)

from AprilTag.scripts.apriltag_image import apriltag_image, apriltag_image2pose

import json
from pathlib import Path
import numpy as np
import cv2


if __name__ == '__main__':
    
    # ---------------------
    #  Camera Parameters
    # ---------------------
    json_path = Path(this_dir+"/UTF-8realsense_high_d435_no_table_tuned_p2048_w_icp.json")          # change the filename if needed
    with json_path.open("r") as f:
        cfg = json.load(f) 
    K = np.asarray(cfg["K"], dtype=np.float32)
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    d435_params = (fx, fy, cx, cy)
    
    
    # -------------------------
    #   Input is a .jpg image
    # -------------------------
    input_images = [this_dir+'/single_tag.jpg']
    output_images = False
    display_images = True
    detection_window_name = 'AprilTag'
    
    apriltag_image(input_images=input_images,
                   output_images=output_images,
                   display_images=display_images,
                   detection_window_name=detection_window_name,
                   camera_params=d435_params)
    
    # ----------------------------------
    #   Input is an image numpy array
    # ----------------------------------
    img = cv2.imread(this_dir+'/single_tag.jpg')    
    img_np = np.asarray(img)
    
    pose = apriltag_image2pose(input_img_numpy=img_np,
                                display_images=display_images,
                                detection_window_name=detection_window_name,
                                camera_params=d435_params)
    
    print("pose: ", pose)
    