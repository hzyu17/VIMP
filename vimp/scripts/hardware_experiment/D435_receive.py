import redis
import time
import json
import struct
import numpy as np
import cv2
from vimp.thirdparty.AprilTag.scripts.apriltag_image import apriltag_image2pose
from vimp.thirdparty.sensor3D_tools.lcm.pcd_lcm_sender import construct_poselcm_msg, publish_lcm_msg


import rospy, select
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf


class CameraRedisPubInterface():
    """
    This is the Python Interface for writing img info / img data buffer to redis server.
    """
    def __init__(self, redis_host="172.16.0.1", redis_port=6379, camera_id=0):
        self.redis_host = redis_host
        self.redis_port = redis_port
        self.info_redis = redis.StrictRedis(
            host=self.redis_host,
            port=self.redis_port,
            charset="utf-8",
            decode_responses=True)

        self.img_redis = redis.StrictRedis(
            host=self.redis_host,
            port=self.redis_port,
            charset="utf-8",
            decode_responses=False)

        self.camera_name = f"camera_{camera_id}"
        self.camera_id = camera_id
        
        for key in self.info_redis.scan_iter(f"{self.camera_name}*"):
            self.info_redis.delete(key)

        self.save_img = False


    def set_img_info(self, img_info):
        """
        Args:
           img_info (dict): dictionary of image information
        """
        json_img_color_str = json.dumps(img_info)
        self.info_redis.set(f"{self.camera_name}::last_img_info", json_img_color_str)

    def set_img_buffer(self,
                       imgs):

        if "color" in imgs:
            img_color = imgs["color"]
            h, w, c = imgs["color"].shape
            shape = struct.pack(">III", h, w, c)
            encoded_color = shape + imgs["color"].tobytes()
            self.img_redis.set(f"{self.camera_name}::last_img_color", encoded_color)
        if "depth" in imgs:
            h, w = imgs["depth"].shape
            shape = struct.pack(">II", h, w)
            encoded_depth = shape + imgs["depth"].tobytes()
            self.img_redis.set(f"{self.camera_name}::last_img_depth", encoded_depth)
        if "points" in imgs:
            h, w = imgs["points"].shape[:2]
            shape = struct.pack(">II", h, w)
            encoded_points = shape + imgs["points"].tobytes()
            self.img_redis.set(f"{self.camera_name}::last_img_points", encoded_points)


    def get_save_img_info(self):
        self.save_img = self.info_redis.get(f"{self.camera_name}::save")
        return bool(self.save_img)

    @property
    def finished(self):
        return self.info_redis.get(f"{self.camera_name}::finish")
    
            
class CameraRedisSubInterface():
    """
    This is the Python Interface for getting image name from redis server. You need to make sure that camera processes are running and images are published to redis server.
    """
    def __init__(self, redis_host="172.16.0.1", redis_port=6379, camera_id=0, use_color=True, use_depth=False):
        self.redis_host = redis_host
        self.redis_port = redis_port
        self.info_redis = redis.StrictRedis(
            host=self.redis_host,
            port=self.redis_port,
            charset="utf-8",
            decode_responses=True)

        self.img_redis = redis.StrictRedis(
            host=self.redis_host,
            port=self.redis_port,
            charset="utf-8",
            decode_responses=False)
        
        self.camera_name = f"camera_{camera_id}"
        self.camera_id = camera_id
        for key in self.info_redis.scan_iter(f"{self.camera_name}*"):
            self.info_redis.delete(key)

        self.use_color = use_color
        self.use_depth = use_depth

        self.camera_type = None

    def start(self, timeout=5):

        start_time = time.time()
        end_time = start_time

        while True:
            info = self.get_img_info()
            if info is not None:
                break
        
        while True:
            json_str = self.info_redis.get(f"{self.camera_name}::last_img_info")
            if json_str is not None:
                for _ in range(5):
                    self.save_img(flag=True)
                    time.sleep(0.02)

                raw_frame = self.get_img()
                rgb_img = cv2.cvtColor(np.array(raw_frame["color"]), cv2.COLOR_BGR2RGB)
                cv2.imshow("D435 Color (via V4L2)", rgb_img)     
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    break

                img_info = self.get_img_info()
                self.camera_type = img_info["camera_type"]

        raise ValueError


    def stop(self):
        for _ in range(5):
            self.save_img(flag=False)
            time.sleep(0.02)

    def save_img(self, flag=False):
        if flag:
            self.info_redis.set(f"{self.camera_name}::save", 1)
        else:
            self.info_redis.delete(f"{self.camera_name}::save")
        
    def get_img_info(self):
        img_info = self.info_redis.get(f"{self.camera_name}::last_img_info")
        if img_info is not None:
            img_info = json.loads(img_info)
        return img_info

    def get_img(self):
        # TODO(Yifeng): Test if redis is faster or reading image from file is faster.

        img_color = None
        img_depth = None
        img_points = None
        if self.use_color:
            color_buffer = self.img_redis.get(f"{self.camera_name}::last_img_color")
            h, w, c = struct.unpack(">III", color_buffer[:12])
            img_color = np.frombuffer(color_buffer[12:], dtype=np.uint8).reshape(h, w, c)

        if self.use_depth:
            depth_buffer = self.img_redis.get(f"{self.camera_name}::last_img_depth")
            h, w = struct.unpack(">II", depth_buffer[:8])
            img_depth = np.frombuffer(depth_buffer[8:], dtype=np.uint16).reshape(h, w)

            # points_buffer = self.img_redis.get(f"{self.camera_name}::last_img_points")
            # img_points = np.frombuffer(points_buffer[8:], dtype=np.uint16).reshape(h, w, 3)

        return {"color": img_color,
                "depth": img_depth,
                # "points": img_points
                }

    def finish(self):
        for _ in range(10):
            self.save_img(flag=False)
            time.sleep(0.02)
        self.info_redis.set(f"{self.camera_name}::finish", 1)
        

    def close(self):
        self.finish()
