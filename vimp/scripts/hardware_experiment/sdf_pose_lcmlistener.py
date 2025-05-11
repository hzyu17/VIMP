# Generate SDF for the hardware experiment
from vimp.thirdparty.sensor3D_tools import SignedDistanceField, SignedDistanceField3D, OccpuancyGrid
import yaml
import numpy as np
from bodyframe_pose_listener import load_body_frames_config
from pathlib import Path

import lcm
from vimp.thirdparty.sensor3D_tools.lcm import pose_t
import threading

import tf.transformations as tft

# ----------
#  Helpers
# ----------

def default_occmap_from_yaml(yaml_file):
    with open(yaml_file,"r") as f:
        cfg = yaml.safe_load(f)
    
    # Create 3D field    
    field = cfg.get("Field", [])
    field_origin = field["field_origin"]
    field_size   = field["field_size"]
    cell_size    = field["cell_size"]
    return OccpuancyGrid(field_size[0], field_size[1], field_size[2], cell_size, origin=field_origin)


class SDFUpdaterListener:
    def __init__(self, config_file):
        
        (self._body_box, self._body_topic, 
         self._rel_poses, self._sizes, self._body_default_pose) = load_body_frames_config(config_file)
        
        self._grid = default_occmap_from_yaml(config_file)
        
        # obstacle -> nonzero elements in the map
        self._body_occ = {} # body frame to an occupancy map
        self._T_world_body = {} # body frame to a matrix pose
        
        for body, pose in self._body_default_pose.items():
            grid = default_occmap_from_yaml(config_file)
            
            # construct the body frame's pose
            T = np.eye(4)
            # translation
            T[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])
            # rotation from quaternion
            quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            T[:3, :3] = tft.quaternion_matrix(quat)[:3, :3]
            self._T_world_body[body] = T
            
            # Insert obstacle
            bname = self._body_box[body]
            box_rel_pose = self._rel_poses[bname]
            box_size = self._sizes[bname]
            
            C = np.eye(4)
            C[:3, 3] = np.array([box_rel_pose.position.x, box_rel_pose.position.y, box_rel_pose.position.z])
            quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            C[:3, :3] = tft.quaternion_matrix(quat)[:3, :3]
            
            T_w_box = T @ C
            center_w = T_w_box[:3, 3]
            
            cs = self._grid.cell_size
            idx = np.rint((center_w - self._grid.origin()) / cs).astype(int)
            
            size_vox = np.ceil(np.array(box_size, float) / cs).astype(int)
            size_vox = (size_vox // 2) * 2 + 1

            grid.add_obstacle(idx, size_vox)
            
            self._body_occ[body] = grid
    
    
    def visualize_boxes(self):
        for body, grid in self._body_occ.items():
            grid.visualize()
    


def body_pose_handler(channel, data):
    msg = pose_t.decode(data)
    print("   Received pcd message on channel \"%s\"" % channel)
    print("   frame name  = %s" % str(msg.frame_name))
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   pose    = %s" % str(msg.pose))

    
    
    # obs = []
    
    # for center, size in obs:
    #     grid.add_obstacle(np.array(center), np.array(size))

    # # 2) Access the body frame list to get the positions of the obstacles
    # body_frames = cfg.get("Body_frames", [])
    
    # # 3) Default box obstacle poses: assume to have parallel edges as the axis
    # for bf in body_frames:
    #     position       = bf["position"]      
    #     orientation    = bf["orientation"]   
        
    #     box = bf["box"]
    #     box_center     = box["center"]        
    #     box_orientation= box["orientation"]   
    #     box_size       = box["size"]          
    #     cell_size      = box["cell_size"]
    
    # field3d = SignedDistanceField3D.generate_field3D(grid.map.detach().numpy(), grid.cell_size)
    
    # origin = np.array([
    #     grid.origin_x,
    #     grid.origin_y,
    #     grid.origin_z
    # ], dtype=np.float64)

    # sdf = SignedDistanceField(origin, grid.cell_size,
    #                             field3d.shape[0], field3d.shape[1], field3d.shape[2])
    # for z in range(field3d.shape[2]):
    #     sdf.initFieldData(z, field3d[:,:,z])
        
    
import time
def lcm_thread(lc):
    while True:
        lc.handle()
        time.sleep(0.01)
        

if __name__ == '__main__':
    
    
    cfg_path = Path(__file__).parent / "config" / "config.yaml"
    
    listener = SDFUpdaterListener(cfg_path)
    
    listener.visualize_boxes()
    
    
    
    # body_box, body_topic, rel_poses, sizes, body_default_pose = load_body_frames_config(cfg_path)

    # lc = lcm.LCM()
    # for name, topic in body_topic.items():
    #     lc.subscribe(topic, body_pose_handler)
        
    # # start LCM in background
    # threading.Thread(target=lambda: lcm_thread(lc), daemon=True).start()
    
    # lock = threading.Lock()
    
    # try:
    #     while True:
    #         time.sleep(0.02)       
    # except KeyboardInterrupt:
    #     pass