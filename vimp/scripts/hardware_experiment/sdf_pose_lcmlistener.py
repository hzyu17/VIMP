# Generate SDF for the hardware experiment
from vimp.thirdparty.sensor3D_tools import SignedDistanceField, SignedDistanceField3D
import numpy as np

import lcm
from vimp.thirdparty.sensor3D_tools.lcm import pose_t
import threading
import tf.transformations as tft
from read_defaul_poses import *

from vimp.scripts.hardware_experiment.moveit_baselines import read_plan_json_to_numpy
from resampling import read_forwardkinematic_from_config, collision_checking

from pathlib import Path
import os

import time
def lcm_thread(lc):
    while True:
        lc.handle()
        time.sleep(0.01)
        

class SDFUpdaterListener:
    def __init__(self, config_file, output_file='disturbed_results.yaml'):
        
        (self._body_box, self._body_topic, 
         self._rel_poses, self._sizes, self._body_default_pose) = load_body_frames_config(config_file)
        
        self._grid = default_occmap_from_yaml(config_file)
        
        # obstacle -> nonzero elements in the map
        self._body_occ = {} # body frame to an occupancy map
        self._T_world_body = {} # body frame to a matrix pose
        
        self._T_cam = read_cam_pose(config_file)
        
        self.output_file = output_file
        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        self._output_yaml = open(self.output_file, 'a')
        
        # Forward kinematics and baseline trajectories
        
        with open(config_file,"r") as f:
            cfg = yaml.safe_load(f)
        planning_cfgfile = Path(cfg["Planning"]["config_file"])
        
        baseline_IDs = cfg["Planning"]["Baselines"]["planner_ID"]
        
        self._baslines = {}
        for id in baseline_IDs:
            filename = f"{id}_plan_trj.yaml"
            baseline_file = Path(__file__).parent / "Data" / filename
            
            if not baseline_file.is_file():
                continue
            
            self._baslines[id] = read_plan_json_to_numpy(baseline_file)
        
        
        self._fk = read_forwardkinematic_from_config(planning_cfgfile)
        
        for body, pose in self._body_default_pose.items():
            grid = default_occmap_from_yaml(config_file)
            
            # construct the body frame's pose
            T = np.eye(4)
            # translation
            T[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])
            # rotation from quaternion
            quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            T[:3, :3] = tft.quaternion_matrix(quat)[:3, :3] # From quaternion to rotation matrix
            self._T_world_body[body] = T
            
            # Insert obstacle
            bname = self._body_box[body]
            box_rel_pose = self._rel_poses[bname]
            box_size = self._sizes[bname]
            
            # construct the box's pose in the body frame
            C = np.eye(4)
            C[:3, 3] = np.array([box_rel_pose.position.x, box_rel_pose.position.y, box_rel_pose.position.z])
            # Default assumed to be parallel to the axis
            quat = np.array([0, 0, 0, 1])
            C[:3, :3] = tft.quaternion_matrix(quat)[:3, :3]
            
            T_w_box = self._T_cam @ T @ C
            center_w = T_w_box[:3, 3]
            
            cs = self._grid.cell_size
            center_idx = np.rint((center_w - self._grid.origin()) / cs).astype(int)
            
            size_vox = np.ceil(np.array(box_size, float) / cs).astype(int)
            size_vox = (size_vox // 2) * 2 + 1

            grid.add_obstacle(center_idx, size_vox)
            
            self._body_occ[body] = grid
    
    
    def visualize_boxes(self):
        for body, grid in self._body_occ.items():
            grid.visualize()
            
    
    def body_pose_handler(self, channel, data):
        msg = pose_t.decode(data)
        print("   Received pcd message on channel \"%s\"" % channel)
        print("   frame name  = %s" % str(msg.frame_name))
        print("   timestamp   = %s" % str(msg.timestamp))
        print("   pose    = %s" % str(msg.pose))
        
        subgrid = self._body_occ[msg.frame_name]
        old_pose = self._T_world_body[msg.frame_name]
        transform_pose = msg.pose @ np.linalg.inv(old_pose)
        self._T_world_body[msg.frame_name] = msg.pose
        
        # Apply the pose transformation
        subgrid.transform(transform_pose, visualize=False)
        
        self._body_occ[msg.frame_name] = subgrid
        
        # Generate new sdf after the pose transformation
        field3D = SignedDistanceField3D.generate_field3D(subgrid.map.detach().numpy(), cell_size=subgrid.cell_size)
        sdf = SignedDistanceField(subgrid.origin(), subgrid.cell_size,
                                  field3D.shape[0], field3D.shape[1], field3D.shape[2])
        for z in range(field3D.shape[2]):
            sdf.initFieldData(z, field3D[:,:,z])
            
        print("==== SDF created! ====")
            
        for id, trj in self._baslines.items():
            min_dist_baseline = collision_checking(sdf, self._fk, trj)
            
            print("min_dist baseline: ", min_dist_baseline)
            
            # Record the poses in the json file
            entry = {
                "timestamp": msg.timestamp,
                "planner_id": id,
                "body_frame_pose": np.array(msg.pose).tolist(),
                "min_dist_baseline":
                    min_dist_baseline.tolist()
            }
            print("Saving baseline collision checking result!")
            # write exactly one JSON object per line
            yaml.safe_dump(entry, self._output_yaml)
            # add document separator
            self._output_yaml.write('---\n')
            self._output_yaml.flush()
        
        # save_path = str(Path(__file__).parent / "Data" / "RandomSDF.bin")
        # sdf.saveSDF(save_path)
        
        # print("Saved new SDF!")
        

    def run(self):
        lc = lcm.LCM()
        for name, topic in self._body_topic.items():
            lc.subscribe(topic, self.body_pose_handler)
            
        # start LCM in background
        threading.Thread(target=lambda: lcm_thread(lc), daemon=True).start()
               
        try:
            while True:
                time.sleep(0.02)       
        except KeyboardInterrupt:
            pass
    
    

if __name__ == '__main__':
    
    
    cfg_path = Path(__file__).parent / "config" / "config.yaml"
    output_file = Path(__file__).parent / "Data" / "disturbed_results.yaml"
    
    listener = SDFUpdaterListener(cfg_path, output_file)
    
    # listener.visualize_boxes()
    
    listener.run()
    