# Generate SDF for the hardware experiment
from vimp.thirdparty.sensor3D_tools import SignedDistanceField, SignedDistanceField3D
import numpy as np

import lcm
from ack_lcm.acknowledgment.ack_t import ack_t
from vimp.thirdparty.sensor3D_tools.lcm import pose_t
import threading
import tf.transformations as tft
from read_default_poses import *
from pose_helpers import pose_to_matrix, matrix_to_pose

from vimp.scripts.hardware_experiment.moveit_baselines import read_plan_json_to_numpy
from resampling import read_forwardkinematic_from_config, collision_checking, resample_block_trajectory

from pathlib import Path
import os

import time
def lcm_thread(lc):
    while True:
        lc.handle()
        time.sleep(0.01)


this_dir = os.path.dirname(os.path.abspath(__file__))
sdf_dir = this_dir + "/../../scripts/hardware_experiment/Data/FrankaHardware_cereal.bin"

# # For debug SDF
# sdf_original = SignedDistanceField()
# sdf_original.loadSDF(sdf_dir)
        

class SDFUpdaterListener:
    def __init__(self, config_file, output_file='disturbed_results_111.yaml', compute_distance=False, send_ack=True):
        self.compute_distance = compute_distance  # If True, compute and compare the distances of the baseline and GVIMP
        self.send_ack = send_ack

        (self._body_box, self._body_topic, 
         self._rel_poses, self._sizes, self._body_default_pose, self._world_sizes) = load_body_frames_config(config_file)
        
        self._grid = default_occmap_from_yaml(config_file)
        
        # obstacle -> nonzero elements in the map
        self._body_occ = {} # body frame to an occupancy map
        self._T_cam_body = {} # body frame to a matrix pose
        self._box_center = {} # box name to its center in world coordinate
        
        self._T_cam = read_cam_pose(config_file)
        
        self.output_file = output_file
        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        self._output_yaml = open(self.output_file, 'a')
        
        # Forward kinematics and baseline trajectories
        with open(config_file,"r") as f:
            cfg = yaml.safe_load(f)
        planning_cfgfile = Path(cfg["Planning"]["config_file"])
        
        if self.compute_distance:
            baseline_IDs = cfg["Baselines"]["planner_ID"]
            
            self._baslines = {}
            for id in baseline_IDs:
                filename = f"{id}_plan_trj.yaml"
                baseline_file = Path(__file__).parent / "Data" / filename
                
                if not baseline_file.is_file():
                    print("There is no baseline trajectory file!")
                    continue
                
                self._baslines[id] = read_plan_json_to_numpy(baseline_file)
            
            self._fk = read_forwardkinematic_from_config(planning_cfgfile)

        # Build the subgrids of the obstacles
        for body, pose in self._body_default_pose.items():
            grid = default_occmap_from_yaml(config_file)
            
            # construct the body frame's pose
            T = pose_to_matrix(pose)
            self._T_cam_body[body] = T

            T_w_body = self._T_cam @ T
            
            # Insert obstacle
            for bname in self._body_box[body]:
                box_rel_pose = self._rel_poses[bname]
                # box_size = self._world_sizes[bname]
                box_size_body_frame = self._sizes[bname]
                
                # # construct the box's pose in the body frame
                # C = np.eye(4)
                # C[:3, 3] = np.array([box_rel_pose.position.x, box_rel_pose.position.y, box_rel_pose.position.z])
                # # Default assumed to be parallel to the axis
                # C[:3, :3] = np.eye(3)
                
                # T_w_box = self._T_cam @ T @ C
                # print("T_w_box: ", T_w_box)

                # center_w = T_w_box[:3, 3]
                # self._box_center[bname] = center_w
                
                # cs = self._grid.cell_size
                # center_idx = np.rint((center_w - self._grid.origin()) / cs).astype(int)
                
                # size_vox = np.ceil(np.array(box_size, float) / cs).astype(int)
                # size_vox = (size_vox // 2) * 2

                # grid.add_obstacle(center_idx, size_vox)

                cs = self._grid.cell_size
                # Use the relative position of the box in the body frame as the center
                center = np.array([box_rel_pose.position.x, box_rel_pose.position.y, box_rel_pose.position.z])
                center_idx = np.rint((center - self._grid.origin()) / cs).astype(int)

                size_vox = np.ceil(np.array(box_size_body_frame, float) / cs).astype(int)
                size_vox = (size_vox // 2) * 2

                grid.add_obstacle(center_idx, size_vox)

            grid.transform(T_w_body, visualize=True)
            self._body_occ[body] = grid
            # grid.visualize()
    
    
    def visualize_boxes(self):
        for body, grid in self._body_occ.items():
            grid.visualize()
            
    
    def body_pose_handler(self, channel, data):
        msg = pose_t.decode(data)
        print("   Received pcd message on channel \"%s\"" % channel)
        print("   frame name  = %s" % str(msg.frame_name))
        print("   timestamp   = %s" % str(msg.timestamp))
        print("   pose    = %s" % str(msg.pose))
        
        # Clean the old occupancy map
        subgrid = self._body_occ[msg.frame_name]
        subgrid.clear()

        # Create a new occupancy map and add the original obstacle
        bname = self._body_box[msg.frame_name]
        center_obs = self._box_center[bname]
        box_size = self._world_sizes[bname]

        cs = self._grid.cell_size
        center_idx = np.rint((center_obs - self._grid.origin()) / cs).astype(int)
        size_vox = np.ceil(np.array(box_size, float) / cs).astype(int)
        size_vox = (size_vox // 2) * 2
        subgrid.add_obstacle(center_idx, size_vox)

        # Transform the occupancy map to the new pose
        old_pose = self._T_cam_body[msg.frame_name]
        transform_pose = self._T_cam @ msg.pose @ np.linalg.inv(old_pose) @ np.linalg.inv(self._T_cam)

        # Apply the pose transformation
        subgrid.transform(transform_pose, visualize=False)
        
        self._body_occ[msg.frame_name] = subgrid
        
        # Generate new sdf after the pose transformation
        field3D = SignedDistanceField3D.generate_field3D(subgrid.map.detach().numpy(), cell_size=subgrid.cell_size)
        sdf = SignedDistanceField(subgrid.origin(), subgrid.cell_size,
                                  field3D.shape[0], field3D.shape[1], field3D.shape[2])
        for z in range(field3D.shape[2]):
            sdf.initFieldData(z, field3D[:,:,z].T)
            
        # pt = np.zeros(3)
        # print("Test signed distance of a new sdf: ", sdf.getSignedDistance(pt))
        # print("Test signed distance of the original sdf: ", sdf_original.getSignedDistance(pt))
        
        print("==== SDF created! ====")
        
        if self.compute_distance:
            for id, trj in self._baslines.items():
                
                # print("Checking collisions for the baseline planner: ", id)
                min_dist_baseline = collision_checking(sdf, self._fk, trj)
                
                min_dist_name = "min_dist"+"_"+id
                min_dist = float(np.min(min_dist_baseline))
                # Record the poses in the json file
                entry = {
                    msg.timestamp: {
                            "planner_id":         id,
                            "body_frame_pose":    np.array(msg.pose).tolist(),
                            min_dist_name:       min_dist
                        }
                }
                # print("Saving baseline collision checking result!")
                # write exactly one JSON object per line
                yaml.safe_dump(entry, self._output_yaml)
                # add document separator
                self._output_yaml.write('---\n')
                self._output_yaml.flush()
                
                # print("Added new data to the result file!")
            
            print("==== Baseline collision checking finished! ====")
            # Resample the trajectory
            experiment_config = this_dir + "/config/config.yaml"
            path_to_yaml = Path(experiment_config)

            with path_to_yaml.open("r") as f:
                cfg = yaml.safe_load(f)
            
            planning_cfg_path = Path(cfg["Planning"]["config_file"])

            vimp_dir = os.path.dirname(os.path.dirname(this_dir))
            result_dir = Path(vimp_dir + "/" + cfg["Planning"]["saving_prefix"])

            min_dist_planning, min_dist_resample = resample_block_trajectory(planning_cfg_path, result_dir, sdf, cfg["Sampling"])

            entry = {
                msg.timestamp: {
                    "planner_id":         "GVIMP",
                    "body_frame_pose":    np.array(msg.pose).tolist(),
                    "min_dist_plan":      float(min_dist_planning)
                }
            }

            # write exactly one JSON object per line
            yaml.safe_dump(entry, self._output_yaml)
            # add document separator
            self._output_yaml.write('---\n')
            self._output_yaml.flush()

            entry = {
                msg.timestamp: {
                    "planner_id":         "GVIMP_Resample",
                    "body_frame_pose":    np.array(msg.pose).tolist(),
                    "min_dist_resample":  float(min_dist_resample)
                }
            }

            print("Saving baseline collision checking result!")
            # write exactly one JSON object per line
            yaml.safe_dump(entry, self._output_yaml)
            # add document separator
            self._output_yaml.write('---\n')
            self._output_yaml.flush()
        
        if self.send_ack:
            # create acknowledgment
            ack = ack_t()
            ack.timestamp   = int(time.time() * 1e6)  # µs
            ack.src_channel = channel

            # publish it back on an ACK channel
            lc = lcm.LCM()
            lc.publish("ACK_"+channel, ack.encode())
            print("Published ack message on channel \"%s\"" % ("ACK_" + channel))

        # save_path = str(Path(__file__).parent / "Data" / "FrankaHardware_cereal.bin")
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
    output_file = Path(__file__).parent / "Data" / "hardware_results.yaml"
    # output_file = Path(__file__).parent / "Data" / "poses_result.yaml"
    
    listener = SDFUpdaterListener(cfg_path, output_file, compute_distance=False, send_ack=False)
    
    # listener.visualize_boxes()
    
    listener.run()
    