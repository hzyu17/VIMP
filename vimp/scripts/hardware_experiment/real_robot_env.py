import gc
import json
import numpy as np
import time
import sys, os
from termcolor import cprint, colored
import cv2
import imageio
from PIL import Image
import os.path as osp
import open3d as o3d
import h5py
import tqdm

from robosuite.utils.transform_utils import mat2quat, quat2axisangle, quat2mat, axisangle2quat
import diffusion_policy_3d.env_runner.deoxys_utils.transform_utils as deoxys_transform
import pathlib
from diffusion_policy_3d.env_runner.yourdfpy_ik import YourdfIk
from diffusion_policy_3d.env_runner.base_runner import BaseRunner
from diffusion_policy_3d.gym_util.multistep_wrapper import MultiStepJointWrapper, MultiStepWrapper
from diffusion_policy_3d.gym_util.video_recording_wrapper import SimpleVideoRecordingWrapper
from datetime import datetime, date
import torch
from diffusion_policy_3d.common.pytorch_util import dict_apply
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from diffusion_policy_3d.optimus_util.process_pts import point_cloud_sampling

import pdb

try:
    # sys.path.append("/home/rl2-ws1/robot_ws/sim2real/3d_diffusion/")
    # from realworld_rollout.perception.cam_utils.utils.apriltag_detector import AprilTagDetector
    # sys.path.append('/home/rl2-ws1/robot_ws/sim2real/optimus_transfer')

    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    from RobotTeleop.utils import Rate, RateMeasure, Timers
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

    proj_root = "/home/rl2-ws1/robot_ws/robot_infra/gprs"
    sys.path.insert(0, proj_root)
    vutil_root = "/home/rl2-ws1/robot_ws/rpl_vision_utils"
    sys.path.insert(0, vutil_root)
    from gprs import config_root
    from gprs.franka_interface import FrankaInterface
    from gprs.utils import YamlConfig

except Exception as e:
    cprint("Got error while importing real robot infra!", "red")
    cprint(e, "red")


try:
    from diffusion_policy_3d.env_runner.robosuite_envs import \
        MujocoSimpleWrapperRobosuite as EnvWrapper
    from diffusion_policy_3d.env_runner.robosuite_envs import MujocoRobotModel
    from diffusion_policy_3d.env_runner.camera_redis_interface import CameraRedisSubInterface
except Exception as e:
    cprint("Got error while importing dp3 infra!", "red")
    cprint(e, "red")

def reset_robot_to(robot_interface, reset_joint_positions):
    controller_type = "JOINT_POSITION"
    controller_cfg = YamlConfig(config_root + f"/joint-impedance-controller.yml").as_easydict()
    action = reset_joint_positions + [-1]

    while True:
        if len(robot_interface._state_buffer) > 0:
            if (
                    np.max(
                        np.abs(
                            np.array(robot_interface._state_buffer[-1].q)
                            - np.array(reset_joint_positions)
                        )
                    )
                    < 1e-3
            ):
                break
        robot_interface.control(
            control_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )

def reset_robot_and_gripper_to(robot_interface, reset_joint_positions, gripper_positions):
    controller_type = "JOINT_POSITION"
    controller_cfg = YamlConfig(config_root + f"/joint-impedance-controller.yml").as_easydict()
    action = reset_joint_positions + [-1]
    while True:
        if len(robot_interface._state_buffer) > 0:
            curr_q = np.array(robot_interface._state_buffer[-1].q)
            curr_grip_pos = np.array(robot_interface._gripper_state_buffer[-1].width)
            curr_grip_state = np.array([curr_grip_pos * 0.5, -curr_grip_pos * 0.5])

            if (np.max(np.abs(curr_q - np.array(reset_joint_positions))) < 1e-3 and
                np.max(np.abs(curr_grip_state - np.array(gripper_positions))) < 0.005):
                break
            if gripper_positions[0] > curr_grip_state[0]:
                action[-1] = -0.1
            elif gripper_positions[0] < curr_grip_state[0]:
                action[-1] = 0.1

        robot_interface.control(
            control_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )


class BaseEnv():
    def __init__(self, control_type, cam_pose, robot_state_type, depth_unit,
                 num_repeat=1, cam_id=0, control_freq=20, base_offset=[-0.56, 0, 0.912]):

        self.rate = Rate(control_freq)
        self.rate_measure = RateMeasure(name="robot", freq_threshold=round(0.95 * control_freq))
        self.timers = Timers(history=100, disable_on_creation=False)

        self.robot_state_type = robot_state_type
        self.base_offset = base_offset
        self.depth_unit = depth_unit

        self.num_repeat = num_repeat

        # self.cam_interfaces = CameraRedisSubInterface(camera_id=cam_id, use_depth=True)

        self.obs_buff = []
        
        robot_interface = FrankaInterface(config_root + f"/charmander.yml", use_visualizer=False)
        self.control_type = control_type

        if control_type == "OSC_POSE":
            controller_cfg = YamlConfig(
                os.path.join(config_root, "osc-controller.yml")
            ).as_easydict()
        elif control_type == "JOINT_IMPEDANCE":
            controller_cfg = YamlConfig(
                config_root + f"/joint-impedance-controller.yml"
            ).as_easydict()
        else:
            raise NotImplementedError

        cprint(controller_cfg, "light_magenta")

        # controller_cfg["joint_kp"] = [100.0, 100.0, 100.0, 100.0, 75.0, 150.0, 50.0]
        # controller_cfg["joint_kd"] = [20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0]

        # controller_cfg["joint_kp"] = [50., 50., 50., 50., 5., 20., 20.]
        # controller_cfg["joint_kd"] = [0., 0., 0., 0., 0., 0., 0.]

        # controller_cfg["joint_kp"] = [600., 600., 600., 600., 75., 150., 50.]
        # controller_cfg["joint_kd"] = [50., 50., 50., 50., 7.5, 15.0, 5.0]

        controller_cfg["joint_kp"] = [125.0, 125.0, 125.0, 125.0, 100.0, 175.0, 75.0]
        controller_cfg["joint_kd"] = [20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0]

        # controller_cfg["joint_kp"] = [125.0, 200.0, 125.0, 125.0, 100.0, 175.0, 75.0]
        # controller_cfg["joint_kd"] = [20.0, 25.0, 20.0, 20.0, 7.5, 15.0, 5.0]

        self.robot_interface = robot_interface
        self.control_cfg = controller_cfg
        self.cam2base = np.array(cam_pose)
        # self.cam_K = self.get_cam_model()

        self.obs_buff = []
        cprint(f"Init BaseEnv!", "green")

    def get_cam_model(self):

        while True:
            info = self.cam_interfaces.get_img_info()
            if info is not None:
                break

        intrinsics = info["intrinsics"]["color"]

        cam_K = np.eye(4)
        cam_K[0, 0] = intrinsics["fx"]
        cam_K[0, 2] = intrinsics["cx"]
        cam_K[1, 1] = intrinsics["fy"]
        cam_K[1, 2] = intrinsics["cy"]
        return cam_K

    def get_curr_cam_obs(self):
        return self.render(None)

    def get_curr_robot_state(self):
        curr_q = np.array(self.robot_interface._state_buffer[-1].q)
        curr_eef_pose = np.array(self.robot_interface._state_buffer[-1].O_T_EE).reshape((4, 4)).T
        # curr_grip_pos = np.array(self.robot_interface._gripper_state_buffer[-1].width)
        # curr_grip_state = np.array([curr_grip_pos * 0.5, -curr_grip_pos * 0.5])

        eef_pos = np.array(curr_eef_pose[:3, 3]) + np.array(self.base_offset)
        eef_quat = mat2quat(curr_eef_pose[:3, :3])

        agent_eef_state = np.concatenate([eef_pos, eef_quat], axis=0)
        agent_joint_state = np.concatenate([curr_q], axis=0)

        ret = {"joint": agent_joint_state, "eef_pose": agent_eef_state,
            #    "robot0_gripper_qpos": curr_grip_state,
               "robot0_joint_pos": curr_q,
               "robot0_eef_pos": eef_pos,
               "robot0_eef_quat": eef_quat,
               }

        return ret

    def get_obs(self):
        raise NotImplementedError

    def convert_prediction_to_control(self, pred):
        raise NotImplementedError

    def step(self, pred):
        for _ in range(self.num_repeat):
            action = self.convert_prediction_to_control(pred)
            if action[-1] > -0.3:
                action[-1] = 1
            else:
                action[-1] = -1

            print("get action")
            self.robot_interface.control(
                control_type=self.control_type,
                action=action,
                controller_cfg=self.control_cfg,
            )

        tic_pts_gen = time.time()
        # obs_dict = self.get_obs()
        obs_dict = {}
        cprint(f"[Obs Gen] - Time: {time.time() - tic_pts_gen:.3f}", "cyan")

        # self.obs_buff.append(obs_dict)

        info = {"success": False}
        done = False
        reward = 0.
        return obs_dict, reward, done, info


    def reset(self):

        reset_joint_positions = [
            0.09162008114028396,
            -0.19826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.30396583422025,
            0.8480939705504309,
        ]

        reset_joint_positions = [2.43350033e-03, 1.45289745e-01,  1.30723719e-02,
                                 - 2.60070515e+00, - 1.48433004e-02,  2.98698775e+00,  7.56310850e-01]
    #
        reset_joint_positions = [
            0.0051045983773442835,
            0.2804111146567188,
            -0.023866495014830666,
            -2.430820395839277,
            -0.023033814495924086,
            2.733470055916419,
            0.7564890691918228
        ]
    #
    #     reset_joint_positions = [
    #     -0.24368137818455496,
    #     0.2668645121425094,
    #     0.08002237028080367,
    #     -2.477629663857783,
    #     -0.18123366578295233,
    #     2.7326845933048913,
    #     0.7568394265363695
    # ]


        can_box_in_shallow_bin = [0.092, -0.198, -0.02, -2.473, -0.013, 2.304, 0.848]
        square_joint_positions = [0, 0.196, 0, -2.617, 0, 2.942, 0.785]

        reset_joint_positions = can_box_in_shallow_bin

        reset_robot_to(self.robot_interface, reset_joint_positions)


        # reset_joint_positions = [0, 0.196, 0, -2.617, -0.001, 2.943, 0.784]
        # reset_gripper_positions = [0.021, -0.021]
        # reset_robot_and_gripper_to(self.robot_interface, reset_joint_positions, reset_gripper_positions)

        self.obs_buff = []
        gc.collect()
        obs_dict = self.get_obs()
        raw_obs = obs_dict.copy()
        self.obs_buff.append(raw_obs)

        return obs_dict

    def reset_to(self, jpos):

        reset_robot_to(self.robot_interface, jpos[:7])

        self.obs_buff = []
        gc.collect()
        obs_dict = self.get_obs()
        raw_obs = obs_dict.copy()
        self.obs_buff.append(raw_obs)
        return obs_dict

    def render(self, mode):
        raw_frame = self.cam_interfaces.get_img()

        rgb_img = cv2.cvtColor(np.array(raw_frame["color"]), cv2.COLOR_BGR2RGB)
        ret = {}
        ret["image"] = rgb_img
        dep = np.array(raw_frame["depth"]) * self.depth_unit
        ret["depth"] = dep
        # ret["points"] = raw_frame["points"]
        return ret

    def get_raw_obs_buff(self):
        return self.obs_buff

    def _check_success(self):
        return False

def write_ply(points, colors, save_path):
    if colors.max() > 1:
        div_ = 255.
    else:
        div_ = 1.
    os.makedirs(osp.dirname(save_path), exist_ok=True)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors / div_)
    o3d.io.write_point_cloud(save_path, pcd, write_ascii=False)


def convert_list_of_dict_to_dict_of_list(dict_seq):
    obs_keys = dict_seq[0].keys()
    dict_list = {}
    for k in obs_keys:
        dict_list[k] = np.array([o[k] for o in dict_seq])
    return dict_list


def dump_data_to_hdf5(data_grp, ep, dict_seq, compress=True):
    ep_data_grp = data_grp.create_group(ep)
    obs_keys = dict_seq.keys()
    ds = 4

    cprint(f"check all obs keys: {obs_keys}", "cyan")

    for k in obs_keys:
        cur_obs = np.array(dict_seq[k])
        if str(k).endswith(("image", "depth", "segmentation_instance")):
            cur_obs = cur_obs[:, ::ds, ::ds]
            print(f"{k} down sample ...", cur_obs.shape)
        if compress:
            ep_data_grp.create_dataset("obs/{}".format(k), data=cur_obs, compression="gzip")
        else:
            ep_data_grp.create_dataset("obs/{}".format(k), data=cur_obs)

    ep_data_grp.attrs["num_samples"] = cur_obs.shape[0]
    return cur_obs.shape[0]


def generate_points_from_depth(depth, proj):
    '''
    :param depth: (B, 1, H, W)
    :param proj: (B, 4, 4)
    :return: point_cloud (B, 3, H, W)
    '''
    batch, height, width = depth.shape[0], depth.shape[2], depth.shape[3]
    inv_proj = torch.inverse(proj)

    rot = inv_proj[:, :3, :3]  # [B,3,3]
    trans = inv_proj[:, :3, 3:4]  # [B,3,1]

    y, x = torch.meshgrid([torch.arange(0, height, dtype=torch.float32, device=depth.device),
                           torch.arange(0, width, dtype=torch.float32, device=depth.device)])
    y, x = y.contiguous(), x.contiguous()
    y, x = y.view(height * width), x.view(height * width)
    xyz = torch.stack((x, y, torch.ones_like(x)))  # [3, H*W]
    xyz = torch.unsqueeze(xyz, 0).repeat(batch, 1, 1)  # [B, 3, H*W]
    rot_xyz = torch.matmul(rot, xyz)  # [B, 3, H*W]
    rot_depth_xyz = rot_xyz * depth.view(batch, 1, -1)
    proj_xyz = rot_depth_xyz + trans.view(batch, 3, 1)  # [B, 3, H*W]
    proj_xyz = proj_xyz.view(batch, 3, height, width)

    return proj_xyz

def estimate_april_tag_poses(img, dep, all_tag_ids, cam_K, cam_pose, tag_size=0.0476):

    world2cam = cam_K @ np.linalg.inv(cam_pose)
    proj_mat_t = torch.from_numpy(world2cam).unsqueeze(0).float()
    depth_t = torch.from_numpy(dep).unsqueeze(0).unsqueeze(0).float()

    points = generate_points_from_depth(depth_t, proj_mat_t)
    points = points.detach().numpy()[0].transpose(1, 2, 0)


    all_det_res = {}
    for tag_id in all_tag_ids:
        this_tag_res = get_tag_pixel_coord(img=img, cam_K=cam_K, tag_id=tag_id,
                                           tag_size=tag_size)

        if this_tag_res is not None:
            [_col, _row], z_rot = this_tag_res
            _col = int(_col)
            _row = int(_row)
            obj_pos = points[_row, _col]
            obj_pos[2] = 0

            obj_rot = Rotation.from_euler("xyz", [0, 0, z_rot], degrees=True).as_matrix()

            obj_pose_4x4 = np.eye(4)
            obj_pose_4x4[:3, :3] = obj_rot
            obj_pose_4x4[:3, 3] = obj_pos

            all_det_res[tag_id] = obj_pose_4x4
        else:
            all_det_res[tag_id] = None

    return all_det_res

def get_tag_pixel_coord(img, cam_K, tag_id, tag_size=0.0476):
    intrinsics = {"fx": cam_K[0, 0], "cx": cam_K[0, 2], "fy": cam_K[1, 1], "cy": cam_K[1, 2]}
    april_detector = AprilTagDetector()
    ret = april_detector.detect(img,
                                intrinsics=intrinsics,
                                tag_size=tag_size)

    cprint(f"AprilTagDetector: {ret}", "yellow")

    # viz_img = april_detector.vis_tag(img)
    # plt.imshow(viz_img)
    # plt.show()

    for x in ret:
        euler_angles = Rotation.from_matrix(x.pose_R).as_euler("xyz", degrees=True)

        if x.tag_id != tag_id:
            continue

        return x.center, euler_angles[2] % 360

    return None

def save_rollout_data(buffer, raw_policy_outputs, ctrl_cfg,
                      keys, save_dir):
    os.makedirs(save_dir, exist_ok=True)

    data_to_save = {}
    for k in keys:
        if k == "point_cloud":
            continue
        data = [obs[k] for obs in buffer]
        data = np.array(data)
        data_to_save[k] = data.tolist()

    data_to_save["raw_policy_outputs"] = np.array(raw_policy_outputs).tolist()
    data_to_save["ctrl_cfg"] = ctrl_cfg

    video_write = imageio.get_writer(f"{save_dir}/video.mp4", fps=20)
    raw_data_path = f"{save_dir}/raw_frames"
    os.makedirs(raw_data_path, exist_ok=True)
    for ii, obs in enumerate(buffer):
        image = obs["image"]
        video_write.append_data(image)

        Image.fromarray(image).save(f"{raw_data_path}/{ii:08d}.png")
        np.save(f"{raw_data_path}/{ii:08d}.npy", np.array(obs["depth"]))

        if "point_cloud" in keys:
            pts_i = obs["point_cloud"]
            write_ply(points=pts_i, colors=np.zeros(pts_i.shape),
                      save_path=f"{raw_data_path}/{ii:08d}.ply")


    with open(save_dir+"/info.json", 'w+') as f:
        json.dump(data_to_save, f,
                  indent=4)
    print("save data to: ", save_dir)


def save_video(buffer, save_dir, save_name):
    os.makedirs(save_dir, exist_ok=True)

    video_write = imageio.get_writer(f"{save_dir}/{save_name}_video.mp4", fps=20)
    raw_data_path = f"{save_dir}/{save_name}_raw_frame"
    os.makedirs(raw_data_path, exist_ok=True)
    for ii, obs in enumerate(buffer):
        image = obs["image"]
        video_write.append_data(image)

        Image.fromarray(image).save(f"{raw_data_path}/{ii:08d}.png")
        pts_i = obs["point_cloud"]
        write_ply(points=pts_i, colors=np.zeros(pts_i.shape),
                  save_path=f"{raw_data_path}/{ii:08d}.ply")


class PoseObs(BaseEnv):
    def __init__(self, obj2tag:dict, obj2offset, obj2quat, obj_order, **kwargs):
        super(PoseObs, self).__init__(**kwargs)
        self.obj2tag = obj2tag
        self.obj_order = obj_order
        self.obj2offset = obj2offset
        self.obj2quat = obj2quat

        cprint(f"Init PoseObs!", "green")

        self.has_det = False
        self.obj2pose = {}


    def reset(self):
        self.has_det = False
        self.obj2pose = {}
        return super().reset()

    def reset_to(self, jpos):
        self.has_det = False
        self.obj2pose = {}
        return super().reset_to(jpos)

    def get_obs(self):
        cam_obs = self.get_curr_cam_obs()
        img = cam_obs["image"]
        dep = cam_obs["depth"]

        if not self.has_det:
            cam_K = self.cam_K.copy()
            cam_pose = self.cam2base.copy()
            tag_det_res = estimate_april_tag_poses(img=img, dep=dep, all_tag_ids=self.obj2tag.values(),
                                                   cam_K=cam_K, cam_pose=cam_pose)

            for name in self.obj_order:
                tag_id = self.obj2tag[name]
                obj_pose_4x4 = tag_det_res[tag_id]
                if obj_pose_4x4 is None:
                    cprint(f"{name} pose is not detected!", "red")

                    raise ValueError

                obj_pos = obj_pose_4x4[:3, 3] + self.obj2offset[name]
                obj_mat = obj_pose_4x4[:3, :3]
                if self.obj2quat[name] is not None:
                    obj_quat = self.obj2quat[name]
                    cprint(f"{name} uses pre-defined quat: {obj_quat}", "yellow")
                else:
                    obj_quat = mat2quat(obj_mat)
                    cprint(f"{name} uses estimated quat: {obj_quat}", "yellow")

                self.obj2pose[name] = np.concatenate([obj_pos, obj_quat]).tolist()

            self.has_det = True

        robot_state = np.array(self.get_curr_robot_state()[self.robot_state_type]).tolist()
        for name in self.obj_order:
            robot_state += self.obj2pose[name]

        cprint(f"agent state: {robot_state}", "cyan")
        ret = {"agent_pos": np.array(robot_state), "image": img, "depth": dep}

        return ret

class PointCloudObs(BaseEnv):
    def __init__(self, x_range, y_range, z_range, stride, num_pts, samp_method, **kwargs):
        super(PointCloudObs, self).__init__(**kwargs)
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range

        self.stride = stride

        self.num_pts = num_pts
        self.samp_method = samp_method

        cprint(f"Init PointCloudObs!", "green")


    def reset(self):
        return super().reset()

    def reset_to(self, jpos):
        return super().reset_to(jpos)

    def get_obs(self):
        cam_obs = self.get_curr_cam_obs()
        img = cam_obs["image"]
        dep = cam_obs["depth"]

        point_cloud = self.generate_point_cloud(dep_np=dep, stride=self.stride)

        robot_state = np.array(self.get_curr_robot_state()[self.robot_state_type]).tolist()


        cprint(f"agent state: {robot_state}", "cyan")

        ret = {"agent_pos": np.array(robot_state),
               "point_cloud": np.array(point_cloud),
               "image": img, "depth": dep}

        return ret

    def generate_point_cloud(self, dep_np, stride):
        cam_K = self.cam_K.copy()
        cam2base = self.cam2base.copy()

        dep_np = dep_np[::stride, ::stride]
        scale = 1. / stride
        cam_K[:2] *= scale

        world2cam = cam_K @ np.linalg.inv(cam2base)
        proj_mat_t = torch.from_numpy(world2cam).unsqueeze(0).float()
        depth_t = torch.from_numpy(dep_np).unsqueeze(0).unsqueeze(0).float()

        points = generate_points_from_depth(depth_t, proj_mat_t)
        points = points.detach().numpy()[0].transpose(1, 2, 0)
        points = points.reshape((-1, 3))

        pts = points + np.array(self.base_offset).reshape((1, 3))

        x_min, x_max = self.x_range
        y_min, y_max = self.y_range
        z_min, z_max = self.z_range

        cond = ((pts[:, 0] > x_min) & (pts[:, 0] < x_max) &
                (pts[:, 1] > y_min) & (pts[:, 1] < y_max) &
                (pts[:, 2] > z_min) & (pts[:, 2] < z_max))
        scene_pts = pts[cond]

        # scene_pts = point_cloud_filter(scene_pts)
        scene_pts = point_cloud_sampling(scene_pts,
                                         num_points=self.num_pts,
                                         method=self.samp_method)

        return scene_pts


class FrozenPointCloudObs(BaseEnv):
    def __init__(self, x_range, y_range, z_range, stride, num_pts, samp_method, **kwargs):
        super(FrozenPointCloudObs, self).__init__(**kwargs)
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range

        self.stride = stride

        self.num_pts = num_pts
        self.samp_method = samp_method

        self.has_det = False
        self.scene_pts = None

        cprint(f"Init FrozenPointCloudObs!", "green")


    def reset(self):

        self.has_det = False
        self.scene_pts = None

        return super().reset()

    def reset_to(self, jpos):

        self.has_det = False
        self.scene_pts = None

        return super().reset_to(jpos)

    def get_obs(self):
        cam_obs = self.get_curr_cam_obs()
        img = cam_obs["image"]
        dep = cam_obs["depth"]

        if not self.has_det:
            point_cloud = self.generate_point_cloud(dep_np=dep, stride=self.stride)
            self.scene_pts = point_cloud
            self.has_det = True


        point_cloud = self.scene_pts.copy()
        robot_state = np.array(self.get_curr_robot_state()[self.robot_state_type]).tolist()

        cprint(f"agent state: {robot_state}", "cyan")

        ret = {"agent_pos": np.array(robot_state),
               "point_cloud": np.array(point_cloud),
               "image": img, "depth": dep}

        return ret

    def generate_point_cloud(self, dep_np, stride):
        cam_K = self.cam_K.copy()
        cam2base = self.cam2base.copy()

        dep_np = dep_np[::stride, ::stride]
        scale = 1. / stride
        cam_K[:2] *= scale

        world2cam = cam_K @ np.linalg.inv(cam2base)
        proj_mat_t = torch.from_numpy(world2cam).unsqueeze(0).float()
        depth_t = torch.from_numpy(dep_np).unsqueeze(0).unsqueeze(0).float()

        points = generate_points_from_depth(depth_t, proj_mat_t)
        points = points.detach().numpy()[0].transpose(1, 2, 0)
        points = points.reshape((-1, 3))

        pts = points + np.array(self.base_offset).reshape((1, 3))

        x_min, x_max = self.x_range
        y_min, y_max = self.y_range
        z_min, z_max = self.z_range

        cond = ((pts[:, 0] > x_min) & (pts[:, 0] < x_max) &
                (pts[:, 1] > y_min) & (pts[:, 1] < y_max) &
                (pts[:, 2] > z_min) & (pts[:, 2] < z_max))
        scene_pts = pts[cond]

        # scene_pts = point_cloud_filter(scene_pts)
        scene_pts = point_cloud_sampling(scene_pts,
                                         num_points=self.num_pts,
                                         method=self.samp_method)

        return scene_pts


class ImageObs(BaseEnv):
    def __init__(self, img_h, img_w, **kwargs):
        super(ImageObs, self).__init__(**kwargs)
        self.img_h = img_h
        self.img_w = img_w

        cprint(f"Init ImageObs!", "green")


    def reset(self):
        return super().reset()

    def reset_to(self, jpos):
        return super().reset_to(jpos)

    def get_obs(self):
        cam_obs = self.get_curr_cam_obs()
        img = cam_obs["image"]
        dep = cam_obs["depth"]

        img = cv2.resize(img, (self.img_w, self.img_h))
        dep = cv2.resize(dep, (self.img_w, self.img_h))

        # resize image and depth
        full_robot_state = self.get_curr_robot_state()
        robot0_joint_pos = full_robot_state["robot0_joint_pos"]
        robot0_eef_pos = full_robot_state["robot0_eef_pos"]
        robot0_eef_quat = full_robot_state["robot0_eef_quat"]
        robot0_gripper_qpos = full_robot_state["robot0_gripper_qpos"]

        ret = {"agentview_depth": dep,
               "agentview_image": img,
               "robot0_joint_pos": robot0_joint_pos,
               "robot0_gripper_qpos": robot0_gripper_qpos,
               "robot0_eef_pos": robot0_eef_pos,
               "robot0_eef_quat": robot0_eef_quat
               }

        return ret


class DeltaJoint2AbsJoint(BaseEnv):
    def __init__(self, **kwargs):
        super(DeltaJoint2AbsJoint, self).__init__(**kwargs)
        cprint(f"Init DeltaJoint2AbsJoint!", "green")
        assert self.control_type == "JOINT_IMPEDANCE", self.control_type

    def convert_prediction_to_control(self, pred):
        curr_q = np.array(self.robot_interface._state_buffer[-1].q)
        grip = pred[-1]

        print("raw pred: ", pred)

        clipped_action = np.clip(np.array(pred[:-1]), a_min=-1, a_max=1) * 0.1

        print("clipped: ", clipped_action)


        targ_q = clipped_action + curr_q
        action = np.array(targ_q.tolist() + [grip])
        return action

class DeltaJoint2AbsJointPLAI(BaseEnv):
    def __init__(self, **kwargs):
        super(DeltaJoint2AbsJointPLAI, self).__init__(**kwargs)
        cprint(f"Init DeltaJoint2AbsJoint!", "green")
        self.desired_q = None
        assert self.control_type == "JOINT_IMPEDANCE", self.control_type

    def convert_prediction_to_control(self, pred):
        # curr_q = np.array(self.robot_interface._state_buffer[-1].q)

        curr_q = self.desired_q.copy()

        print("raw pred: ", pred)
        grip = pred[-1]
        clipped_action = np.clip(np.array(pred[:-1]), a_min=-1, a_max=1) * 0.1
        print("clipped: ", clipped_action)

        targ_q = clipped_action + curr_q
        self.desired_q = targ_q.copy()

        action = np.array(targ_q.tolist() + [grip])

        return action

    def reset(self):
        obs = super().reset()
        self.desired_q = np.array(self.robot_interface._state_buffer[-1].q)
        return obs

    def reset_to(self, jpos):
        obs = super().reset_to(jpos)
        self.desired_q = np.array(self.robot_interface._state_buffer[-1].q)
        return obs


class AbsJoint2AbsJoint(BaseEnv):
    def __init__(self, **kwargs):
        super(AbsJoint2AbsJoint, self).__init__(**kwargs)
        cprint(f"Init AbsJoint2AbsJoint!", "green")
        assert self.control_type == "JOINT_IMPEDANCE", self.control_type

    def convert_prediction_to_control(self, pred):
        return pred


class AbsJoint2AbsJointClip(BaseEnv):
    def __init__(self, **kwargs):
        super(AbsJoint2AbsJointClip, self).__init__(**kwargs)
        cprint(f"Init AbsJoint2AbsJoint!", "green")
        assert self.control_type == "JOINT_IMPEDANCE", self.control_type

    def convert_prediction_to_control(self, pred):

        # while True:

        #     print(len(self.robot_interface._state_buffer))
        #     if (len(self.robot_interface._state_buffer) > 0):
        #         break
        print("Received message")

        cur_jpos = np.array(self.robot_interface._state_buffer[-1].q)
        delta_jpos = pred[:7] - cur_jpos
        delta_jpos = np.clip(delta_jpos / 0.05, a_min=-1, a_max=1)
        target_jpos = cur_jpos + delta_jpos * 0.05

        cprint(f"Clipped action: {target_jpos}", "yellow")

        target_action = np.array(target_jpos.tolist() + [pred[-1]])
    
        return target_action


# class AbsEEFPose2AbsJoint(BaseEnv):
#     def __init__(self, num_ik_iter, **kwargs):
#         super(AbsEEFPose2AbsJoint, self).__init__(**kwargs)
#         self.num_ik_iter = num_ik_iter

#         cur_path = pathlib.Path(__file__).parent.resolve()
#         urdf_path = f"{cur_path}/franka_description/robots/panda_arm_hand.urdf"
#         self.ik_solver = YourdfIk(urdf_path, tool_link='panda_hand', speed=False)

#         cprint(f"Init AbsEEFPose2AbsJoint!", "green")
#         assert self.control_type == "JOINT_IMPEDANCE", self.control_type

#     def transform_abs_eef_pose_to_ik_target(self, eef_pose, robot_base):
#         mjc_frame2world = np.array([[0., -1., 0.],
#                                     [1., 0., 0.],
#                                     [0., 0., 1.]])
#         ft2eef = np.eye(4)
#         ft2eef[:3, :3] = mjc_frame2world
#         ft2eef[:3, 3] = np.array([0, 0, -0.097])

#         ft2world = eef_pose @ ft2eef
#         ft2robot = ft2world.copy()
#         ft2robot[:3, 3] -= robot_base  # this is the ik target!
#         return ft2robot

#     def convert_raw_action_to_joint_action(self, raw_action):
#         assert len(raw_action) == 7, raw_action
#         grip = raw_action[-1]
#         rot_mat = quat2mat(axisangle2quat(raw_action[3:6]))
#         trs = raw_action[:3]
#         eef_pose_in_world = np.eye(4)
#         eef_pose_in_world[:3, 3] = trs
#         eef_pose_in_world[:3, :3] = rot_mat

#         ik_target_pose = self.transform_abs_eef_pose_to_ik_target(eef_pose_in_world, self.base_offset)
#         cur_jpos = np.array(self.robot_interface._state_buffer[-1].q)

#         solutions = []
#         for i in range(self.num_ik_iter):
#             target_conf = self.ik_solver.solve(ik_target_pose, seed_conf=cur_jpos)
#             if target_conf is not None:
#                 solutions.append(target_conf)

#         if len(solutions) == 0:
#             print("warning: fail to find ik solution.")
#             target_conf = cur_jpos
#         else:
#             dist = np.sum((np.expand_dims(cur_jpos, axis=0) - np.array(solutions)) ** 2, axis=1, keepdims=False)
#             ind = np.argmin(dist)
#             target_conf = solutions[ind]

#         joint_action = np.zeros((8))
#         joint_action[:7] = target_conf
#         joint_action[7] = grip
#         return joint_action

#     def convert_prediction_to_control(self, pred):
#         joint_action = self.convert_raw_action_to_joint_action(pred)
#         return joint_action

# class AbsEEFPose2AbsEEFPose(BaseEnv):
#     def __init__(self, **kwargs):
#         super(AbsEEFPose2AbsEEFPose, self).__init__(**kwargs)
#         cprint(f"Init AbsEEFPose2AbsEEFPose!", "green")

#         self.obs_eef_to_ctrl_eef = np.array([[0, 1, 0, -1.28e-3],
#                                              [-1, 0, 0, -4.26e-4],
#                                              [0, 0, 1, -4e-3],
#                                              [0, 0, 0, 1]])

#         assert self.control_type == "OSC_POSE", self.control_type

#     def convert_prediction_to_control(self, pred):

#         grip = pred[-1]
#         rot_mat = quat2mat(axisangle2quat(pred[3:6]))
#         pred_abs_eef = np.eye(4)
#         pred_abs_eef[:3, 3] = pred[:3]
#         pred_abs_eef[:3, :3] = rot_mat[:3, :3]

#         pred_eef_pose_w_offset = pred_abs_eef @ np.linalg.inv(self.obs_eef_to_ctrl_eef)
#         pred_eef_pose = pred_eef_pose_w_offset.copy()
#         pred_eef_pose[:3, 3] -= self.base_offset

#         curr_eef_pose = np.array(self.robot_interface._state_buffer[-1].O_T_EE).reshape((4, 4)).T

#         target_pos = pred_eef_pose[:3, 3:]
#         target_rot = pred_eef_pose[:3, :3]
#         target_quat = deoxys_transform.mat2quat(target_rot)

#         current_pos = curr_eef_pose[:3, 3:]
#         current_rot = curr_eef_pose[:3, :3]
#         current_quat = deoxys_transform.mat2quat(current_rot)

#         if np.dot(target_quat, current_quat) < 0.0:
#             current_quat = -current_quat

#         quat_diff = deoxys_transform.quat_distance(target_quat, current_quat)
#         current_axis_angle = deoxys_transform.quat2axisangle(current_quat)
#         axis_angle_diff = deoxys_transform.quat2axisangle(quat_diff)
#         action_pos = (target_pos - current_pos).flatten() * 10
#         action_axis_angle = axis_angle_diff.flatten() * 1
#         action_pos = np.clip(action_pos, -1.0, 1.0)
#         action_axis_angle = np.clip(action_axis_angle, -0.5, 0.5)

#         action = action_pos.tolist() + action_axis_angle.tolist() + [grip]

#         return action


# class RelDecoupEEFPose2AbsJoint(BaseEnv):
#     def __init__(self, num_ik_iter, **kwargs):
#         super(RelDecoupEEFPose2AbsJoint, self).__init__(**kwargs)
#         self.num_ik_iter = num_ik_iter
#         cur_path = pathlib.Path(__file__).parent.resolve()
#         urdf_path = f"{cur_path}/franka_description/robots/panda_arm_hand.urdf"
#         self.ik_solver = YourdfIk(urdf_path, tool_link='panda_hand', speed=False)

#         self.obs_eef_to_ctrl_eef = np.array([[0, 1, 0, -1.28e-3],
#                                         [-1, 0, 0, -4.26e-4],
#                                         [0, 0, 1, -4e-3],
#                                         [0, 0, 0, 1]])

#         cprint(f"Init RelDecoupEEFPose2AbsJoint!", "green")
#         assert self.control_type == "JOINT_IMPEDANCE", self.control_type


#     def pose_integration(self, cur_eef_pose, raw_pred):
#         '''
#         raw pred is derived from relative transform between abs eef pose in world (same as mimicgen abs pose traj)
#         '''
#         obs_eef_to_ctrl_eef = self.obs_eef_to_ctrl_eef.copy()

#         # convert cur eef to abs eef
#         cur_abs_eef = cur_eef_pose @ obs_eef_to_ctrl_eef

#         # compute disired abs eef with raw pred
#         desired_abs_eef = np.eye(4)
#         desired_abs_eef[:3, 3] = cur_abs_eef[:3, 3] + raw_pred[:3]
#         rel_rot = quat2mat(axisangle2quat(raw_pred[3:6]))
#         desired_abs_eef[:3, :3] = cur_abs_eef[:3, :3] @ np.linalg.inv(rel_rot)

#         return desired_abs_eef

#     def transform_abs_eef_pose_to_ik_target(self, eef_pose, robot_base):
#         mjc_frame2world = np.array([[0., -1., 0.],
#                                     [1., 0., 0.],
#                                     [0., 0., 1.]])
#         ft2eef = np.eye(4)
#         ft2eef[:3, :3] = mjc_frame2world
#         ft2eef[:3, 3] = np.array([0, 0, -0.097])

#         ft2world = eef_pose @ ft2eef
#         ft2robot = ft2world.copy()
#         ft2robot[:3, 3] -= robot_base  # this is the ik target!
#         return ft2robot

#     def convert_abs_eef_in_world_to_joint_action(self, abs_pose_in_world, grip):
#         ik_target_pose = self.transform_abs_eef_pose_to_ik_target(abs_pose_in_world, self.base_offset)
#         cur_jpos = np.array(self.robot_interface._state_buffer[-1].q)

#         solutions = []
#         for i in range(self.num_ik_iter):
#             target_conf = self.ik_solver.solve(ik_target_pose, seed_conf=cur_jpos)
#             if target_conf is not None:
#                 solutions.append(target_conf)

#         if len(solutions) == 0:
#             print("warning: fail to find ik solution.")
#             target_conf = cur_jpos
#         else:
#             dist = np.sum((np.expand_dims(cur_jpos, axis=0) - np.array(solutions)) ** 2, axis=1, keepdims=False)
#             ind = np.argmin(dist)
#             target_conf = solutions[ind]

#         joint_action = np.zeros((8))
#         joint_action[:7] = target_conf
#         joint_action[7] = grip
#         return joint_action

#     def convert_prediction_to_control(self, pred):
#         grip = pred[-1]
#         raw_pred = pred[:6]
#         curr_eef_pose_rs = np.array(self.robot_interface._state_buffer[-1].O_T_EE).reshape((4, 4)).T
#         curr_eef_pose_rs[:3, 3] += np.array(self.base_offset)

#         desired_pose = self.pose_integration(cur_eef_pose=curr_eef_pose_rs,
#                                              raw_pred=raw_pred)
#         joint_action = self.convert_abs_eef_in_world_to_joint_action(desired_pose, grip)
#         return joint_action


# class RelDecoupEEFPose2AbsJointPLAI(BaseEnv):
#     def __init__(self, num_ik_iter, **kwargs):
#         super(RelDecoupEEFPose2AbsJointPLAI, self).__init__(**kwargs)
#         self.num_ik_iter = num_ik_iter
#         cur_path = pathlib.Path(__file__).parent.resolve()
#         urdf_path = f"{cur_path}/franka_description/robots/panda_arm_hand.urdf"
#         self.ik_solver = YourdfIk(urdf_path, tool_link='panda_hand', speed=False)

#         self.obs_eef_to_ctrl_eef = np.array([[0, 1, 0, -1.28e-3],
#                                             [-1, 0, 0, -4.26e-4],
#                                             [0, 0, 1, -4e-3],
#                                             [0, 0, 0, 1]])
#         self.desired_abs_eef = None

#         cprint(f"Init RelDecoupEEFPose2AbsJoint!", "green")
#         assert self.control_type == "JOINT_IMPEDANCE", self.control_type

#     def pose_integration(self, cur_eef_pose, raw_pred):
#         '''
#         raw pred is derived from relative transform between abs eef pose in world (same as mimicgen abs pose traj)
#         '''
#         # obs_eef_to_ctrl_eef = self.obs_eef_to_ctrl_eef.copy()

#         # convert cur eef to abs eef
#         # cur_abs_eef = cur_eef_pose @ obs_eef_to_ctrl_eef
#         cur_abs_eef = self.desired_abs_eef.copy()

#         # compute disired abs eef with raw pred
#         desired_abs_eef = np.eye(4)
#         desired_abs_eef[:3, 3] = cur_abs_eef[:3, 3] + raw_pred[:3]
#         rel_rot = quat2mat(axisangle2quat(raw_pred[3:6]))
#         desired_abs_eef[:3, :3] = cur_abs_eef[:3, :3] @ np.linalg.inv(rel_rot)

#         self.desired_abs_eef = desired_abs_eef.copy()

#         return desired_abs_eef

#     def transform_abs_eef_pose_to_ik_target(self, eef_pose, robot_base):
#         mjc_frame2world = np.array([[0., -1., 0.],
#                                     [1., 0., 0.],
#                                     [0., 0., 1.]])
#         ft2eef = np.eye(4)
#         ft2eef[:3, :3] = mjc_frame2world
#         ft2eef[:3, 3] = np.array([0, 0, -0.097])

#         ft2world = eef_pose @ ft2eef
#         ft2robot = ft2world.copy()
#         ft2robot[:3, 3] -= robot_base  # this is the ik target!
#         return ft2robot

#     def convert_abs_eef_in_world_to_joint_action(self, abs_pose_in_world, grip):
#         ik_target_pose = self.transform_abs_eef_pose_to_ik_target(abs_pose_in_world, self.base_offset)
#         cur_jpos = np.array(self.robot_interface._state_buffer[-1].q)

#         solutions = []
#         for i in range(self.num_ik_iter):
#             target_conf = self.ik_solver.solve(ik_target_pose, seed_conf=cur_jpos)
#             if target_conf is not None:
#                 solutions.append(target_conf)

#         if len(solutions) == 0:
#             print("warning: fail to find ik solution.")
#             target_conf = cur_jpos
#         else:
#             dist = np.sum((np.expand_dims(cur_jpos, axis=0) - np.array(solutions)) ** 2, axis=1, keepdims=False)
#             ind = np.argmin(dist)
#             target_conf = solutions[ind]

#         joint_action = np.zeros((8))
#         joint_action[:7] = target_conf
#         joint_action[7] = grip
#         return joint_action

#     def convert_prediction_to_control(self, pred):
#         grip = pred[-1]
#         raw_pred = pred[:6]
#         curr_eef_pose_rs = np.array(self.robot_interface._state_buffer[-1].O_T_EE).reshape((4, 4)).T
#         curr_eef_pose_rs[:3, 3] += np.array(self.base_offset)

#         desired_pose = self.pose_integration(cur_eef_pose=curr_eef_pose_rs,
#                                              raw_pred=raw_pred)
#         joint_action = self.convert_abs_eef_in_world_to_joint_action(desired_pose, grip)
#         return joint_action

#     def reset(self):
#         obs = super().reset()
#         curr_eef_pose_rs = np.array(self.robot_interface._state_buffer[-1].O_T_EE).reshape((4, 4)).T
#         curr_eef_pose_rs[:3, 3] += np.array(self.base_offset)
#         cur_abs_eef = curr_eef_pose_rs @ self.obs_eef_to_ctrl_eef.copy()
#         self.desired_abs_eef = cur_abs_eef
#         return obs

#     def reset_to(self, jpos):
#         obs = super().reset_to(jpos)
#         curr_eef_pose_rs = np.array(self.robot_interface._state_buffer[-1].O_T_EE).reshape((4, 4)).T
#         curr_eef_pose_rs[:3, 3] += np.array(self.base_offset)
#         cur_abs_eef = curr_eef_pose_rs @ self.obs_eef_to_ctrl_eef.copy()
#         self.desired_abs_eef = cur_abs_eef
#         return obs

# class Pose2DeltaJointControlJoint(PoseObs, DeltaJoint2AbsJoint):
#     def __init__(self, obj2tag, obj_order, obj2offset, obj2quat, control_type, cam_pose, robot_state_type, depth_unit,):
#         super(Pose2DeltaJointControlJoint, self).__init__(obj2tag=obj2tag, obj_order=obj_order,
#                                                           obj2offset=obj2offset, obj2quat=obj2quat,
#                                                           control_type=control_type, cam_pose=cam_pose,
#                                                           robot_state_type=robot_state_type, depth_unit=depth_unit)


# class Pose2JointControlJoint(PoseObs, AbsJoint2AbsJoint):
#     def __init__(self, obj2tag, obj_order, obj2offset, obj2quat, control_type, cam_pose, robot_state_type, depth_unit,):
#         super(Pose2JointControlJoint, self).__init__(obj2tag=obj2tag, obj_order=obj_order,
#                                                      obj2offset=obj2offset, obj2quat=obj2quat,
#                                                      control_type=control_type, cam_pose=cam_pose,
#                                                      robot_state_type=robot_state_type, depth_unit=depth_unit)


# class Pose2AbsEEFControlJoint(PoseObs, AbsEEFPose2AbsJoint):
#     def __init__(self, obj2tag, obj_order, num_ik_iter, obj2offset, obj2quat, control_type, cam_pose, robot_state_type, depth_unit,):
#         super(Pose2AbsEEFControlJoint, self).__init__(obj2tag=obj2tag, obj_order=obj_order,
#                                                       obj2offset=obj2offset, obj2quat=obj2quat,
#                                                       control_type=control_type, cam_pose=cam_pose,
#                                                       robot_state_type=robot_state_type, depth_unit=depth_unit,
#                                                       num_ik_iter=num_ik_iter)


# class Pose2RelDecoupEEFControlJoint(PoseObs, RelDecoupEEFPose2AbsJoint):
#     def __init__(self, obj2tag, obj_order, num_ik_iter, obj2offset, obj2quat, control_type, cam_pose, robot_state_type, depth_unit,):
#         super(Pose2RelDecoupEEFControlJoint, self).__init__(obj2tag=obj2tag, obj_order=obj_order,
#                                                             obj2offset=obj2offset, obj2quat=obj2quat,
#                                                             control_type=control_type, cam_pose=cam_pose,
#                                                             robot_state_type=robot_state_type, depth_unit=depth_unit,
#                                                             num_ik_iter=num_ik_iter)

# class Image2JointControlJointClip(ImageObs, AbsJoint2AbsJointClip):
#     def __init__(self, control_type, img_h, img_w, robot_state_type, cam_pose, depth_unit):
#         super(Image2JointControlJointClip, self).__init__(img_h=img_h,
#                                                           img_w=img_w,
#                                                           control_type=control_type,
#                                                           robot_state_type=robot_state_type,
#                                                           cam_pose=cam_pose,
#                                                           depth_unit=depth_unit)


# class PointCloud2JointControlJoint(PointCloudObs, AbsJoint2AbsJoint):
#     def __init__(self, x_range, y_range, z_range, stride, num_pts, samp_method, control_type,
#                  cam_pose, robot_state_type, depth_unit,):
#         super(PointCloud2JointControlJoint, self).__init__(x_range=x_range, y_range=y_range,
#                                                            z_range=z_range, stride=stride,
#                                                            num_pts=num_pts, samp_method=samp_method,
#                                                            control_type=control_type, cam_pose=cam_pose,
#                                                            robot_state_type=robot_state_type, depth_unit=depth_unit)

class PointCloud2JointControlJointClip(PointCloudObs, AbsJoint2AbsJointClip):
    def __init__(self, x_range, y_range, z_range, stride, num_pts, samp_method, control_type,
                 cam_pose, robot_state_type, depth_unit,):
        super(PointCloud2JointControlJointClip, self).__init__(x_range=x_range, y_range=y_range,
                                                           z_range=z_range, stride=stride,
                                                           num_pts=num_pts, samp_method=samp_method,
                                                           control_type=control_type, cam_pose=cam_pose,
                                                           robot_state_type=robot_state_type, depth_unit=depth_unit)


# class FrozenPointCloud2JointControlJoint(FrozenPointCloudObs, AbsJoint2AbsJoint):
#     def __init__(self, x_range, y_range, z_range, stride, num_pts, samp_method, control_type,
#                  cam_pose, robot_state_type, depth_unit,):
#         super(FrozenPointCloud2JointControlJoint, self).__init__(x_range=x_range, y_range=y_range,
#                                                            z_range=z_range, stride=stride,
#                                                            num_pts=num_pts, samp_method=samp_method,
#                                                            control_type=control_type, cam_pose=cam_pose,
#                                                            robot_state_type=robot_state_type, depth_unit=depth_unit)


# class PointCloud2AbsEEFControlJoint(PointCloudObs, AbsEEFPose2AbsJoint):
#     def __init__(self, x_range, y_range, z_range, stride, num_pts, samp_method, control_type,
#                  cam_pose, robot_state_type, depth_unit, num_ik_iter):
#         super(PointCloud2AbsEEFControlJoint, self).__init__(x_range=x_range, y_range=y_range,
#                                                            z_range=z_range, stride=stride,
#                                                            num_pts=num_pts, samp_method=samp_method,
#                                                            control_type=control_type, cam_pose=cam_pose,
#                                                             num_ik_iter=num_ik_iter,
#                                                            robot_state_type=robot_state_type, depth_unit=depth_unit)


# class PointCloud2DeltaJointControlJoint(PointCloudObs, DeltaJoint2AbsJoint):
#     def __init__(self, x_range, y_range, z_range, stride, num_pts, samp_method, control_type,
#                  cam_pose, robot_state_type, depth_unit,):
#         super(PointCloud2DeltaJointControlJoint, self).__init__(x_range=x_range, y_range=y_range,
#                                                            z_range=z_range, stride=stride,
#                                                            num_pts=num_pts, samp_method=samp_method,
#                                                            control_type=control_type, cam_pose=cam_pose,
#                                                            robot_state_type=robot_state_type, depth_unit=depth_unit)

# class PointCloud2RelDecoupEEFControlJoint(PointCloudObs, RelDecoupEEFPose2AbsJoint):
#     def __init__(self, x_range, y_range, z_range, stride, num_pts, samp_method, control_type,
#                  cam_pose, robot_state_type, depth_unit, num_ik_iter):
#         super(PointCloud2RelDecoupEEFControlJoint, self).__init__(x_range=x_range, y_range=y_range,
#                                                            z_range=z_range, stride=stride,
#                                                            num_pts=num_pts, samp_method=samp_method,
#                                                             num_ik_iter=num_ik_iter,
#                                                            control_type=control_type, cam_pose=cam_pose,
#                                                            robot_state_type=robot_state_type, depth_unit=depth_unit)


# class PointCloud2AbsEEFControlOSC(PointCloudObs, AbsEEFPose2AbsEEFPose):
#     def __init__(self, x_range, y_range, z_range, stride, num_pts, samp_method, control_type,
#                  cam_pose, robot_state_type, depth_unit, num_repeat):
#         super(PointCloud2AbsEEFControlOSC, self).__init__(x_range=x_range, y_range=y_range,
#                                                            z_range=z_range, stride=stride,
#                                                            num_pts=num_pts, samp_method=samp_method,
#                                                            control_type=control_type, cam_pose=cam_pose,
#                                                            num_repeat=num_repeat, robot_state_type=robot_state_type,
#                                                           depth_unit=depth_unit)

def dump_data_to_train_hdf5(data_grp, ep, obs_dict_seq, action_dict_seq, compress=True):
    ep_data_grp = data_grp.create_group(ep)

    agent_pos = np.array(obs_dict_seq["agent_pos"])[:-1]

    n = len(agent_pos)
    coords = np.array(obs_dict_seq["point_cloud"])[:n, ]
    colors = np.ones(coords.shape, ) * 255
    point_cloud = np.concatenate([coords, colors], axis=2)

    obs_to_data = {"agentview_pts": point_cloud,
                   "robot0_joint_pos": agent_pos[:, :7],
                   "robot0_gripper_qpos": agent_pos[:, 7:9],
                   "robot0_eef_pos": np.zeros((n, 3)),
                   "robot0_eef_quat": np.zeros((n, 4)),
                   }
    for k, v in obs_to_data.items():
        if compress:
            ep_data_grp.create_dataset("obs/{}".format(k), data=v, compression="gzip")
        else:
            ep_data_grp.create_dataset("obs/{}".format(k), data=v)

    actions = np.array(action_dict_seq["action"])
    ep_data_grp.create_dataset("joint_configs", data=actions)

    assert len(actions) == n, actions.shape

    ep_data_grp.attrs["num_samples"] = n
    return n


# class RealRobotRunner(BaseRunner):
#     def __init__(self, output_dir, robot_state_type, policy_type, cam_pose, eval_episodes,
#                  depth_unit, max_episode_steps, required_keys, n_obs_steps, n_action_steps, **kwargs):

#         super().__init__(output_dir)

#         self.policy_type = policy_type
#         self.robot_state_type = robot_state_type
#         self.required_keys = required_keys
#         self.n_obs_steps = n_obs_steps
#         self.n_action_steps = n_action_steps
#         self.max_episode_steps = max_episode_steps
#         self.eval_episodes = eval_episodes

#         self.cam_pose = np.array(cam_pose)
#         self.depth_unit = depth_unit

#         self.env = self.env_fn()

#     def env_fn(self):
#         raise NotImplementedError

#     def run(self, policy, save_pred_keys, save_obs_to_hdf5, save_to_train_hdf5=False,
#             save_train_name=None, estimate_tags=None):
#         device = policy.device
#         env = self.env

#         dt_string = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")


#         real_rollout_folder = f"{self.output_dir}/real_rollout_{dt_string}"
#         os.makedirs(real_rollout_folder, exist_ok=True)

#         if save_obs_to_hdf5:
#             save_obs_hdf5_path = f"{real_rollout_folder}/{osp.basename(real_rollout_folder)}_obs.hdf5"
#             f_obs_out = h5py.File(save_obs_hdf5_path, "w")
#             data_obs_grp = f_obs_out.create_group("data")
#             num_obs_samp = 0

#         if save_pred_keys is not None:
#             save_pred_hdf5_path = f"{real_rollout_folder}/{osp.basename(real_rollout_folder)}_pred.hdf5"
#             f_pred_out = h5py.File(save_pred_hdf5_path, "w")
#             data_pred_grp = f_pred_out.create_group("pred")
#             num_pred_samp = 0

#         if save_to_train_hdf5:
#             if save_train_name is not None:
#                 save_train_hdf5_path = f"{real_rollout_folder}/{save_train_name}.hdf5"
#             else:
#                 save_train_hdf5_path = f"{real_rollout_folder}/{osp.basename(real_rollout_folder)}_train.hdf5"

#             f_train_out = h5py.File(save_train_hdf5_path, "w")
#             train_grp = f_train_out.create_group("data")
#             num_transition_collected = 0

#         for episode_idx in tqdm.tqdm(range(self.eval_episodes)):

#             # start rollout
#             env.env.clear_video_buffer()
#             obs = env.reset()

#             if estimate_tags is not None:

#                 render_obs = env.render(None)
#                 img = render_obs["image"]
#                 dep = render_obs["depth"]
#                 det_res = estimate_april_tag_poses(img=img,
#                                                    dep=dep,
#                                                    all_tag_ids=estimate_tags,
#                                                    cam_pose=self.cam_pose,
#                                                    cam_K=self.env.cam_K
#                                                    )

#                 for k, v in det_res.items():
#                     det_res[k] = v.tolist()

#                 cprint(f"tag detection res: {det_res}", "cyan")
#                 save_scene_cfg_path = f"{real_rollout_folder}/{osp.basename(real_rollout_folder)}_scene_cfg_{episode_idx}.json"
#                 with open(save_scene_cfg_path, "w") as f:
#                     json.dump(det_res, f, indent=4)


#             policy.reset()

#             done = False
#             actual_step_count = 0
#             pred_data_to_save = {}

#             tic_end_of_prv = time.time()
#             while not done:
#                 # create obs dict
#                 np_obs_dict = dict(obs)

#                 # device transfer
#                 obs_dict = dict_apply(np_obs_dict,
#                                       lambda x: torch.from_numpy(x).to(
#                                           device=device))

#                 tic_action = time.time()
#                 # run policy
#                 with torch.no_grad():
#                     obs_dict_input = {}  # flush unused keys
#                     for obs_k in self.required_keys:
#                         if obs_k in obs_dict:
#                             obs_dict_input[obs_k] = obs_dict[obs_k].unsqueeze(0)

#                     for obs_k in self.required_keys:
#                         in_k = obs_dict_input[obs_k]
#                         print(f"[Input {obs_k}] - shape: {in_k.shape}")

#                     action_dict = policy.predict_action(obs_dict_input)

#                     if "EoE" in action_dict and action_dict["EoE"].sum() > 0:
#                         break

#                 # device_transfer
#                 np_action_dict = dict_apply(action_dict,
#                                             lambda x: x.detach().to('cpu').numpy())

#                 if save_pred_keys is not None:
#                     for k in save_pred_keys:
#                         pred_k = np_action_dict[k].squeeze(0)
#                         pred_data_to_save[k] = pred_data_to_save.get(k, []) + [pred_k]

#                 action = np_action_dict['action'].squeeze(0)

#                 print(f"[Action Pred] - Time: {time.time() - tic_action: .3f}, Shape: {action.shape}")
#                 cprint(f"action: {action}", "cyan")

#                 tic_exec = time.time()

#                 # step env
#                 print(f"[Execution between batch] - Time: {time.time() - tic_end_of_prv: .3f}")
#                 obs, reward, done, info = env.step(action)
#                 actual_step_count += 1

#                 print(f"[Execution #{actual_step_count}] - Time: {time.time() - tic_exec:.3f}, Done: {done}")
#                 tic_end_of_prv = time.time()

#                 # input()

#             raw_obs_buff = env.env.env.get_raw_obs_buff()
#             save_video(buffer=raw_obs_buff, save_dir=real_rollout_folder,
#                        save_name=f"{episode_idx:04d}")
#             cprint(f"Save rollout to {real_rollout_folder} ...", "red")

#             if save_to_train_hdf5:
#                 obs_dict_seq = convert_list_of_dict_to_dict_of_list(raw_obs_buff)
#                 for pred_name in save_pred_keys:
#                     pred_k_cat = np.concatenate(pred_data_to_save[pred_name], axis=0)
#                     # cprint(f"check saved k: {pred_k_cat.shape}", "cyan")
#                     pred_data_to_save[pred_name] = pred_k_cat

#                 traj_len = dump_data_to_train_hdf5(data_grp=train_grp,
#                                         ep=f"demo_{episode_idx}",
#                                         obs_dict_seq=obs_dict_seq,
#                                         action_dict_seq=pred_data_to_save,
#                                         compress=True)
#                 num_transition_collected += traj_len

#             if save_obs_to_hdf5:
#                 obs_dict_seq = convert_list_of_dict_to_dict_of_list(raw_obs_buff)
#                 traj_len = dump_data_to_hdf5(data_grp=data_obs_grp,
#                                              ep=f"demo_{episode_idx}",
#                                              dict_seq=obs_dict_seq)
#                 num_obs_samp += traj_len

#             if save_pred_keys is not None:
#                 for pred_name in save_pred_keys:
#                     pred_k_cat = np.concatenate(pred_data_to_save[pred_name], axis=0)
#                     # cprint(f"check saved k: {pred_k_cat.shape}", "cyan")
#                     pred_data_to_save[pred_name] = pred_k_cat

#                 traj_len = dump_data_to_hdf5(data_grp=data_pred_grp,
#                                              ep=f"demo_{episode_idx}",
#                                              dict_seq=pred_data_to_save)
#                 num_pred_samp += traj_len

#             _ = env.reset()
#             while True:
#                 pk = input(colored("Have you reset the environment? Y/N", "red"))
#                 if pk == "Y":
#                     break

#         if save_to_train_hdf5:
#             train_grp.attrs["total"] = num_transition_collected
#             f_train_out.close()

#         if save_obs_to_hdf5:
#             data_obs_grp.attrs["total"] = num_obs_samp
#             f_obs_out.close()

#         if save_pred_keys is not None:
#             data_pred_grp.attrs["total"] = num_pred_samp
#             f_pred_out.close()

#         # clear memory
#         del env


# class RealRobotPointCloudBasedRunner(RealRobotRunner):
#     def __init__(self, output_dir, robot_state_type, policy_type, cam_pose,
#                  x_range, y_range, z_range, stride, num_pts, samp_method, depth_unit,
#                  max_episode_steps, required_keys, eval_episodes=2, n_obs_steps=8, n_action_steps=8,):

#         self.x_range = x_range
#         self.y_range = y_range
#         self.z_range = z_range
#         self.stride = stride
#         self.num_pts = num_pts
#         self.samp_method = samp_method
#         super().__init__(output_dir=output_dir, robot_state_type=robot_state_type, policy_type=policy_type,
#                          cam_pose=cam_pose, depth_unit=depth_unit, max_episode_steps=max_episode_steps,
#                          required_keys=required_keys, n_obs_steps=n_obs_steps, n_action_steps=n_action_steps,
#                          eval_episodes=eval_episodes)

#     def env_fn(self, ):
#         if self.policy_type == "Pts2Joint":
#             return MultiStepWrapper(
#                 SimpleVideoRecordingWrapper(
#                     PointCloud2JointControlJoint(
#                         x_range=self.x_range,
#                         y_range=self.y_range,
#                         z_range=self.z_range,
#                         stride=self.stride,
#                         num_pts=self.num_pts,
#                         samp_method=self.samp_method,
#                         control_type="JOINT_IMPEDANCE",
#                         robot_state_type=self.robot_state_type,
#                         cam_pose=self.cam_pose,
#                         depth_unit=self.depth_unit
#                     )
#                 ),
#                 n_obs_steps=self.n_obs_steps,
#                 n_action_steps=self.n_action_steps,
#                 max_episode_steps=self.max_episode_steps,
#                 reward_agg_method='sum',
#                 required_keys=self.required_keys,
#             )
#         elif self.policy_type == "Pts2JointClip":
#             return MultiStepWrapper(
#                 SimpleVideoRecordingWrapper(
#                     PointCloud2JointControlJointClip(
#                         x_range=self.x_range,
#                         y_range=self.y_range,
#                         z_range=self.z_range,
#                         stride=self.stride,
#                         num_pts=self.num_pts,
#                         samp_method=self.samp_method,
#                         control_type="JOINT_IMPEDANCE",
#                         robot_state_type=self.robot_state_type,
#                         cam_pose=self.cam_pose,
#                         depth_unit=self.depth_unit
#                     )
#                 ),
#                 n_obs_steps=self.n_obs_steps,
#                 n_action_steps=self.n_action_steps,
#                 max_episode_steps=self.max_episode_steps,
#                 reward_agg_method='sum',
#                 required_keys=self.required_keys,
#             )
#         elif self.policy_type == "Pts2DeltaJoint":
#             return MultiStepWrapper(
#                 SimpleVideoRecordingWrapper(
#                     PointCloud2DeltaJointControlJoint(
#                         x_range=self.x_range,
#                         y_range=self.y_range,
#                         z_range=self.z_range,
#                         stride=self.stride,
#                         num_pts=self.num_pts,
#                         samp_method=self.samp_method,
#                         control_type="JOINT_IMPEDANCE",
#                         robot_state_type=self.robot_state_type,
#                         cam_pose=self.cam_pose,
#                         depth_unit=self.depth_unit,
#                     )
#                 ),
#                 n_obs_steps=self.n_obs_steps,
#                 n_action_steps=self.n_action_steps,
#                 max_episode_steps=self.max_episode_steps,
#                 reward_agg_method='sum',
#                 required_keys=self.required_keys,
#             )
#         elif self.policy_type == "Pts2AbsEEFPose":
#             return MultiStepWrapper(
#                 SimpleVideoRecordingWrapper(
#                     PointCloud2AbsEEFControlOSC(
#                         x_range=self.x_range,
#                         y_range=self.y_range,
#                         z_range=self.z_range,
#                         stride=self.stride,
#                         num_pts=self.num_pts,
#                         samp_method=self.samp_method,
#                         control_type="OSC_POSE",
#                         robot_state_type=self.robot_state_type,
#                         cam_pose=self.cam_pose,
#                         depth_unit=self.depth_unit,
#                         num_repeat=10,
#                     )
#                 ),
#                 n_obs_steps=self.n_obs_steps,
#                 n_action_steps=self.n_action_steps,
#                 max_episode_steps=self.max_episode_steps,
#                 reward_agg_method='sum',
#                 required_keys=self.required_keys,
#             )
#         elif self.policy_type == "FrozenPts2Joint":
#             return MultiStepWrapper(
#                 SimpleVideoRecordingWrapper(
#                     FrozenPointCloud2JointControlJoint(
#                         x_range=self.x_range,
#                         y_range=self.y_range,
#                         z_range=self.z_range,
#                         stride=self.stride,
#                         num_pts=self.num_pts,
#                         samp_method=self.samp_method,
#                         control_type="JOINT_IMPEDANCE",
#                         robot_state_type=self.robot_state_type,
#                         cam_pose=self.cam_pose,
#                         depth_unit=self.depth_unit
#                     )
#                 ),
#                 n_obs_steps=self.n_obs_steps,
#                 n_action_steps=self.n_action_steps,
#                 max_episode_steps=self.max_episode_steps,
#                 reward_agg_method='sum',
#                 required_keys=self.required_keys,
#             )
#         else:
#             raise NotImplementedError


# class RealRobotPoseBasedRunner(RealRobotRunner):
#     def __init__(self,
#                  output_dir,
#                  robot_state_type,
#                  policy_type,
#                  cam_pose,
#                  obj2tag,
#                  obj_order,
#                  obj2offset,
#                  obj2quat,
#                  depth_unit,
#                  max_episode_steps,
#                  required_keys,
#                  eval_episodes=2,
#                  n_obs_steps=8,
#                  n_action_steps=8,
#                  ):

#         self.obj2tag = obj2tag
#         self.obj_order = obj_order
#         self.obj2offset = obj2offset
#         self.obj2quat = obj2quat

#         super().__init__(output_dir=output_dir, robot_state_type=robot_state_type, policy_type=policy_type,
#                          cam_pose=cam_pose, eval_episodes=eval_episodes, depth_unit=depth_unit,
#                          max_episode_steps=max_episode_steps, required_keys=required_keys,
#                          n_obs_steps=n_obs_steps, n_action_steps=n_action_steps)

#     def env_fn(self, ):
#         if self.policy_type == "FrozenPose2Joint":
#             return MultiStepWrapper(
#                 SimpleVideoRecordingWrapper(
#                     Pose2JointControlJoint(
#                         obj2tag=self.obj2tag,
#                         obj2offset=self.obj2offset,
#                         obj2quat=self.obj2quat,
#                         obj_order=self.obj_order,
#                         control_type="JOINT_IMPEDANCE",
#                         robot_state_type=self.robot_state_type,
#                         cam_pose=self.cam_pose,
#                         depth_unit=self.depth_unit,
#                     )
#                 ),
#                 n_obs_steps=self.n_obs_steps,
#                 n_action_steps=self.n_action_steps,
#                 max_episode_steps=self.max_episode_steps,
#                 reward_agg_method='sum',
#                 required_keys=self.required_keys,
#             )
#         elif self.policy_type == "FrozenPose2DeltaJoint":
#             return MultiStepWrapper(
#                 SimpleVideoRecordingWrapper(
#                     Pose2DeltaJointControlJoint(
#                         obj2tag=self.obj2tag,
#                         obj2offset=self.obj2offset,
#                         obj2quat=self.obj2quat,
#                         obj_order=self.obj_order,
#                         control_type="JOINT_IMPEDANCE",
#                         robot_state_type=self.robot_state_type,
#                         cam_pose=self.cam_pose,
#                         depth_unit=self.depth_unit,
#                     )
#                 ),
#                 n_obs_steps=self.n_obs_steps,
#                 n_action_steps=self.n_action_steps,
#                 max_episode_steps=self.max_episode_steps,
#                 reward_agg_method='sum',
#                 required_keys=self.required_keys,
#             )
#         elif self.policy_type == "FrozenPose2AbsEEFPose":
#             return MultiStepWrapper(
#                 SimpleVideoRecordingWrapper(
#                     Pose2AbsEEFControlJoint(
#                         obj2tag=self.obj2tag,
#                         obj2offset=self.obj2offset,
#                         obj2quat=self.obj2quat,
#                         obj_order=self.obj_order,
#                         control_type="JOINT_IMPEDANCE",
#                         robot_state_type=self.robot_state_type,
#                         cam_pose=self.cam_pose,
#                         depth_unit=self.depth_unit,
#                         num_ik_iter=60,
#                     )
#                 ),
#                 n_obs_steps=self.n_obs_steps,
#                 n_action_steps=self.n_action_steps,
#                 max_episode_steps=self.max_episode_steps,
#                 reward_agg_method='sum',
#                 required_keys=self.required_keys,
#             )
#         elif self.policy_type == "FrozenPose2RelDecoupEEFPose":
#             return MultiStepWrapper(
#                 SimpleVideoRecordingWrapper(
#                     Pose2RelDecoupEEFControlJoint(
#                         obj2tag=self.obj2tag,
#                         obj2offset=self.obj2offset,
#                         obj2quat=self.obj2quat,
#                         obj_order=self.obj_order,
#                         control_type="JOINT_IMPEDANCE",
#                         robot_state_type=self.robot_state_type,
#                         cam_pose=self.cam_pose,
#                         depth_unit=self.depth_unit,
#                         num_ik_iter=60
#                     )
#                 ),
#                 n_obs_steps=self.n_obs_steps,
#                 n_action_steps=self.n_action_steps,
#                 max_episode_steps=self.max_episode_steps,
#                 reward_agg_method='sum',
#                 required_keys=self.required_keys,
#             )
#         else:
#             raise NotImplementedError


# class RealRobotImageBasedRunner(RealRobotRunner):
#     def __init__(self, output_dir, robot_state_type, policy_type, cam_pose,
#                  img_h, img_w, depth_unit, max_episode_steps, required_keys,
#                  eval_episodes=2, n_obs_steps=8, n_action_steps=8,):
#         self.img_h = img_h
#         self.img_w = img_w
#         super().__init__(output_dir=output_dir, robot_state_type=robot_state_type, policy_type=policy_type,
#                          cam_pose=cam_pose, depth_unit=depth_unit, max_episode_steps=max_episode_steps,
#                          required_keys=required_keys, n_obs_steps=n_obs_steps, n_action_steps=n_action_steps,
#                          eval_episodes=eval_episodes)

#     def env_fn(self, ):
#         if self.policy_type == "Img2JointClip":
#             return MultiStepWrapper(
#                 SimpleVideoRecordingWrapper(
#                     Image2JointControlJointClip(
#                         control_type="JOINT_IMPEDANCE",
#                         img_h=self.img_h,
#                         img_w=self.img_w,
#                         cam_pose=self.cam_pose,
#                         depth_unit=self.depth_unit,
#                         robot_state_type=self.robot_state_type,
#                     )
#                 ),
#                 n_obs_steps=self.n_obs_steps,
#                 n_action_steps=self.n_action_steps,
#                 max_episode_steps=self.max_episode_steps,
#                 reward_agg_method='sum',
#                 required_keys=self.required_keys,
#             )
#         else:
#             raise NotImplementedError


# robot hand tag_id: 50
import yaml
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def load_yaml_trajectory_to_numpy(yaml_file):
    """Load a MoveIt‐dumped trajectory from YAML."""
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    points     = data.get('points', [])

    traj = np.zeros((len(points), 7))
    for i, pt in enumerate(points):
        # required
        traj[i] = np.array(pt.get('positions', []))
    
    print("traj ", traj.shape)
    return traj

if __name__ == '__main__':
    import json
    from pathlib import Path
    from plot_results import load_multi_doc, index_planner_results
    json_path = Path(__file__).parent / "UTF-8realsense_high_d435_no_table_tuned_p2048_w_icp.json"          # change the filename if needed
    with json_path.open("r") as f:
        cfg = json.load(f) 
    cam_pose = np.asarray(cfg["pose"], dtype=np.float32)


    env = PointCloud2JointControlJointClip(
                        x_range=[0, 0.1],
                        y_range=[-0.1, 0.1],
                        z_range=[0, 0.1],
                        stride=2,
                        num_pts=512,
                        samp_method="fps",
                        control_type="JOINT_IMPEDANCE",
                        robot_state_type="joint",
                        cam_pose=np.eye(4),
                        depth_unit=0.0005
                    )


    # RRTConnect_yaml = Path(__file__).parent / "Data" / "Trajectories" / "RRTConnect_plan_trj.yaml"
    rrt_trj = load_yaml_trajectory_to_numpy(RRTConnect_yaml)
    
    # import time
    # # send RRTConnect trajectory
    # for i in range(rrt_trj.shape[0]):

    #     print(f"joint {i}: {rrt_trj[i]}")
    #     env.step(rrt_trj[i][:7])

    #     input()

    # send GVIMP trajectory
    gvimp_file = Path(__file__).parent / "Data" / "Trajectories" / "zk_sdf.csv"
    gvimp_trj = np.loadtxt(gvimp_file, delimiter=',').transpose()
    for i in range(gvimp_trj.shape[0]):

        print(f"joint {i}: {gvimp_trj[i]}")
        env.step(gvimp_trj[i][:7])

        input()
    