"""Example script of moving robot joint positions."""
import argparse
import pickle
import threading
import time
from pathlib import Path
import yaml

import matplotlib.pyplot as plt
import numpy as np

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()

def load_yaml_trajectory_to_numpy(yaml_file):
    """Load a MoveIt-dumped trajectory from YAML."""
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    points     = data.get('points', [])

    traj = np.zeros((len(points), 7))
    for i, pt in enumerate(points):
        # required
        traj[i] = np.array(pt.get('positions', []))
    
    # print("traj ", traj.shape)
    return traj

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="joint-position-controller.yml"
    )
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()

    controller_type = "JOINT_POSITION"

    RRTConnectCHOMP_yaml = Path(__file__).parent / "Data" / "Baselines" / "RRTConnect+STOMP_plan_trj.yaml"
    rrt_trj = load_yaml_trajectory_to_numpy(RRTConnectCHOMP_yaml)

    # gvimp_file = Path(__file__).parent / "Data" / "Trajectories" / "zk_sdf.csv"
    gvimp_file = Path(__file__).parent / "Data" / "GVIMP_Hardware_Result" / "good_zk.csv"
    gvimp_trj = np.loadtxt(gvimp_file, delimiter=',').transpose()

    for i in range(gvimp_trj.shape[0]):

        target_positions = gvimp_trj[i][:7]
        # target_positions = rrt_trj[i][:7]

        action = target_positions.tolist() + [-2.0]
        # print(target_positions)
        # print(action)

        while True:
            if len(robot_interface._state_buffer) > 0:
                logger.info(f"Current Robot joint: {np.round(robot_interface.last_q, 3)}")
                logger.info(f"Desired Robot joint: {np.round(robot_interface.last_q_d, 3)}")

                if (
                    np.max(
                        np.abs(
                            np.array(robot_interface._state_buffer[-1].q)
                            - np.array(target_positions)
                        )
                    )
                    < 1e-3
                ):
                    break
            robot_interface.control(
                controller_type=controller_type,
                action=action,
                controller_cfg=controller_cfg,
            )
        
        time.sleep(0.01)
        # input()

    robot_interface.close()


if __name__ == "__main__":
    main()
