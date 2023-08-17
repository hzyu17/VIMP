## Show planned trajectories in ROS
# Author/: Hongzhe Yu
# Date: 08/16/2023

import argparse
 
import os
from trajectory_displayer_ros_WAM import *

full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)
root_VIMP = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(full_path))))


def show_one_traj(path_dir):
    # initialize the trajectory displayer class
    displayer = MoveGroupPythonInterfaceTutorial()
    

    file_name = path_dir+"/zk_sdf.csv"
    print("file_name: ", file_name)
    displayer.update_trajectory(file_name)
    
    displayer.go_to_joint_state()
    displayer.display_trajectory()

def main():
    parser = argparse.ArgumentParser(description="A simple argument parser example")
    
    parser.add_argument('--algorithm', '-a', required=True, help="gpmp2, gvi-mp or pgcs-mp")
    parser.add_argument('--experiment', '-e', required=True, help="experiment index")
    parser.add_argument('--verbose', '-v', action='store_true', help="Enable verbose mode")
    
    args = parser.parse_args()
    
    if args.verbose:
        print("Verbose mode enabled.")
    
    print("algorithm: ", args.algorithm)
    print(f"experiment index: {args.experiment}")
    
    
    if (args.algorithm == "gvi-mp"):
        wam_dir = root_VIMP + "/matlab_helpers/GVIMP-examples/WAM/case" + args.experiment
    elif (args.algorithm == "pgcs-mp"):
        wam_dir = root_VIMP + "/matlab_helpers/PGCS-examples/WAM/case" + args.experiment
    elif (args.algorithm == "gpmp2"):
        wam_dir = root_VIMP + "/matlab_helpers/PGCS-examples/WAM/case" + args.experiment + "/gpmp2"
        
    show_one_traj(wam_dir)
    
    # if len(sys.argv) != 2:
    #     print(
    #         'Correct usage:: \n"python3 show_trajectories_paper.py case_index"\n e.g.: python3 show_trajectories_paper.py 1'
    #     )
    #     sys.exit()
    # try: 
    #     print("")
    #     print("----------------------------------------------------------")
    #     print("VIMP ROS trajectory display")
    #     print("----------------------------------------------------------")
    #     print("Press Ctrl-D to exit at any time")
    #     print("")
    #     input(
    #         "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    #     )
        
    #     show_one_traj(int(sys.argv[1]))
        
    # except rospy.ROSInterruptException:
    #     return
    # except KeyboardInterrupt:
    #     return
    

if __name__ == "__main__":
    main()