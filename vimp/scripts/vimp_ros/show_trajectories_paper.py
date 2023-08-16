## Show planned trajectories in ROS
# Author/: Hongzhe Yu
# Date: 08/16/2023
 
import os
from trajectory_displayer_ros_WAM import *

full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)
root_VIMP = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(full_path))))

wam_root = root_VIMP + "/matlab_helpers/PGCS-examples/WAM"

def show_one_traj(i_case):
    # initialize the trajectory displayer class
    displayer = MoveGroupPythonInterfaceTutorial()
    dir_i = wam_root + "/case" + str(i_case)

    file_name = dir_i+"/zk_sdf.csv"
    print("file_name: ", file_name)
    displayer.update_trajectory(file_name)
    
    displayer.go_to_joint_state()
    displayer.display_trajectory()

def main():
    if len(sys.argv) != 2:
        print(
            'Correct usage:: \n"python3 show_trajectories_paper.py case_index"\n e.g.: python3 show_trajectories_paper.py 1'
        )
        sys.exit()
    try: 
        print("")
        print("----------------------------------------------------------")
        print("VIMP ROS trajectory display")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        
        show_one_traj(int(sys.argv[1]))
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    

if __name__ == "__main__":
    main()