import yaml
from rosbridge_library.internal import message_conversion as mc
from moveit_commander import MoveGroupCommander,RobotCommander
import rospy
import math
import os
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
import numpy as np


# Run these two lines before running this script (without conda envs):
# source Path_To_ws_moveit/devel/setup.bash
# source /opt/ros/noetic/setup.bash

def read_plan_json_to_numpy(plan_trj_file):
    with open(plan_trj_file, "r") as f:
        traj = yaml.safe_load(f)
        
    pos_trajectory = np.array([pt["positions"] for pt in traj["points"]])

    return pos_trajectory


class BaselinePlanner:
    def __init__(self, ros_config, exp_config, planner_id='RRTConnect'):
        self._ros_config = ros_config
        self._exp_config = exp_config

        self._planner = planner_id
        
        rospy.init_node("Baseline planning in moveit...", anonymous=True)

    def plan_from_start_to_goal_joints(self, 
                                       start_degrees, 
                                       goal_degrees, 
                                       result_dir='.'):
        
        
        rospy.loginfo("Going to the start state...")
        self.plan_from_current_to_goal_joints(start_degrees, 
                                              move_group="panda_arm",
                                              execute=True, 
                                              save_results=False)
        
        rospy.loginfo("Plan from the start state to the goal...")
        self.plan_from_current_to_goal_joints(goal_degrees, 
                                              move_group="panda_arm",
                                              execute=False, 
                                              save_results=True, 
                                              result_dir = result_dir)
        
        
    def plan_from_current_to_goal_joints(self, 
                                         goal_degrees, 
                                         move_group="panda_arm",
                                         execute=True, 
                                         save_results=True, 
                                         result_dir='.'):

        # 1) Initialize
        robot = RobotCommander()
        group = MoveGroupCommander(move_group) 
        group.set_planner_id(self._planner)
            
        names = group.get_active_joints()
        # print("Active joints: ", names)
        
        rospy.sleep(0.5)  

        # 2) Define your start and goal joint arrays (length = number of active joints)
        goal_radians = [math.radians(d) for d in goal_degrees]

        # 3) Build a RobotState for your custom start    
        joint_names  = group.get_active_joints()
        joint_values = group.get_current_joint_values()
        
        print("start state: ", [math.degrees(s) for s in joint_values])
        
        js = JointState()
        js.header.stamp   = rospy.Time.now()
        js.name           = joint_names
        js.position       = joint_values
        
        rs = RobotState()
        rs.joint_state = js
        
        group.set_start_state(rs)

        # 4) Set the goal in joint space
        target_dict = dict(zip(names, goal_radians))
        group.set_joint_value_target(target_dict)

        # 5) (Optional) tweak planning parameters
        group.set_planning_time( 5.0 )      # seconds
        group.set_num_planning_attempts(10)

        # 6) Plan and execute
        if execute:
            group.go(wait=True)
    
        else:
            plan = group.plan()
            if plan and save_results:
                traj_msg = plan[1].joint_trajectory
                # convert to plain Python
                traj_dict = mc.extract_values(traj_msg)
                result_file = result_dir + "/" + self._planner + "_plan_trj.yaml"
                
                with open(result_file,'w') as f:
                    yaml.safe_dump(traj_dict, f, default_flow_style=False)
                print("Plan serialized to plan.yaml")
            
            if isinstance(plan, tuple) and len(plan) >= 2:
                success, plan_msg = plan[0], plan[1]
            else:
                # older/newer API might just return the plan
                success, plan_msg = True, plan

            # check
            if not success or not plan_msg.joint_trajectory.points:
                rospy.logerr("Planning failed or empty trajectory")
                return
            
            # display the plan in RViz
            display_pub = rospy.Publisher(
                "/move_group/display_planned_path", DisplayTrajectory, queue_size=1
            )
            rospy.sleep(0.5)
            
            display = DisplayTrajectory()
            display.trajectory_start = robot.get_current_state()
            display.trajectory.append(plan_msg)

            display_pub.publish(display)
            
            
            # For visualization in RViz
            pub = rospy.Publisher('/joint_states', JointState, queue_size=1, latch=True)
            # rospy.init_node('start_state_publisher', anonymous=True)

            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name      = joint_names       # e.g. ['panda_joint1', … , 'panda_joint7']
            js.position  = goal_radians   # your 7 start angles in radians

            # latch=True means RViz will get it even if it connects after you publish once
            pub.publish(js)
            rospy.loginfo("Published start joint state, exiting.")
            

    def print_available_planners(self):
        
        with open(self._ros_config) as f:
            ros_cfg = yaml.safe_load(f)
            
        moveit_config_pkg = ros_cfg["moveit_config_pkg"]
        
        import rospkg
        # 1) load ros_config.yaml to locate your moveit config package
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(moveit_config_pkg)
        
        # 2) load ompl_planning.yaml
        yaml_path = os.path.join(pkg_path, "config", "ompl_planning.yaml")
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)
        
        # 3) extract the planner IDs
        planner_ids = list(cfg["planner_configs"].keys())
        print("Available planner IDs:")
        for planner in planner_ids:
            print("  ", planner)


    
if __name__ == '__main__':
    # save_moveit_plan()
    start_degrees = [165, 80, -70, -138, 26, 148, 152]
    goal_degrees = [-33, 54, 23, -101, 113, 100, -72]
    
    this_dir = os.path.dirname(os.path.abspath(__file__))
    result_dir = this_dir + "/Data"
    
    # Print abailable planners
    ros_config_file = this_dir + "/config/config_ros.yaml"
    
        
    exp_cfg_file = this_dir + "/config/config.yaml"
    with open(exp_cfg_file) as f:
        cfg = yaml.safe_load(f)
        
    
    baseline = BaselinePlanner(ros_config_file, exp_cfg_file, planner_id='RRTConnect')
    baseline.print_available_planners()
    
    # Do the plan
    baseline.plan_from_start_to_goal_joints(start_degrees, 
                                            goal_degrees,
                                            result_dir=result_dir)
    
    pos_trj = read_plan_json_to_numpy(result_dir + "/plan_trj.yaml")
    print("pos_trj.shape: ", pos_trj.shape)
    
