import yaml
from rosbridge_library.internal import message_conversion as mc
from moveit_commander import MoveGroupCommander,RobotCommander
import rospy
import math

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
import numpy as np

    
def plan_from_current_to_goal_joints(goal_degrees, result_dir='.', planner_id='RRTConnect'):
    rospy.init_node("joint_start_and_goal_example", anonymous=True)

    # 1) Initialize
    robot = RobotCommander()
    group = MoveGroupCommander("panda_arm") 
    group.set_planner_id(planner_id)
        
    names = group.get_active_joints()
    print("Active joints: ", names)
    
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
    plan = group.plan()
    
    if plan:
        traj_msg = plan[1].joint_trajectory
        # convert to plain Python
        traj_dict = mc.extract_values(traj_msg)
        result_file = result_dir + "/plan_trj.yaml"
        
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
    
    # execute
    rospy.loginfo("Planning succeeded, executing …")
    
    
    display_pub = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=1
    )
    rospy.sleep(0.5)
    
    display = DisplayTrajectory()
    display.trajectory_start = robot.get_current_state()
    display.trajectory.append(plan_msg)

    display_pub.publish(display)
    
    # 7) Clean up
    group.clear_pose_targets()


def read_plan_json_to_numpy(plan_trj_file):
    with open(plan_trj_file, "r") as f:
        traj = yaml.safe_load(f)
        
    pos_trajectory = np.array([pt["positions"] for pt in traj["points"]])

    return pos_trajectory


def print_available_planners(moveit_config_pkg):
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
    # goal_degrees = [0.0, 58, 1, -69, 35, 170, -30]
    goal_degrees = [-33, 54, 23, -101, 113, 100, -72]
    import os
    this_dir = os.path.dirname(os.path.abspath(__file__))
    result_dir = this_dir + "/Data"
    
    # Print abailable planners
    ros_config_file = this_dir + "/config/config_ros.yaml"
    with open(ros_config_file) as f:
        ros_cfg = yaml.safe_load(f)
    print_available_planners(ros_cfg["moveit_config_pkg"])
        
    exp_cfg_file = this_dir + "/config/config.yaml"
    with open(exp_cfg_file) as f:
        cfg = yaml.safe_load(f)
    

    Planning_cfg = 
    
    # Do the plan
    plan_from_current_to_goal_joints(goal_degrees, result_dir)
    
    pos_trj = read_plan_json_to_numpy(result_dir + "/plan_trj.yaml")
    print("pos_trj.shape: ", pos_trj.shape)
    
