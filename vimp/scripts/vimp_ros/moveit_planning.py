import yaml
from rosbridge_library.internal import message_conversion as mc
from moveit_commander import MoveGroupCommander,RobotCommander
import rospy
import copy, math

from moveit_msgs.msg    import RobotState
from sensor_msgs.msg    import JointState
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
    
    
def plan_from_start_to_goal_joints(result_file='panda_plan.yaml'):
    rospy.init_node("joint_start_and_goal_example", anonymous=True)

    # 1) Initialize
    robot = RobotCommander()
    group = MoveGroupCommander("panda_arm")  
    
    names = group.get_active_joints()
    print("Active joints: ", names)
    
    rospy.sleep(0.5)  

    # 2) Define your start and goal joint arrays (length = number of active joints)
    start_joints = [0.0, -0.5, 0.5, -1.0,  1.0,  0.5, 0.0]
    goal_degrees = [0.0, 58, 1, -69, 35, 170, -30]
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
    #    You can either pass a list (ordered as group.get_active_joints())
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

    
    
if __name__ == '__main__':
    # save_moveit_plan()
    plan_from_start_to_goal_joints()