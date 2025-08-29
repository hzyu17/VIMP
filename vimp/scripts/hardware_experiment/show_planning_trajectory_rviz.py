import yaml
from moveit_commander import MoveGroupCommander,RobotCommander
import rospy

from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def load_yaml_trajectory(yaml_file):
    """Load a MoveIt-dumped trajectory from YAML."""
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    joint_names = data.get('joint_names', [])
    points     = data.get('points', [])

    traj = JointTrajectory()
    traj.joint_names = joint_names

    for pt in points:
        jp = JointTrajectoryPoint()
        # required
        jp.positions = pt.get('positions', [])
        # optional
        if 'velocities' in pt:
            jp.velocities = pt['velocities']
        if 'accelerations' in pt:
            jp.accelerations = pt['accelerations']
        if 'effort' in pt:
            jp.effort = pt['effort']

        # parse time_from_start into a rospy.Duration
        tfs = pt.get('time_from_start', {})
        secs = tfs.get('secs', 0)
        nsecs = tfs.get('nsecs', 0)
        jp.time_from_start = rospy.Duration(secs, nsecs)

        traj.points.append(jp)

    return traj


def publish_to_rviz(joint_traj):
    """Wrap into RobotTrajectory + DisplayTrajectory and publish."""
    rospy.init_node('display_json_trajectory', anonymous=True)
    display_pub = rospy.Publisher(
        "/move_group/display_planned_path",
        DisplayTrajectory,
        queue_size=1
    )
    # wait for publisher to register
    rospy.sleep(0.5)

    # get current robot state for trajectory_start
    robot = RobotCommander()
    current_state = robot.get_current_state()

    # assemble RobotTrajectory
    robot_traj = RobotTrajectory()
    robot_traj.joint_trajectory = joint_traj

    # assemble DisplayTrajectory
    display = DisplayTrajectory()
    display.trajectory_start = current_state
    display.trajectory.append(robot_traj)

    display_pub.publish(display)
    rospy.loginfo("Published trajectory to RViz.")

    
if __name__ == '__main__':
    import os
    this_dir = os.path.dirname(os.path.abspath(__file__))
    # result_dir = this_dir + "/Data/Paper_Result/Baselines"
    result_dir = this_dir + "/Baselines"
    
    pos_trj = load_yaml_trajectory(result_dir + "/RRTConnect_plan_trj.yaml")
    publish_to_rviz(pos_trj)
    # rospy.spin()