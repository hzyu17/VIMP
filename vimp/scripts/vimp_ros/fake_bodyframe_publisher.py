import rospy
from geometry_msgs.msg import PoseStamped
import random


def make_random_msg():
    msg = PoseStamped()
    msg.header.frame_id = "world"          # or whatever frame you need
    msg.pose.position.x    = random.random()
    msg.pose.position.y    = 1.0
    msg.pose.position.z    = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    
    return msg

def fake_pose_publisher():
    pub = rospy.Publisher('/B1_pose', PoseStamped, queue_size=10)
    
    rospy.init_node('fake_pose_publisher', anonymous=True)
    
    rate = rospy.Rate(10)
    
    # 5) Loop until shutdown, publishing the same pose
    while not rospy.is_shutdown():
        msg = make_random_msg()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_pose_publisher()
    except rospy.ROSInterruptException:
        pass