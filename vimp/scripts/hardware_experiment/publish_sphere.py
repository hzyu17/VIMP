import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg           import Header, ColorRGBA
from geometry_msgs.msg      import Point, Quaternion, Vector3

def publish_sphere():
    rospy.init_node("sphere_marker_publisher")
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)
    rate = rospy.Rate(10)

    # Construct a Marker message
    marker = Marker()
    marker.header = Header(frame_id="world")
    marker.ns        = "my_spheres"
    marker.id        = 0
    marker.type      = Marker.SPHERE
    marker.action    = Marker.ADD

    # Fill in the pose (as true ROS msgs, not dicts)
    marker.pose.position    = Point(1.0, 2.0, 0.5)
    marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    # Set the diameter via scale.x/y/z
    marker.scale = Vector3(0.3, 0.3, 0.3)

    # RGBA color
    marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)

    # 0 = forever
    marker.lifetime = rospy.Duration()

    while not rospy.is_shutdown():
        # Stamp each publish so RViz knows it's fresh
        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_sphere()
    except rospy.ROSInterruptException:
        pass