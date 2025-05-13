import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg           import Header, ColorRGBA
from geometry_msgs.msg      import Point, Vector3
import numpy as np
from resampling import read_forwardkinematic_from_config
from pathlib import Path
from moveit_commander import PlanningSceneInterface
from moveit_commander import MoveGroupCommander

import yaml


class ForwardKinematicsCollisionPublisher:
    def __init__(self, config_file):       
        hardware_cfg = Path(config_file)
        self._cfg = hardware_cfg
        self._fk = read_forwardkinematic_from_config(hardware_cfg)    

        rospy.init_node("publishing_collision_checking_balls")
        self._scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(1.0)
        
    
    def publish_fk_checking_spheres(self, moveit_group="panda_arm"):
        group = MoveGroupCommander(moveit_group) 
            
        names = group.get_active_joints()
        print("Active joints: ", names)
        
        # 3) Build a RobotState for your custom start    
        q_current = np.array(group.get_current_joint_values())

        checking_pts = self._fk.compute_sphere_centers(q_current)
        
        radius = self._fk.radii
        
        self.publish_spheres(checking_pts, radius)
                

    def publish_spheres(self, centers, radii=1.0):
        """
        Publish a collection of centers. 

        Args:
            centers (np.array): Shaped: (N,3)
        """
        # rospy.init_node("sphere_marker_publisher")
        pub = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
        rate = rospy.Rate(10)
        
        N = centers.shape[0]
        
        if np.isscalar(radii):
            radii = np.full(N, float(radii), dtype=float)
        else:
            radii = np.asarray(radii, dtype=float)
            if radii.shape != (N,):
                raise ValueError(f"radii must be scalar or shape {(N,)}, got {radii.shape}")
        
        marr = MarkerArray()
        for i, ((x, y, z), r) in enumerate(zip(centers, radii)):
            m = Marker()
            m.header = Header(frame_id="world")
            m.ns = "variable_spheres"
            m.id = i                    # unique ID
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            # Position
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
            m.pose.orientation.w = 1.0  # no rotation
            
            # Scale = diameter
            d = 2.0 * float(r)
            m.scale.x = d
            m.scale.y = d
            m.scale.z = d
            # Color (optional: do whatever you like per sphere)
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 0.5
            # Infinite lifetime (stay until overwritten or removed)
            m.lifetime = rospy.Duration(0)
            marr.markers.append(m)


        # # Construct a Marker message
        # marker = Marker()
        # marker.header = Header(frame_id="world")
        # marker.ns        = "my_spheres"
        # marker.id        = 0
        # marker.type      = Marker.SPHERE_LIST
        # marker.action    = Marker.ADD
        
        # for (x, y, z) in centers:
        #     pt = Point(x=float(x), y=float(y), z=float(z))
        #     marker.points.append(pt)

        # # Set the diameter via scale.x/y/z
        # marker.scale = Vector3(0.3, 0.3, 0.3)

        # # RGBA color
        # marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)

        # # 0 = forever
        # marker.lifetime = rospy.Duration()

        # while not rospy.is_shutdown():
        # for i in range(5):
        #     # Stamp each publish so RViz knows it's fresh
        #     marker.header.stamp = rospy.Time.now()
        #     pub.publish(marker)
        #     rate.sleep()
        
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            for m in marr.markers:
                m.header.stamp = now
            pub.publish(marr)
            rate.sleep()


if __name__ == "__main__":    
    config_file = Path(__file__).parent / 'config' / 'franka_hardware.yaml'
    
    pub = ForwardKinematicsCollisionPublisher(config_file)
    pub.publish_fk_checking_spheres(moveit_group="panda_arm")