import rospy, geometry_msgs.msg
import open3d as o3d
import numpy as np
from visualization_msgs.msg import Marker

def publish_voxel_grid(grid, topic="/voxel_map", frame="map"):
    pub = rospy.Publisher(topic, Marker, queue_size=1, latch=True)
    rospy.sleep(0.5)

    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns  = "voxels"
    marker.id  = 0
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD

    marker.scale.x = marker.scale.y = marker.scale.z = grid.voxel_size
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.2, 0.7, 1.0, 0.8

    origin = np.asarray(grid.origin)
    vs = grid.voxel_size
    for v in grid.get_voxels():
        centre = origin + (np.array(v.grid_index, float) + 0.5) * vs
        marker.points.append(geometry_msgs.msg.Point(*centre))

    pub.publish(marker)
    rospy.loginfo("Published %d voxels.", len(marker.points))