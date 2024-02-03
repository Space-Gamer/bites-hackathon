#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from utils import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def sph_coord_cb(msg):
    global marker_pub

    marker = Marker()
    marker.header = msg.header
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.r = 1.0
    marker.color.a = 1.0

    #Parsing cloud data
    for p in pc2.read_points(msg, field_names=("rad", "azi", "ele"), skip_nans=True):
        rad, azi, ele = p
        # Spherical to cartesian
        xyz_point = rad_az_ele_to_xyz(rad, azi, ele)

        point = Point()
        point.x = xyz_point[0]
        point.y = xyz_point[1]
        point.z = xyz_point[2]
        marker.points.append(point)

    marker_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("rviz_visualizer", anonymous=True)
    print("rviz_visualizer node started")

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
    rospy.Subscriber("/spherical_coord", PointCloud2, sph_coord_cb)
    rospy.spin()


