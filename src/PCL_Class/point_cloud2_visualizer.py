#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from utils import *


def sph_coord_cb(msg):
    global cloud_pub

    cloud_points = []
    cloud_header = msg.header

    #Parsing cloud data
    for p in pc2.read_points(msg, field_names=("rad", "azi", "ele"), skip_nans=True):
        rad, azi, ele = p
        # Spherical to cartesian
        xyz_point = rad_az_ele_to_xyz(rad, azi, ele)
        cloud_points.append([xyz_point[0], xyz_point[1], xyz_point[2]])

    cloud_msg = pc2.create_cloud_xyz32(header=cloud_header, points=cloud_points)
    cloud_pub.publish(cloud_msg)

if __name__ == "__main__":
    rospy.init_node("rviz_visualizer", anonymous=True)
    print("rviz_visualizer node started")

    cloud_pub = rospy.Publisher("/spherical_point_cloud", PointCloud2, queue_size=2)
    rospy.Subscriber("/spherical_coord", PointCloud2, sph_coord_cb)
    rospy.spin()
