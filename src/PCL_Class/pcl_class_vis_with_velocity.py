#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from utils import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import time

prev_frame = None
prev_time = None
marker_pub = None

def sph_coord_cb(msg):
    global prev_frame, prev_time

    if prev_frame is None:
        prev_frame = msg
        prev_time = time.time()
        return

    current_time = time.time()
    time_interval = current_time - prev_time

    num_points = 0
    total_distance = 0
    prev_point = None

    for p in pc2.read_points(msg, field_names=("rad", "azi", "ele"), skip_nans=True):
        rad, azi, ele = p

        # Spherical to cartesian
        xyz_point = rad_az_ele_to_xyz(rad, azi, ele)

        if prev_point is not None:
            distance = np.linalg.norm(np.array(xyz_point) - np.array(prev_point))
            total_distance += distance

        prev_point = xyz_point

    # Calculate speed in meters per second
    if time_interval > 0:
        speed_mps = total_distance / time_interval
        print("Estimated speed:", speed_mps, "m/s")
    else:
        rospy.logwarn("Time interval is zero. Cannot calculate speed.")

    visualize_point_cloud(msg)

    # Update prev_time for next frame
    prev_time = current_time


def visualize_point_cloud(msg):
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

    # Parsing cloud data
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
    rospy.init_node("rviz_visualizer_with_speed", anonymous=True)
    print("rviz_visualizer_with_speed node started")

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
    rospy.Subscriber("/spherical_coord", PointCloud2, sph_coord_cb)
    rospy.spin()
