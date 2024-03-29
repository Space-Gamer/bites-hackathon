#!/usr/bin/python3

import rospy

from utils import *

from visualization_msgs.msg import Marker, MarkerArray
from bites_hackathon.msg import spherical_coord

def rviz_pts(pts, marker_pub):

    if len(pts) == 0:
        return MarkerArray()

    markers = MarkerArray()

    for i in range(len(pts)):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            # marker.action = marker.ADD
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = pts[i][0]
            marker.pose.position.y = pts[i][1]
            marker.pose.position.z = pts[i][2]
            
            markers.markers.append(marker)
    return markers


def sph_coord_cb(msg):
    global cartesian_pts
    
    cartesian_pts.append(rad_az_ele_to_xyz(msg.rad, msg.azi, msg.ele))


if __name__=="__main__":

    cartesian_pts = []
      
    rospy.init_node("rviz_visualizer", anonymous=True)
    print("rviz_visualizer node started")
    marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 2)
    rospy.Subscriber("/spherical_coord", spherical_coord, sph_coord_cb)

    while not rospy.is_shutdown():
        marker_pub.publish(rviz_pts(cartesian_pts, marker_pub))
        rospy.sleep(1)