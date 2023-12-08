#!/usr/bin/python3

import rospy

from utils import *

from visualization_msgs.msg import Marker, MarkerArray
from bites_hackathon.srv import spherical_polar_coord

def rviz_pts(pts):

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


def sph_coord_cb(resp):
    global cartesian_pts
    
    cartesian_pts.append(rad_az_ele_to_xyz(resp.rad, resp.azi, resp.ele))


if __name__=="__main__":

    cartesian_pts = []
    
    rospy.init_node("rviz_visualizer", anonymous=True)
    print("rviz_visualizer node started")

    marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 2)
    
    for i in range(10):
        rospy.wait_for_service('spherical_coord')
        try:
            sph_coord = rospy.ServiceProxy('spherical_coord', spherical_polar_coord)
            resp = sph_coord(i)
            sph_coord_cb(resp)
            marker_pub.publish(rviz_pts(cartesian_pts))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        rospy.sleep(1)