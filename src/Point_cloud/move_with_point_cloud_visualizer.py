#!/usr/bin/python3

import rospy

from utils import *
from visualization_msgs.msg import Marker, MarkerArray
from bites_hackathon.msg import spherical_coord_mv_temp_point_cloud

def rviz_pts(pts, temperature,max_temperature,points, marker_pub):
    if len(pts) == 0:
        return MarkerArray()

    markers = MarkerArray()

    # Temperature Marker
    temp_marker = Marker()
    temp_marker.header.stamp = rospy.Time.now()
    temp_marker.id = 0
    temp_marker.header.frame_id = "map"
    temp_marker.type = temp_marker.TEXT_VIEW_FACING
    temp_marker.text = "Temperature: " + str(int(temperature)) + " C"
    temp_marker.scale.x = 0.5
    temp_marker.scale.y = 0.5
    temp_marker.scale.z = 0.5
    temp_marker.color.a = 1.0
    temp_marker.color.r = 1.0
    temp_marker.color.g = 1.0
    temp_marker.color.b = 1.0

    markers.markers.append(temp_marker)

    # Points Marker
    point_marker = Marker()
    point_marker.header.stamp = rospy.Time.now()
    point_marker.id = 700
    point_marker.header.frame_id = "map"
    point_marker.type = temp_marker.TEXT_VIEW_FACING
    point_marker.text = "Points: " + str(int(points))
    point_marker.scale.x = 0.5
    point_marker.scale.y = 0.5
    point_marker.scale.z = 0.5
    point_marker.color.a = 1.0
    point_marker.color.r = 1.0
    point_marker.color.g = 1.0
    point_marker.color.b = 1.0
    point_marker.pose.orientation.w = 1.0
    point_marker.pose.position.x = 0.5
    point_marker.pose.position.y = 0.5
    point_marker.pose.position.z = 2.0

    markers.markers.append(point_marker)

    # Point Cloud Markers
    for i in range(len(pts)):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.id = i + 1
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.scale.x = 0.25  
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        temperature_color = temperature / max_temperature

        # Color Gradient
        marker.color.r = 1.0 - temperature_color
        marker.color.g = temperature_color
        marker.color.b = 0.0

        # Size Variation
        marker.scale.x = 0.25 + 0.1 * temperature_color
        marker.scale.y = 0.25 + 0.1 * temperature_color
        marker.scale.z = 0.25 + 0.1 * temperature_color

        # Transparency
        marker.color.a = 0.5 + 0.5 * (1.0 - temperature_color)
        
        # Position
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = pts[i][0]
        marker.pose.position.y = pts[i][1]
        marker.pose.position.z = pts[i][2]

        markers.markers.append(marker)

    return markers

def sph_coord_cb(msg):
    global cartesian_pts
    global index
    global temperature
    global max_temperature
    global points

    if msg.index != index:
        cartesian_pts = []
        index = msg.index
        points = msg.points
        temperature = msg.temperature
        max_temperature = msg.max_temperature

    cartesian_pts.append([msg.rad, msg.azi, msg.ele,msg])

if __name__ == "__main__":
    cartesian_pts = []
    index = 0
    temperature = 0
    points = 0 
    max_temperature = 100

    rospy.init_node("rviz_visualizer", anonymous=True)
    print("rviz_visualizer node started")

    marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=2)
    rospy.Subscriber("/spherical_coord", spherical_coord_mv_temp_point_cloud, sph_coord_cb)
    while not rospy.is_shutdown():
        marker_pub.publish(rviz_pts(cartesian_pts, temperature,max_temperature,points, marker_pub))
        rospy.sleep(0.001)
