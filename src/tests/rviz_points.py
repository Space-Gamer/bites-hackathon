import rospy

from visualization_msgs.msg import Marker, MarkerArray


def rviz_pts(pts, marker_pub):

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

if __name__=="__main__":

    rospy.init_node("rviz_points")
    print("rviz_points node started")
    marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 2)
    pts = [[0.0, 0.0, .0], [1.0, 0.0, 3.0], [2.0, 0.0, -2.0], [3.0, 0.0, 5.0]]
    
    
    while not rospy.is_shutdown():
        marker_pub.publish(rviz_pts(pts, marker_pub))
        rospy.sleep(1)