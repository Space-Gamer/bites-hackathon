#!/usr/bin/python3

import math
import rospy
import random
import pcl
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

def sph_coord_pub():
    pub = rospy.Publisher('spherical_coord', PointCloud2, queue_size=10)
    rospy.init_node('spherical_coord_pub', anonymous=True)
    rate = rospy.Rate(10)
    j = 0

    center = [0.0, 0.0, 0.0]  # Location of sensor
    rotation_radius = 5.0  # Rotation radius
    
    while not rospy.is_shutdown():
        cloud_points = []
        for i in range(200):
            rad = random.uniform(0.1, 0.2)  # radius range
            azi = random.uniform(0.0, 0.25*math.pi)  # Azimuth range (0 to 45 degrees)
            ele = random.uniform(0.0, 0.25*math.pi)  # Elevation range (0 to 45 degrees)

            rotation_angle = (j * 0.1) % (2 * math.pi)  # Circular motion in azimuth
            rotated_azi = azi + rotation_angle

            # Translate to the center point with rotation_radius
            translated_rad = rad + rotation_radius
            translated_azi = rotated_azi + center[0]
            translated_ele = ele + center[1]

            cloud_points.append([translated_rad, translated_azi, translated_ele])

        # Convert point cloud to PCL point cloud object
        pcl_cloud = pcl.PointCloud()
        pcl_cloud.from_list(cloud_points)

        #Voxel grid filtering (downsampling)
        vg = pcl_cloud.make_voxel_grid_filter()
        vg.set_leaf_size(0.01, 0.01, 0.01)
        filtered_cloud = vg.filter()

        #PCL point cloud to ROS PointCloud2 message
        filtered_cloud_points = filtered_cloud.to_list()
        cloud_msg = create_point_cloud2(filtered_cloud_points)
        pub.publish(cloud_msg)

        j += 1
        rate.sleep()

def create_point_cloud2(points):
    fields = [
        PointField(name="rad", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="azi", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="ele", offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    header = rospy.Header(frame_id="map")
    cloud_msg = pc2.create_cloud(header, fields, points)
    return cloud_msg

if __name__ == '__main__':
    try:
        sph_coord_pub()
    except rospy.ROSInterruptException:
        pass

