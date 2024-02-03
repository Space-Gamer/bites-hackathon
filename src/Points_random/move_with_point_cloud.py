#!/usr/bin/python3

import math
import rospy
import random
from bites_hackathon.msg import spherical_coord_mv_temp_point_cloud

def sph_coord_pub():
    pub = rospy.Publisher('spherical_coord', spherical_coord_mv_temp_point_cloud, queue_size=10)
    rospy.init_node('spherical_coord_pub', anonymous=True)
    rate = rospy.Rate(10)
    j = 0

    center = [0.0, 0.0, 0.0]  # Location of sensor
    rotation_radius = 5.0  # Rotation radius
    
    t_lst = [i for i in range(35, 101)]

    while not rospy.is_shutdown():

        temperature = random.choice(t_lst)
        num_points = int(200 - (temperature - 35) ** 2)  # Decreasing points with increasing temperature

        for i in range(num_points):  # Number of points in a point cloud

            #Radar fov
            rad = random.uniform(0.1, 0.2)  # radius range
            azi = random.uniform(0.0, 0.25*math.pi)  # Azimuth range (0 to 45 degrees)
            ele = random.uniform(0.0, 0.25*math.pi) 
            
           # Rotation around the center point
            rotation_angle = (j * 0.1) % (2 * math.pi) # Circular motion in azimuth
            rotated_azi = azi + rotation_angle

            # Translate to the center point with rotation_radius
            translated_rad = rad + rotation_radius
            translated_azi = rotated_azi + center[0]
            translated_ele = ele + center[1]

            
            msg = spherical_coord_mv_temp_point_cloud()
            msg.rad, msg.azi, msg.ele = translated_rad, translated_azi, translated_ele
            msg.index = j
            msg.temperature = temperature
            msg.max_temperature = 100
            msg.points  = num_points
            rospy.loginfo(msg)
            pub.publish(msg)
        j += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        sph_coord_pub()
    except rospy.ROSInterruptException:
        pass
