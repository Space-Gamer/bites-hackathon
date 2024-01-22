#!/usr/bin/python3

import math
import rospy
import random
from bites_hackathon.msg import spherical_coord_mv

def sph_coord_pub():
    pub = rospy.Publisher('spherical_coord', spherical_coord_mv, queue_size=10)
    rospy.init_node('spherical_coord_pub', anonymous=True)
    rate = rospy.Rate(10)
    j = 0

    center = [0.0, 0.0, 0.0]  # location of sensor
    rotation_radius = 8.0  #rotation radius
    points_density = 1  #density of points

    while not rospy.is_shutdown():
        for i in range(200):  #number of points in a point cloud
            
            rad = random.uniform(0.1, 0.2)  # radius range
            azi = random.uniform(0.0, 0.25*math.pi)  # Azimuth range (0 to 45 degrees)
            ele = random.uniform(0.0, 0.25*math.pi)  # Elevation range (0 to 45 degrees) #Basically sensor range can be defined

            # Rotation around the center point
            rotation_angle = (j * 0.1) % (2 * math.pi)  # Circular motion in azimuth
            rotated_azi = azi + rotation_angle

            # Translate to the center point with rotation_radius
            translated_rad = rad + rotation_radius
            translated_azi = rotated_azi + center[0]
            translated_ele = ele + center[1]

            msg = spherical_coord_mv()
            msg.rad, msg.azi, msg.ele = translated_rad, translated_azi, translated_ele
            msg.index = j
            rospy.loginfo(msg)
            pub.publish(msg)
        j += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        sph_coord_pub()
    except rospy.ROSInterruptException:
        pass
