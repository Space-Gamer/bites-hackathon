#!/usr/bin/python3

import math
import rospy

from bites_hackathon.msg import spherical_coord

def sph_coord_pub():
    pub = rospy.Publisher('spherical_coord', spherical_coord, queue_size=10)
    rospy.init_node('spherical_coord_pub', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    rad = 1.0
    azi = math.pi/2
    ele = math.pi/2
    for _ in range(10):
        rad = rad + 0.1
        azi = azi + 0.1
        ele = ele + 0.1
        msg = spherical_coord()
        msg.rad = rad
        msg.azi = azi
        msg.ele = ele
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        sph_coord_pub()
    except rospy.ROSInterruptException:
        pass