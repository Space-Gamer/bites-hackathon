#!/usr/bin/python3

import math
import rospy
import random
from bites_hackathon.msg import spherical_coord_mv

def sph_coord_pub():
    pub = rospy.Publisher('spherical_coord', spherical_coord_mv, queue_size=10)
    rospy.init_node('spherical_coord_pub', anonymous=True)
    rate = rospy.Rate(1)  # 1hz

    # Define the base square shape
    base_sq_dat = [
        [3.1622776601683795, 0.0, 0.3217505543966422],
        [3.3541019662496847, 0.0, 0.4636476090008061],
        [3.605551275463989, 0.0, 0.5880026035475676],
        [4.031128874149275, 0.0, 0.519146114246523],
        [4.47213595499958, 0.0, 0.4636476090008061],
        [4.272001872658765, 0.0, 0.35877067027057225],
        [4.123105625617661, 0.0, 0.24497866312686414],
        [3.640054944640259, 0.0, 0.2782996590051114]
    ]

    # Number of points in the square
    num_points = len(base_sq_dat)

    for j in range(20):
        factor = 2 * math.pi / num_points

        # Slight randomization for each iteration
        random_factor = random.uniform(0.5, 1.5)

        for i in range(num_points):
            msg = spherical_coord_mv()

            # Apply the randomization factor to the base values
            msg.rad, msg.azi, msg.ele = [val * random_factor for val in base_sq_dat[i]]

            # Apply a fixed azimuth increment
            msg.azi += j * factor

            msg.index = j
            rospy.loginfo(msg)
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        sph_coord_pub()
    except rospy.ROSInterruptException:
        pass
