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

    # Calculate the center of the square
    center = [sum([point[0] for point in base_sq_dat]) / len(base_sq_dat),
              sum([point[1] for point in base_sq_dat]) / len(base_sq_dat),
              sum([point[2] for point in base_sq_dat]) / len(base_sq_dat)]

    # Number of points in the square
    num_points = len(base_sq_dat)

    # Outer loop for iterations (20 in this case)
    for j in range(20):
        # Calculate the azimuth increment based on the number of points
        factor = 2 * math.pi / num_points

        # Slight randomization for each iteration
        random_factor = random.uniform(0.5, 1.5)

        # Inner loop for each point in the square
        for i in range(num_points):
            # Create a message object
            msg = spherical_coord_mv()

            # Copy azimuth and elevation from the base square
            msg.azi, msg.ele = base_sq_dat[i][1], base_sq_dat[i][2]

            # Calculate the radius from the center and apply randomization
            radius_from_center = base_sq_dat[i][0] * random_factor
            msg.rad = center[0] + radius_from_center

            # Additional logging for radius and elevation
            rospy.loginfo(f"Iteration {j}, Point {i}: Radius={msg.rad}, Azimuth={msg.azi}, Elevation={msg.ele}")

            # Apply a fixed azimuth increment multiplied by the outer loop index
            msg.azi += j * factor

            # Set the index and publish the message
            msg.index = j
            pub.publish(msg)

        # Sleep to control the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        sph_coord_pub()
    except rospy.ROSInterruptException:
        pass
