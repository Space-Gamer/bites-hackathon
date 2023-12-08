#!/usr/bin/python3

import math
import rospy

from bites_hackathon.srv import spherical_polar_coord, spherical_polar_coordResponse

def handle_sph_coord(req):
    index = req.index
    print("Returning spherical polar coordinates for index %s" % index)
    return spherical_polar_coordResponse(1.0, ((math.pi)/10)*index, ((math.pi)/10)*index)

def sph_coord_pub():
    rospy.init_node('spherical_coord_server', anonymous=True)
    service = rospy.Service('spherical_coord', spherical_polar_coord, handle_sph_coord)
    print("Ready to return spherical polar coordinates.")
    rospy.spin()

if __name__ == '__main__':
    try:
        sph_coord_pub()
    except rospy.ROSInterruptException:
        pass