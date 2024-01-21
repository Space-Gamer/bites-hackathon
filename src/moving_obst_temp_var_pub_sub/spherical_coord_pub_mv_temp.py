#!/usr/bin/python3

import math
import rospy
import random

from bites_hackathon.msg import spherical_coord_mv_temp

def sph_coord_pub():
    pub = rospy.Publisher('spherical_coord', spherical_coord_mv_temp, queue_size=10)
    rospy.init_node('spherical_coord_pub', anonymous=True)
    rate = rospy.Rate(100) # 100hz

    sq_dat = [[3.1622776601683795, 0.0, 0.3217505543966422], 
              [3.3541019662496847, 0.0, 0.4636476090008061], 
              [3.605551275463989, 0.0, 0.5880026035475676], 
              [4.031128874149275, 0.0, 0.519146114246523], 
              [4.47213595499958, 0.0, 0.4636476090008061], 
              [4.272001872658765, 0.0, 0.35877067027057225], 
              [4.123105625617661, 0.0, 0.24497866312686414], 
              [3.640054944640259, 0.0, 0.2782996590051114]]
    
    t_lst = [i for i in range(35, 50)]
    
    
    for _ in range(2):
        for j in range(1000):
            if j % 100 == 0:
                current_temp = random.choice(t_lst)
            factor = 2*math.pi / 1000
            for i in range(len(sq_dat)):
                msg = spherical_coord_mv_temp()
                sq_dat[i][1] = sq_dat[i][1] + factor
                msg.rad, msg.azi, msg.ele = sq_dat[i]
                msg.temperature = current_temp
                msg.index = j
                rospy.loginfo(msg)
                if current_temp in range(45, 51) and j % 50 == 0:
                    pub.publish(msg)
                elif current_temp in range(41, 45) and j % 20 == 0:
                    pub.publish(msg)
                elif current_temp in range(35, 41) and j % 5 == 0:
                    pub.publish(msg)
                    
            rate.sleep()
           

if __name__ == '__main__':
    try:
        sph_coord_pub()
    except rospy.ROSInterruptException:
        pass