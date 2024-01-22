#!/usr/bin/python3

import math
import rospy
import random
import tkinter as tk
from tkinter import Scale
from std_msgs.msg import String
from bites_hackathon.msg import spherical_coord_mv_temp_point_cloud

class SphericalCoordPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('spherical_coord', spherical_coord_mv_temp_point_cloud, queue_size=10)
        rospy.init_node('spherical_coord_pub', anonymous=True)
        self.rate = rospy.Rate(10)
        self.j = 0

        self.center = [0.0, 0.0, 0.0]
        self.rotation_radius = 5.0
        self.points_density = 0.1
        self.t_lst = [i for i in range(35, 101)]

        self.create_gui()

    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("Temperature Control")

        self.temperature_label = tk.Label(self.root, text="Temperature")
        self.temperature_label.pack()

        self.temperature_slider = Scale(self.root, from_=35, to=100, orient=tk.HORIZONTAL, command=self.update_temperature)
        self.temperature_slider.pack()

    def update_temperature(self, temperature):
        temperature = int(temperature)
        self.generate_point_cloud(temperature)

    def generate_point_cloud(self, temperature):
        num_points = int(200 - (temperature - 35) ** 2)

        for i in range(num_points):
            rad = random.uniform(0.1, 0.5)
            azi = random.uniform(0.0, 0.5 * math.pi)
            ele = random.uniform(0.0, 0.5 * math.pi)

            rotation_angle = (self.j * 0.1) % (2 * math.pi)
            rotated_azi = azi + rotation_angle

            translated_rad = rad + self.rotation_radius
            translated_azi = rotated_azi + self.center[0]
            translated_ele = ele + self.center[1]

            msg = spherical_coord_mv_temp_point_cloud()
            msg.rad, msg.azi, msg.ele = translated_rad, translated_azi, translated_ele
            msg.index = self.j
            msg.temperature = temperature
            msg.max_temperature = 100
            msg.points = num_points
            rospy.loginfo(msg)
            self.pub.publish(msg)

        self.j += 1
        self.rate.sleep()

    def run_gui(self):
        self.root.mainloop()

if __name__ == '__main__':
    try:
        spherical_coord_publisher = SphericalCoordPublisher()
        spherical_coord_publisher.run_gui()  
        rospy.spin()  
    except rospy.ROSInterruptException:
        pass
