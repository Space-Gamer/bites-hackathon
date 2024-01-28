
#include "ros/ros.h"
#include "bites_hackathon/spherical_coord.h"
#include <sstream>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "spherical_coord_pub");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<bites_hackathon::spherical_coord>("spherical_coord", 10);
    ros::Rate rate(1);  // 1 Hz

    double rad = 1.0;
    double azi = M_PI / 2;
    double ele = M_PI / 2;

    for (int i = 0; i < 10; ++i) {
        rad += 0.1;
        azi += 0.1;
        ele += 0.1;

        bites_hackathon::spherical_coord msg;
        msg.rad = rad;
        msg.azi = azi;
        msg.ele = ele;

        ROS_INFO("Publishing spherical coordinate: rad=%.2f, azi=%.2f, ele=%.2f", msg.rad, msg.azi, msg.ele);
        pub.publish(msg);
        rate.sleep();
    }

    return 0;
}