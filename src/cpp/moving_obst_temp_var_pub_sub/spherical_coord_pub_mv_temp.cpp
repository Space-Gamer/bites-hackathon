#include "ros/ros.h"
#include "bites_hackathon/spherical_coord_mv_temp.h"
#include <cmath>
#include <cstdlib> // provides rand() function
#include <vector>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spherical_coord_pub_mv_temp");
    ////ros::init function is used to initialize the ROS library. It takes the command line arguments (argc and argv) and a string argument representing the name of the ROS node. 
 
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<bites_hackathon::spherical_coord_mv_temp>("spherical_coord", 10);
    //Declares a publisher object named pub for publishing messages
    //Advertises a topic named "spherical_coord" for publishing messages of type ites_hackathon::spherical_coord_mv_temp
    //10-queue size
    ros::Rate rate(100); // 100hz

    std::vector<std::vector<double>> sq_dat = {
            {3.1622776601683795, 0.0, 0.3217505543966422},
            {3.25, 0.0, 0.39479111969976155},
            {3.3541019662496847, 0.0, 0.4636476090008061},
            {3.473110997362451, 0.0, 0.5280744484263598},
            {3.605551275463989, 0.0, 0.5880026035475676},
            {3.816084380618437, 0.0, 0.5516549825285469},
            {4.031128874149275, 0.0, 0.519146114246523},
            {4.25, 0.0, 0.4899573262537283},
            {4.47213595499958, 0.0, 0.4636476090008061},
            {4.366062299143245, 0.0, 0.4124104415973874},
            {4.272001872658765, 0.0, 0.35877067027057225},
            {4.190763653560053, 0.0, 0.3028848683749714},
            {4.123105625617661, 0.0, 0.24497866312686414},
            {3.881043674065006, 0.0, 0.26060239174734096},
            {3.640054944640259, 0.0, 0.2782996590051114},
            {3.400367627183861, 0.0, 0.2984989315861793}
    
        };
    // declares a 2D vector with std::vector<double> as its inner type, and it's initialized with a list of lists of doubles
    int current_temp;
    std::vector<int> t_lst;//initialize an array
    for (int i = 35; i <= 50; ++i)
    {
        t_lst.push_back(i);
    }

    for (int iter = 0; iter < 10; ++iter)
    {
        for (int j = 0; j < 1000; ++j)
        {
            if (j % 250 == 0)
            {
                current_temp = t_lst[rand() % t_lst.size()];
            //t_lst.size(): Returns the number of elements in the vector t_lst.
            //rand() % t_lst.size(): Generates a random integer in the range [0, t_lst.size() - 1]. This is achieved by taking the remainder when dividing a random integer (rand()) by the size of the vector
            }

            double factor = 2 * M_PI / 1000;
            for (size_t i = 0; i < sq_dat.size(); ++i)
            {
                bites_hackathon::spherical_coord_mv_temp msg;
                sq_dat[i][1] = sq_dat[i][1] + factor;
                msg.rad = sq_dat[i][0];
                msg.azi = sq_dat[i][1];
                msg.ele = sq_dat[i][2];
                msg.temperature = current_temp;
                msg.index = j;
                ROS_INFO_STREAM(msg);

                if (current_temp >= 45 && current_temp <= 50 && j % 50 == 0 && i%4==0)
                {
                    pub.publish(msg);
                }
                else if (current_temp >= 41 && current_temp < 45 && j % 20 == 0 && i%2==0)
                {
                    pub.publish(msg);
                }
                else if (current_temp >= 35 && current_temp < 41 && j % 5 == 0)
                {
                    pub.publish(msg);
                }
            }

            rate.sleep();
        }
    }

    return 0;
}
