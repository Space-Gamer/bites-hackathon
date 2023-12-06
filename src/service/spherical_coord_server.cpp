#include <ros/ros.h>
#include <bites_hackathon/spherical_polar_coord.h>

bool handle_sph_coord(bites_hackathon::spherical_polar_coord::Request& req,
                      bites_hackathon::spherical_polar_coord::Response& res) {
    int index = req.index;
    ROS_INFO("Returning spherical polar coordinates for index %d", index);
    
    res.rad = 1.0;
    res.azi = (M_PI / 10) * index;
    res.ele = (M_PI / 10) * index;

    return true;
}

void sph_coord_pub() {
    
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("spherical_coord", handle_sph_coord);
    ROS_INFO("Ready to return spherical polar coordinates.");
    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "spherical_coord_server");
    
    try {
        sph_coord_pub();
    } catch (std::exception &e) {
        ROS_ERROR("An exception occurred: %s", e.what());
    }

    return 0;
}
