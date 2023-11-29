#include <ros/ros.h>
#include <bites_hackathon/spherical_polar_coord.h>

bool handle_sph_coord(bites_hackathon::spherical_polar_coord::Request& req,
                      bites_hackathon::spherical_polar_coord::Response& res) {
    int index = req.index;
    ROS_INFO("Returning spherical polar coordinates for index %d", index);
    
    res.radius = 1.0;
    res.azimuth = (M_PI / 10) * index;
    res.elevation = (M_PI / 10) * index;

    return true;
}

void sph_coord_pub() {
    ros::init(argc, argv, "spherical_coord_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("spherical_coord", handle_sph_coord);
    ROS_INFO("Ready to return spherical polar coordinates.");
    ros::spin();
}

int main(int argc, char** argv) {
    try {
        sph_coord_pub();
    } catch (ros::ROSInterruptException& e) {
        ROS_ERROR("An exception occurred: %s", e.what());
    }

    return 0;
}
