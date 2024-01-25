#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <bites_hackathon/spherical_coord_mv_temp.h>

#include "utils.h"  // Assuming utils.h contains the necessary function declarations

std::vector<geometry_msgs::Point> cartesian_pts;
//cartesian_pts: name of the variable. A vector that holds instances of geometry_msgs::Point

int index = 0;
int temperature = 0;

visualization_msgs::MarkerArray rviz_pts(const std::vector<geometry_msgs::Point>& pts) {
    visualization_msgs::MarkerArray markers;

    if (pts.empty()) {
        return markers;
    }

    // Temperature Marker
    visualization_msgs::Marker temperature_marker;
    temperature_marker.header.stamp = ros::Time::now();
    temperature_marker.id = 0;
    temperature_marker.header.frame_id = "map";
    temperature_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    temperature_marker.text = "Temperature: " + std::to_string(temperature) + " C";
    temperature_marker.scale.x = 0.5;
    temperature_marker.scale.y = 0.5;
    temperature_marker.scale.z = 0.5;
    temperature_marker.color.a = 1.0;
    temperature_marker.color.r = 1.0;
    temperature_marker.color.g = 1.0;
    temperature_marker.color.b = 1.0;

    markers.markers.push_back(temperature_marker);

    for (size_t i = 0; i < pts.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.id = i + 1;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = pts[i].x;
        marker.pose.position.y = pts[i].y;
        marker.pose.position.z = pts[i].z;

        markers.markers.push_back(marker);
    }

    return markers;
}

void sph_coord_cb(const bites_hackathon::spherical_coord_mv_temp::ConstPtr& msg) {
    if (msg->index != index) {
        cartesian_pts.clear();
        index = msg->index;
        temperature = msg->temperature;
    }

    geometry_msgs::Point cartesian_pt;
    cartesian_pt = rad_az_ele_to_xyz(msg->rad, msg->azi, msg->ele);
    cartesian_pts.push_back(cartesian_pt);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rviz_visualizer_mv_temp");
    ROS_INFO("rviz_visualizer node started");

    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 2);
    ros::Subscriber sub = nh.subscribe("/spherical_coord", 2, sph_coord_cb);

    ros::Rate loop_rate(1000);  // Adjust the rate as needed

    while (ros::ok()) {
        marker_pub.publish(rviz_pts(cartesian_pts));
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
