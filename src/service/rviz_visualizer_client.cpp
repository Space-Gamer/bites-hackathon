#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <bites_hackathon/spherical_coord.h>
#include "utils.h"

std::vector<geometry_msgs::Point> cartesian_pts;

visualization_msgs::MarkerArray rviz_pts(const std::vector<geometry_msgs::Point>& pts) {
    if (pts.empty()) {
        return visualization_msgs::MarkerArray();
    }

    visualization_msgs::MarkerArray markers;

    for (size_t i = 0; i < pts.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.id = i;
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

void sph_coord_cb(const bites_hackathon::spherical_coord::ConstPtr& msg) {
    cartesian_pts.push_back(rad_az_ele_to_xyz(msg->rad, msg->azi, msg->ele));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_visualizer_client");
    ROS_INFO("rviz_visualizer node started");

    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 2);
    ros::Subscriber sph_coord_sub = nh.subscribe("/spherical_coord", 1, sph_coord_cb);

    while (ros::ok()) {
        marker_pub.publish(rviz_pts(cartesian_pts));
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    return 0;
}
