#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <bites_hackathon/spherical_coord_mv_temp.h>
#include <sstream>
#include "geometry_msgs/Point.h"

#include "utils.h"  // Assuming utils.h contains the necessary function declarations

std::vector<geometry_msgs::Point> cartesian_pts;
//cartesian_pts: name of the global variable. 
//std::vector - a dynamic array container
//geometry_msgs::Point is a message type
//A vector that holds instances of geometry_msgs::Point


int my_index = 0;
int temperature = 0;

std::tuple<double, double, double> color = std::make_tuple(0, 0, 0);//color=(0,0,0)


visualization_msgs::MarkerArray rviz_pts(const std::vector<geometry_msgs::Point>& pts) {
//function, named rviz_pts, takes a vector of geometry_msgs::Point as input and returns a visualization_msgs::MarkerArray
 
    visualization_msgs::MarkerArray markers; //initializes an empty visualization_msgs::MarkerArray named markers

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
        marker.lifetime = ros::Duration(0.1);
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0;
        // marker.color.r = 1.0;
        // marker.color.g = 0.0;
        // marker.color.b = 0.0;
        marker.color.r = std::get<0>(color);
        marker.color.g = std::get<1>(color);
        marker.color.b = std::get<2>(color);


        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = pts[i].x;
        marker.pose.position.y = pts[i].y;
        marker.pose.position.z = pts[i].z;

        markers.markers.push_back(marker);
    }

    return markers;
}

void sph_coord_cb(const bites_hackathon::spherical_coord_mv_temp::ConstPtr& msg) {
//declares a function named sph_coord_cb that takes a constant reference to a pointer of type bites_hackathon::spherical_coord_mv_temp as an argument. The function doesn't return any value (void)    
    
    if (msg->index != my_index) {
        cartesian_pts.clear();//If the condition in the previous if statement is true, it means that the index has changed. In that case, this line clears the contents of the cartesian_pts vector.
        my_index = msg->index;
        temperature = msg->temperature;
        color = (temperature >= 45) ? std::make_tuple(1, 0, 0) : (temperature > 40) ? std::make_tuple(1, 1, 0) : std::make_tuple(0, 1, 0);
    
    }

    geometry_msgs::Point cartesian_pt;
    cartesian_pt = rad_az_ele_to_xyz(msg->rad, msg->azi, msg->ele);
    cartesian_pts.push_back(cartesian_pt);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rviz_visualizer_mv_temp");
    //ros::init function is used to initialize the ROS library. It takes the command line arguments (argc and argv) and a string argument representing the name of the ROS node. 
    ROS_INFO("rviz_visualizer node started");

    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 2);
    //Declares a publisher object named marker_pub for publishing messages
    //Advertises a topic named "/visualization_marker_array" for publishing messages of type visualization_msgs::MarkerArray
    //2-queue size

    ros::Subscriber sub = nh.subscribe("/spherical_coord", 10, sph_coord_cb);
    //Declares a subscriber object named sub for subscribing to messages
    //Subscribes to the topic "/spherical_coord" for messages of type bites_hackathon::spherical_coord_mv_temp
    //2-queue size

    ros::Rate rate(1000);  // Adjust the rate as needed

    while (ros::ok()) {
        marker_pub.publish(rviz_pts(cartesian_pts));
        ros::spinOnce();
        //loop_rate.sleep();
        rate.sleep();
    }
    

    return 0;
}
