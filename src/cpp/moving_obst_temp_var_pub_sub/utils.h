#include <cmath>
#include <geometry_msgs/Point.h>
/*
    Convert spherical coordinates to cartesian coordinates.

    Parameters
    ----------
    rad : float
        Radius.
    az : float
        Azimuth.
    ele : float
        Elevation.

    Returns
    -------
    x : float
        X coordinate.
    y : float
        Y coordinate.
    z : float
        Z coordinate.
*/








geometry_msgs::Point rad_az_ele_to_xyz(double rad, double az, double ele) {
    geometry_msgs::Point cartesian_point;
    cartesian_point.x = rad * std::cos(ele) * std::cos(az);
    cartesian_point.y = rad * std::cos(ele) * std::sin(az);
    cartesian_point.z = rad * std::sin(ele);
    return cartesian_point;
}
