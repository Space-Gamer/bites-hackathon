#include <cmath>
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
std::tuple<double, double, double> rad_az_ele_to_xyz(double rad, double az, double ele) {
    double x = rad * std::cos(ele) * std::cos(az);
    double y = rad * std::cos(ele) * std::sin(az);
    double z = rad * std::sin(ele);
    return std::make_tuple(x, y, z);
}
