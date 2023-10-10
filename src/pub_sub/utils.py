import math


def rad_az_ele_to_xyz(rad, az, ele):
    """
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
    """
    x = rad * math.cos(ele) * math.cos(az)
    y = rad * math.cos(ele) * math.sin(az)
    z = rad * math.sin(ele)
    return x, y, z
