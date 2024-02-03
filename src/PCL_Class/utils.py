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

def xyz_to_rad_az_ele(x, y, z):
    """
    Convert cartesian coordinates to spherical coordinates.

    Parameters
    ----------
    x : float
        X coordinate.
    y : float
        Y coordinate.
    z : float
        Z coordinate.

    Returns
    -------
    rad : float
        Radius.
    az : float
        Azimuth.
    ele : float
        Elevation.
    """
    rad = math.sqrt(x**2 + y**2 + z**2)
    az = math.atan2(y, x)
    ele = math.asin(z/rad)
    return rad, az, ele
