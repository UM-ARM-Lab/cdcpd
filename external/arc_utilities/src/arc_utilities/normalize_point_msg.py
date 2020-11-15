import math

from geometry_msgs.msg import Point


def make_unit_point(x, y, z):
    mag = math.sqrt(x ** 2 + y ** 2 + z ** 2)
    assert (mag > 0.0)
    unit_point = Point()
    unit_point.x = x / mag
    unit_point.y = y / mag
    unit_point.z = z / mag
    return unit_point


def normalize_point(raw_point):
    return make_unit_point(raw_point.x, raw_point.y, raw_point.z)


