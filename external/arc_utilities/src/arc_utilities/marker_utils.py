from visualization_msgs.msg import MarkerArray


def scale_marker_array(msg: MarkerArray, s: float):
    for marker in msg.markers:
        marker.scale.x *= s
        marker.scale.y *= s
        marker.scale.z *= s

    return msg
