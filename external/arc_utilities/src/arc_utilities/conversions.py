from typing import Union, List, Dict

import re
import numpy as np

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PointStamped
from tf.transformations import quaternion_from_euler


def parse_file_size(s: str):
    m = re.fullmatch(r"(\d*\.?\d*)?([kKmMgG])?([bB])?", s)
    if m:
        quantity = float(m.group(1))
        unit = m.group(2)
        if unit is None:
            unit_multiplier = 1
        elif unit in ['k', 'K']:
            unit_multiplier = 1e3
        elif unit in ['m', 'M']:
            unit_multiplier = 1e6
        elif unit in ['g', 'G']:
            unit_multiplier = 1e9
        else:
            raise ValueError("invalid unit. you should fix the regex to catch this.")
        bytes = int(np.round(quantity * unit_multiplier))
        return bytes
    else:
        raise ValueError("string did not match regex")


def normalize_quaternion(quaternion: Union[List[int], Quaternion]):
    if isinstance(quaternion, Quaternion):
        norm = np.linalg.norm(np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w]))
        quaternion.x = quaternion.x / norm
        quaternion.y = quaternion.y / norm
        quaternion.z = quaternion.z / norm
        quaternion.w = quaternion.w / norm
        return quaternion
    elif isinstance(quaternion, list):
        norm = np.linalg.norm(np.array(quaternion))
        quaternion.x = quaternion.x / norm
        quaternion.y = quaternion.y / norm
        quaternion.z = quaternion.z / norm
        quaternion.w = quaternion.w / norm
        return quaternion
    else:
        raise NotImplementedError()


def convert_to_pose_msg(pose: Union[List, Pose, PoseStamped]) -> PoseStamped:
    if isinstance(pose, PoseStamped):
        pose_out = pose
    elif isinstance(pose, Pose):
        pose_out = PoseStamped()
        pose_out.pose = pose
        pose_out.header.stamp = rospy.Time.now()
    elif isinstance(pose, list):
        if len(pose) == 6:
            pose_out = PoseStamped()
            pose_out.header.stamp = rospy.Time.now()
            pose_out.pose.position.x = pose[0]
            pose_out.pose.position.y = pose[1]
            pose_out.pose.position.z = pose[2]
            q = quaternion_from_euler(pose[3], pose[4], pose[5])
            pose_out.pose.orientation.x = q[0]
            pose_out.pose.orientation.y = q[1]
            pose_out.pose.orientation.z = q[2]
            pose_out.pose.orientation.w = q[3]
        elif len(pose) == 7:
            pose_out = PoseStamped()
            pose_out.header.stamp = rospy.Time.now()
            pose_out.pose.position.x = pose[0]
            pose_out.pose.position.y = pose[1]
            pose_out.pose.position.z = pose[2]
            pose_out.pose.orientation.x = pose[3]
            pose_out.pose.orientation.y = pose[4]
            pose_out.pose.orientation.z = pose[5]
            pose_out.pose.orientation.w = pose[6]
        else:
            raise ValueError("pose as a list must be either x,y,z,r,p,y or x,y,z,qz,qy,qz,qw")
    else:
        raise NotImplementedError("cannot convert from type {}".format(type(pose)))

    return pose_out


def convert_to_position(position):
    if isinstance(position, Point):
        out_position = position
    elif isinstance(position, PointStamped):
        out_position = position.point
    else:
        out_position = Point()
        out_position.x = position[0]
        out_position.y = position[1]
        out_position.z = position[2]
    return out_position


def convert_to_positions(delta_positions: Dict):
    out_delta_positions = {}
    for k, v in delta_positions.items():
        out_delta_positions[k] = convert_to_position(v)
    return out_delta_positions
