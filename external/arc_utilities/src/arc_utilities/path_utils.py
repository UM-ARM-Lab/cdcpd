#! /usr/bin/env python

"""
Useful functions for dealing with paths
Unless otherwise noted, paths are a list of waypoints. Often it is useful to store these in a numpy array
"""
<<<<<<< HEAD
=======
from more_itertools import pairwise
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
import pathlib
import sys
from copy import deepcopy

import numpy as np
<<<<<<< HEAD
from more_itertools import pairwise
=======
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default

import rospy
from trajectory_msgs.msg import JointTrajectory


def clamp(num, min_val, max_val):
    return min(max(min_val, num), max_val)


def dist(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def closest_point_to_line_segment(line, point):
    """
    Returns:
    point, alpha
    alpha: (0 to 1) fraction along line segment to closest point
    """
    v_line = np.array(line[1]) - np.array(line[0])
    if np.linalg.norm(v_line) < 10 * sys.float_info.epsilon:
        return line[0], 0

    n_v_line = v_line / np.linalg.norm(v_line)
    v_l0_point = np.array(point) - np.array(line[0])

    alpha = clamp(np.dot(v_l0_point, n_v_line) / np.linalg.norm(v_line), 0, 1)

    p_closest = np.array(line[0]) + alpha * v_line
    return p_closest, alpha


def closest_point(path, query_point):
    """
    Computes the closest point on the path to the query point
<<<<<<< HEAD

    Returns:
    point, ind, alpha
    point: closest point on path to query point
    ind: index of the preceding point of the path to point
=======
    
    Returns:
    point, ind, alpha
    point: closest point on path to query point
    ind: index of the preceeding point of the path to point
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    alpha: The fraction from path[ind] to path[ind+1] where path is
    """
    d_close = dist(path[0], query_point)
    alpha_close = 0
    point_close = path[0]
    ind_close = 0
    for ind in range(len(path) - 1):
        p, alpha = closest_point_to_line_segment([path[ind], path[ind + 1]], query_point)
        d = dist(p, query_point)
        if d < d_close:
            d_close = d
            alpha_close = alpha
            point_close = p
            ind_close = ind
    if alpha_close == 1:
        alpha_close = 0
        ind_close += 1

    return point_close, ind_close, alpha_close


def densify_line(start_point, end_point, max_dist):
    """
<<<<<<< HEAD
    Returns a linear path from start point (exclusive) to end point (inclusive)
=======
    Returns a linear path from start point (exclusive) to end point (inclusive) 
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    with a distance of most max_dist between points
    """
    num_points = int(np.ceil(dist(start_point, end_point) / max_dist))
    s_np = np.array(start_point)
    dir_np = np.array(end_point) - start_point
    return [s_np + dir_np * (idx + 1) / num_points for idx in range(num_points)]


def densify(path, max_dist):
    """
    Returns a path that follows path with distance at most max_dist between points
    """
    if len(path) == 0:
        return path

    new_path = [np.array(path[0])]

    for i in range(1, len(path)):
        new_path = new_path + densify_line(new_path[-1], path[i], max_dist)

    return new_path


def travel_along(path, distance, starting_point=None):
    """
    Travels along the path from the starting point for distance

    Parameters:
    path: path
    distance: total euclidean distance to travel. Negative follows backwards
    starting_point: path traversal starts at the closest point on the path to this point

<<<<<<< HEAD
    Returns:
=======
    Returns: 
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    new_path: subpath which lies completely on original path while following inputs as best as possible
    """
    if starting_point is None:
        starting_point = path[0]

    direction = int(np.sign(distance))
    dist_to_go = abs(distance)

    q, ind, alpha = closest_point(path, starting_point)
    newpath = [q]

    path = np.array(path)

    if alpha != 0:
        ind += 1
        path = np.concatenate((path[0:ind], [q], path[ind:]))

    while dist_to_go > 0:
        if direction == 1 and ind == len(path) - 1:
            return newpath

        if direction == -1 and ind == 0:
            return newpath

        ind = ind + direction
        dist_to_next = dist(q, path[ind])

        if dist_to_next > dist_to_go:
            motion = path[ind] - q
            motion = motion / np.linalg.norm(motion)
            newpath.append(motion * dist_to_go + q)
            break

        q = path[ind]
        newpath.append(q)
        dist_to_go -= dist_to_next

    return newpath


def path_length(path):
    if len(path) == 0:
        return 0

    path = np.array(path)
    q = path[0]
    d = 0
    for ind in range(1, len(path)):
        d += dist(q, path[ind])
        q = path[ind]

    return d


def reverse_trajectory(trajectory: JointTrajectory):
    reversed_trajectory = deepcopy(trajectory)
    reversed_trajectory.points = deepcopy(trajectory.points[::-1])
    reversed_trajectory.points[0].time_from_start = rospy.Duration(0)

    time_from_start = rospy.Duration(0)
    for (pt_next, pt), r_pt in zip(pairwise(trajectory.points[::-1]), reversed_trajectory.points[1:]):
        time_from_start += pt_next.time_from_start - pt.time_from_start
        r_pt.time_from_start = time_from_start
    return reversed_trajectory


def rm_tree(path):
    path = pathlib.Path(path)
    for child in path.glob('*'):
        if child.is_file():
            child.unlink()
        else:
            rm_tree(child)
    path.rmdir()
