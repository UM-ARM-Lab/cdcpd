#! /usr/bin/env python

import unittest
import numpy as np

import rospy
from arc_utilities import path_utils as pu
import IPython

from arc_utilities.path_utils import reverse_trajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TestPathUtils(unittest.TestCase):
    def assertApprox(self, a, b, eps=0.00001):
        d = np.linalg.norm(np.array(a) - np.array(b))
        self.assertTrue(d < eps)

    def test_closest_point_to_line_segment_simple(self):
        l0 = [-1, 1]
        l1 = [1, 1]
        line = [l0, l1]
        p, a = pu.closest_point_to_line_segment(line, [0, 0])
        self.assertApprox(a, 0.5)
        self.assertApprox(p, [0, 1])

        p, a = pu.closest_point_to_line_segment(line, [-1, 0])
        self.assertApprox(a, 0.0)
        self.assertApprox(p, [-1, 1])

        p, a = pu.closest_point_to_line_segment(line, [-1.1, 0])
        self.assertApprox(a, 0.0)
        self.assertApprox(p, [-1, 1])

        for true, computed in zip([-1, 1], p):
            self.assertEqual(true, computed)

        p, a = pu.closest_point_to_line_segment(line, [0.9, 1])
        self.assertApprox(a, 0.95)
        self.assertApprox(p, [0.9, 1])

        p, a = pu.closest_point_to_line_segment([[0, 0], [0, 0]], [10, 10])
        self.assertApprox(p, [0, 0])

    def test_closest_point_to_line_segment_large(self):
        l0 = np.array([1, 2, 3, 4, 5, 6, 7])
        l1 = -1 * l0
        line = [l0, l1]

        p, a = pu.closest_point_to_line_segment(line, [0, 0, 0, 0, 0, 0, 0])
        self.assertApprox(a, 0.5)
        self.assertApprox(p, [0, 0, 0, 0, 0, 0, 0])

        p, a = pu.closest_point_to_line_segment(line, [-7, -6, -5, 0, 3, 2, 1])
        self.assertApprox(a, 0.5)
        self.assertApprox(p, [0, 0, 0, 0, 0, 0, 0])

        p, a = pu.closest_point_to_line_segment(line, [9, 9, 9, 9, 9, 9, 9])
        self.assertApprox(a, 0.0)
        self.assertApprox(p, [1, 2, 3, 4, 5, 6, 7])

    def test_closest_point_to_path(self):
        path = [[-1, 0], [0, 0], [0, 1], [1, 10]]

        p, ind, alpha = pu.closest_point(path, [0, 0])
        self.assertEqual(ind, 1)
        self.assertApprox(p, [0, 0])
        self.assertApprox(alpha, 0)

        p, ind, alpha = pu.closest_point(path, [-10, 1])
        self.assertEqual(ind, 0)
        self.assertApprox(p, [-1, 0])
        self.assertApprox(alpha, 0)

        p, ind, alpha = pu.closest_point(path, [10, 10])
        self.assertEqual(ind, 3)
        self.assertApprox(p, [1, 10])
        self.assertApprox(alpha, 0)

        p, ind, alpha = pu.closest_point(path, [.1, .5])
        self.assertEqual(ind, 1)
        self.assertApprox(p, [0, .5])
        self.assertApprox(alpha, .5)

    def test_travel_along(self):
        path = [[-1, 0], [0, 0], [0, 1], [1, 10]]

        newpath = pu.travel_along(path, 2.0, [-.5, .1])
        self.assertApprox(pu.path_length(newpath), 2.0)

        newpath = pu.travel_along(path, -2.0, [5, 5])
        self.assertApprox(pu.path_length(newpath), 2.0)

        newpath = pu.travel_along(path, -2.0, [-.5, .1])
        self.assertApprox(pu.path_length(newpath), .5)

        newpath = pu.travel_along(path, 100000, [-1.1, 0])
        self.assertApprox(pu.path_length(newpath), pu.path_length(newpath))

        newpath = pu.travel_along(path, 100000)
        self.assertTrue(np.all(np.equal(path, newpath)))

    def test_path_length(self):
        path = [[0, 1], [0, 2], [2, 2], [3, 3]]
        self.assertApprox(pu.path_length(path), 3 + np.sqrt(2))

    def test_densify_line(self):
        start = [1.0, 2.0, 3.0]
        end = [1.0, 2.0, 4.0]
        dense = pu.densify_line(start, end, .5)
        expected = [[1.0, 2.0, 3.5], [1.0, 2.0, 4.0]]
        for i in range(2):
            for j in range(3):
                self.assertApprox(dense[i][j], expected[i][j])

    def test_densify(self):
        path = [[1.0, 2.0],
                [1.0, 3.0],
                [1.0, 3.1],
                [1.6, 3.2]]
        dense = pu.densify(path, .5)
        expected = [[1.0, 2.0],
                    [1.0, 2.5],
                    [1.0, 3.0],
                    [1.0, 3.1],
                    [1.3, 3.15],
                    [1.6, 3.2]]
        self.assertEqual(len(expected), len(dense))
        for i in range(len(expected)):
            for j in range(2):
                self.assertApprox(dense[i][j], expected[i][j])

    def test_reverse(self):
        traj = JointTrajectory()
        traj.points.append(JointTrajectoryPoint(positions=[0.1], time_from_start=rospy.Duration(0)))
        traj.points.append(JointTrajectoryPoint(positions=[0.2], time_from_start=rospy.Duration(0.1)))
        traj.points.append(JointTrajectoryPoint(positions=[0.4], time_from_start=rospy.Duration(0.3)))
        traj.points.append(JointTrajectoryPoint(positions=[0.5], time_from_start=rospy.Duration(0.4)))
        rev_traj = reverse_trajectory(traj)
        self.assertEqual(rev_traj.points[0].time_from_start.to_sec(), 0)
        self.assertEqual(rev_traj.points[1].time_from_start.to_sec(), 0.1)
        self.assertEqual(rev_traj.points[2].time_from_start.to_sec(), 0.3)
        self.assertEqual(rev_traj.points[3].time_from_start.to_sec(), 0.4)

        self.assertEqual(rev_traj.points[0].positions[0], 0.5)
        self.assertEqual(rev_traj.points[1].positions[0], 0.4)
        self.assertEqual(rev_traj.points[2].positions[0], 0.2)
        self.assertEqual(rev_traj.points[3].positions[0], 0.1)


if __name__ == '__main__':
    unittest.main()
