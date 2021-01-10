import unittest

import numpy as np

from arc_utilities.transformation_helper import vector3_to_spherical, spherical_to_vector3


class TestTransformationHelper(unittest.TestCase):

    def test_vector3_to_spherical_zero(self):
        xyz = [0, 0, 0]
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_vector3_to_spherical_case1(self):
        xyz = [1, 0, 0]
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_vector3_to_spherical_case2(self):
        xyz = [0, 1, 0]
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_vector3_to_spherical_case3(self):
        xyz = [0, 0, 1]
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_vector3_to_spherical_case4(self):
        xyz = [0, 1, 1]
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_vector3_to_spherical_random(self):
        rng = np.random.RandomState(0)
        for i in range(100):
            xyz = rng.uniform(-2, 1, size=[3])
            r_phi_theta = vector3_to_spherical(xyz)
            xyz_out = spherical_to_vector3(r_phi_theta)
            np.testing.assert_allclose(xyz, xyz_out, rtol=1)


if __name__ == '__main__':
    np.set_printoptions(suppress=True, precision=6)
    unittest.main()
