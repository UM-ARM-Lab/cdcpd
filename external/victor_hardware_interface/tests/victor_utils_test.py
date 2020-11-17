#!/usr/bin/env python

import unittest
import IPython
from victor_hardware_interface import victor_utils as vu



class TestVictorUtils(unittest.TestCase):

    def test_jvq_conversions(self):
        q = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        jvq = vu.list_to_jvq(q)
        self.assertEqual(jvq.joint_1, 1.0)
        self.assertEqual(jvq.joint_7, 7.0)

        and_back = vu.jvq_to_list(jvq)
        self.assertEqual(len(and_back),7)

        for i in range(len(q)):
            self.assertEqual(q[i], and_back[i]);

        with self.assertRaises(AssertionError):
            vu.list_to_jvq([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

if __name__ == "__main__":
    unittest.main()
        
