#! /usr/bin/env python
import time
import unittest

import rosnode
import rospy
from arc_utilities import ros_init


class TestRosInit(unittest.TestCase):

    def test_ros_init(self):
        name = "test_node_name_" + str(int(time.time()))

        names = rosnode.get_node_names()
        self.assertNotIn("/" + name, names)

        ros_init.rospy_and_cpp_init(name)
        rospy.sleep(1)

        names = rosnode.get_node_names()
        self.assertIn("/" + name, names)
        self.assertIn("/cpp_" + name, names)

        ros_init.shutdown()

        rospy.sleep(1)
        # this will still exist, shutdown only shut's down the internal C++ node
        # WRONG --> self.assertNotIn("/" + name, rosnode.get_node_names())
        self.assertNotIn("/cpp_" + name, rosnode.get_node_names())


if __name__ == '__main__':
    unittest.main()
