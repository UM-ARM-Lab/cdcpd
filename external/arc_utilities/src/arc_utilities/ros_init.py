import roscpp_initializer

import rospy


def rospy_and_cpp_init(name):
    """ Use this function any time you want to call into C++ ROS code.
      You can use this in place of the moveit roscpp_initializer, and """
    roscpp_initializer.init_node("cpp_" + name, [], disable_signals=True)
    rospy.init_node(name)


def shutdown():
    """ ensures the C++ node handle is shut down cleanly. It's good to call this a the end of any program
      where you called rospy_and_cpp_init """
    roscpp_initializer.shutdown()
