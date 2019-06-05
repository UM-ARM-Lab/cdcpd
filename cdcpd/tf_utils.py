import rospy
import numpy as np
import time
import tf2_ros

class TF2Wrapper:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_static_broadcasters = []

    def GripperTFName(self, arm_name, robot_name):
    	if(robot_name=="victor"):
    		return "victor_" + arm_name + "_gripper"
    	elif(robot_name=="val"):
    		return name + "gripper_tip"


    def get_transform_ros(self, parent, child, verbose=True,
                      spin_delay=rospy.Duration(secs=0, nsecs=500 * 1000 * 1000), time=rospy.Time()):
        """
        Waits for a transform to become available. Blocks until a transform is available or an exception is raised.
        :param parent: frame name for the parent (see below)
        :param child: frame name for the child (see below)
        :param verbose: If verbose is True, then output messages are sent on rosinfo as the function waits for
                        a transform, otherwise on rosdebug
        :param spin_delay: How long to wait between output messages
        :param time: The timepoint to request a transform at. Defaults to "latest available".
        :return: A matrix representation of the transform (numpy). Returns None if a tf2 exception is raised.
        The notation here follows the following convention:
        p_measured_in_parent = returned_transform * p_measured_in_child
        p_measured_in_target = returned_transform * p_measured_in_source
        """

        try:
            while not self.tf_buffer.can_transform(target_frame=parent, source_frame=child,
                                                   time=time, timeout=spin_delay):
                if rospy.is_shutdown():
                    raise KeyboardInterrupt("ROS has shutdown")
                if verbose:
                    rospy.loginfo("Waiting for TF frames %s and %s", parent, child)
                else:
                    rospy.logdebug("Waiting for TF frames %s and %s", parent, child)

            return self.tf_buffer.lookup_transform(target_frame=parent, source_frame=child, time=time)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No transform available: %s to %s", parent, child)
            return None

