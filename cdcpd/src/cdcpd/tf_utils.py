import rospy
import tf2_ros
from cdcpd.ros_wrappers import get_ros_param
from tf.transformations import *

class TF2Wrapper:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_static_broadcasters = []

    def GripperTFName(self, arm_name):
        use_victor = get_ros_param(param_name="use_victor", default=True)
        use_val = get_ros_param(param_name="use_val", default=False)

        if not (use_victor ^ use_val):
            raise ValueError("One (and only one) of use_victor or use_val must be set on the parameter server")

        if use_victor:
            return "victor_" + arm_name + "_gripper"
        if use_val:
            return arm_name + "gripper_tip"

        raise TypeError("Logic error: this code should be unreachable")


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

    def TransformToMatrix(self, old_transform):
        [translation, quaternion] = self.ComponentsFromTransform(old_transform)
        tfmatrix = self.BuildMatrix(translation, quaternion)
        return tfmatrix

    def ComponentsFromTransform(self, old_transform):
        translation = [old_transform.transform.translation.x, old_transform.transform.translation.y, old_transform.transform.translation.z]
        quaternion = [old_transform.transform.rotation.x, old_transform.transform.rotation.y, old_transform.transform.rotation.z, old_transform.transform.rotation.w]
        return [translation, quaternion]

    def BuildMatrix(self, translation, quaternion):
        tfmatrix = quaternion_matrix(quaternion)
        tfmatrix[0][3] = translation[0]
        tfmatrix[1][3] = translation[1]
        tfmatrix[2][3] = translation[2]
        return tfmatrix

