#!/usr/bin/env python
import rospy
import sys
sys.path.append('/home/deformtrack/catkin_ws/src/cdcpd/src')
from tf_utils import TF2Wrapper
import tf2_ros
from geometry_msgs.msg import TransformStamped
from ros_wrappers import get_ros_param

rospy.init_node('gripper_tf_node')

pub_left = rospy.Publisher("/left_gripper/prior", TransformStamped, queue_size=10)
pub_right = rospy.Publisher("/right_gripper/prior", TransformStamped, queue_size=10)

use_victor = get_ros_param(param_name="use_victor", default=True)
use_val = get_ros_param(param_name="use_val", default=False)
assert(use_victor ^ use_val)

target_frame = get_ros_param(param_name="~kinect_frame", default='kinect2_tripodA_rgb_optical_frame')
gripper0_name = get_ros_param(param_name="gripper0_name", default='left')
gripper1_name = get_ros_param(param_name="gripper1_name", default='right')

tf_wrapper = TF2Wrapper()
def getGripperTransform(gripper_name, target_frame, stamp):

	if (use_victor):
		palm_transform = tf_wrapper.get_transform_ros(parent=target_frame, child=gripper_name+"_palm_surface")
		fingertipA_transform = tf_wrapper.get_transform_ros(parent=target_frame, child=gripper_name+"_fingerA_dist")
		fingertipB_transform = tf_wrapper.get_transform_ros(parent=target_frame, child=gripper_name+"_fingerB_dist")
		fingertipC_transform = tf_wrapper.get_transform_ros(parent=target_frame, child=gripper_name+"_fingerC_dist")

		tf = TransformStamped()
		tf.header.frame_id = target_frame
		tf.header.stamp = stamp
		tf.child_frame_id = gripper_name + "_fingertip_average"
		tf.transform.translation.x = (fingertipA_transform.transform.translation.x + fingertipB_transform.transform.translation.x + fingertipC_transform.transform.translation.x) / 3.0;
		tf.transform.translation.y = (fingertipA_transform.transform.translation.y + fingertipB_transform.transform.translation.y + fingertipC_transform.transform.translation.y) / 3.0;
		tf.transform.translation.z = (fingertipA_transform.transform.translation.z + fingertipB_transform.transform.translation.z + fingertipC_transform.transform.translation.z) / 3.0;
		tf.transform.rotation = palm_transform.transform.rotation
		return tf

	elif (use_val):
		fingertip_transform = tf_wrapper.get_transform_ros(parent=target_frame, child=gripper_name)
		return fingertip_transform

        else:
            assert(False)

def main():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        stamp = rospy.get_rostime()
    	left_gripper = getGripperTransform(gripper_name=tf_wrapper.GripperTFName(arm_name=gripper0_name), target_frame=target_frame, stamp=stamp)
    	right_gripper = getGripperTransform(gripper_name=tf_wrapper.GripperTFName(arm_name=gripper1_name), target_frame=target_frame, stamp=stamp)
    	pub_left.publish(left_gripper)
    	pub_right.publish(right_gripper)
    	rate.sleep()
    #rospy.Subscriber("/kinect2/qhd/points", PointCloud2, callback, queue_size=2) #for rope_fast.bag
    rospy.spin()



main()
