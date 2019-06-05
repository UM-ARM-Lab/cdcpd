import rospy
import sys
sys.path.append('/home/deformtrack/dev/cdcpd')
from cdcpd.tf_utils import TF2Wrapper
import tf2_ros
from geometry_msgs.msg import TransformStamped

pub_left = rospy.Publisher("/left_gripper/prior", TransformStamped, queue_size=10)
pub_right = rospy.Publisher("/right_gripper/prior", TransformStamped, queue_size=10)

rospy.init_node('gripper_tf_node')
tf_wrapper = TF2Wrapper()
def getGripperTransform(gripper_name, target_frame, robot_name, stamp):

	if (robot_name=='victor'):
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

	elif (robot_name=="val"):
		fingertip_transform = tf_wrapper.get_transform_ros(parent=target_frame, child=gripper_name)
		return fingertip_transform

def main():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	left_gripper = getGripperTransform(gripper_name=tf_wrapper.GripperTFName(arm_name="left", robot_name="victor"),target_frame='kinect2_victor_head_link', robot_name='victor', stamp=rospy.get_rostime())
    	right_gripper = getGripperTransform(gripper_name=tf_wrapper.GripperTFName(arm_name="right", robot_name="victor"),target_frame='kinect2_victor_head_link', robot_name='victor', stamp=rospy.get_rostime())
    	pub_left.publish(left_gripper)
    	pub_right.publish(right_gripper)
    	rate.sleep()
    #rospy.Subscriber("/kinect2/qhd/points", PointCloud2, callback, queue_size=2) #for rope_fast.bag
    rospy.spin()



main()
