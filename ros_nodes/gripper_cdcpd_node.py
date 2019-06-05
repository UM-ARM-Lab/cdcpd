import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
import ros_numpy
from cdcpd.cdcpd import CDCPDParams, ConstrainedDeformableCPD
from cdcpd.cpd import CPDParams
from cdcpd.optimizer import PriorConstrainedOptimizer
from cdcpd.geometry_utils import build_line
from cdcpd.cv_utils import chroma_key_rope
from cdcpd.prior import UniformPrior
from geometry_msgs.msg import TransformStamped
from copy import deepcopy
import time
from threading import Lock


class Listener:
    def __init__(self, topic_name, topic_type, wait_for_data=False):
        """
        Listener is a wrapper around a subscriber where the callback simply records the latest msg.
        Listener does not consume the message
            (for consuming behavior, use the standard ros callback pattern)
        Listener does not check timestamps of message headers
        Parameters:
            topic_name (str):      name of topic to subscribe to
            topic_type (msg_type): type of message received on topic
            wait_for_data (bool):  block constructor until a message has been received
        """

        self.data = None
        self.lock = Lock()

        self.subscriber = rospy.Subscriber(topic_name, topic_type, self.callback)
        self.get(wait_for_data)

    def callback(self, msg):
        with self.lock:
            self.data = msg

    def get(self, block_until_data=True):
        """
        Returns the latest msg from the subscribed topic
        Parameters:
            block_until_data (bool): block if no message has been received yet.
                                     Guarantees a msg is returned (not None)
        """
        wait_for(lambda: not (block_until_data and self.data is None))

        with self.lock:
            return deepcopy(self.data)


def wait_for(func):
    """
    Waits for function evaluation to be true. Exits cleanly from ros.
    Introduces sleep delay, not recommended for time critical operations
    """

    while not func() and not rospy.is_shutdown():
        time.sleep(0.01)

# initialize tracker
kinect_intrinsics = np.array(
    [1068.842896477257, 0.0, 950.2974736758024, 0.0, 1066.0150152835104, 537.097974092338, 0.0, 0.0, 1.0],
    dtype=np.float32).reshape((3, 3))
kinect_intrinsics[:2] /= 2.0

template_verts, template_edges = build_line(1.0, 50)
key_func = chroma_key_rope

prior = UniformPrior()
optimizer = PriorConstrainedOptimizer(template=template_verts, edges=template_edges)
listener_left = Listener(topic_name="/left_gripper/prior", topic_type=TransformStamped)
listener_right = Listener(topic_name="/right_gripper/prior", topic_type=TransformStamped)
cpd_params = CPDParams()
cdcpd_params = CDCPDParams(prior=prior, optimizer=optimizer)
cdcpd = ConstrainedDeformableCPD(template=template_verts,
                                 cdcpd_params=cdcpd_params)

# initialize ROS publisher
pub = rospy.Publisher("/cdcpd_tracker/points", PointCloud2, queue_size=10)


def callback(msg: PointCloud2):
    # converting ROS message to dense numpy array
    data = ros_numpy.numpify(msg)
    left_data = listener_left.get()
    right_data = listener_right.get()
    arr = ros_numpy.point_cloud2.split_rgb_field(data)
    point_cloud_img = structured_to_unstructured(arr[['x', 'y', 'z']])
    color_img = structured_to_unstructured(arr[['r', 'g', 'b']])
    mask_img = key_func(point_cloud_img, color_img)
    left_gripper = [left_data.transform.translation.x,left_data.transform.translation.y,left_data.transform.translation.z]
    right_gripper = [right_data.transform.translation.x,right_data.transform.translation.y,right_data.transform.translation.z]    
    prior_pos = np.array([left_gripper, right_gripper])
    prior_idx = [0, 49]
    optimizer.set_prior(prior_pos=prior_pos, prior_idx=prior_idx)
    # invoke tracker
    tracking_result = cdcpd.step(point_cloud=point_cloud_img,
                                 mask=mask_img,
                                 cpd_param=cpd_params)

    # converting tracking result to ROS message
    if tracking_result.dtype is not np.float32:
        tracking_result = tracking_result.astype(np.float32)
    out_struct_arr = unstructured_to_structured(tracking_result, names=['x', 'y', 'z'])
    pub_msg = ros_numpy.msgify(PointCloud2, out_struct_arr)
    pub_msg.header = msg.header
    pub.publish(pub_msg)


def main():
    rospy.init_node('gripper_cdcpd_node')
    rospy.Subscriber("/kinect2_victor_head/qhd/points", PointCloud2, callback, queue_size=2)

    #rospy.Subscriber("/kinect2/qhd/points", PointCloud2, callback, queue_size=2) #for rope_fast.bag
    rospy.spin()


main()

