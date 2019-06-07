import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
import ros_numpy
from cdcpd.cdcpd import CDCPDParams, ConstrainedDeformableCPD
from cdcpd.cpd import CPDParams
from cdcpd.optimizer import PriorConstrainedOptimizer
from cdcpd.geometry_utils import build_line
from cdcpd.geometry_utils import build_rectangle
from cdcpd.cv_utils import chroma_key_rope
from cdcpd.cv_utils import chroma_key_mflag_lab
from cdcpd.prior import UniformPrior
from cdcpd.failure_recovery import SmoothFreeSpaceCost
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

class Tracker:
    def __init__(self, object_name, use_gripper_prior=True, use_failure_recovery=True):
        """
        Tracker is a node which combines CDCPD tracking with the information from 
        grippers and also uses failure recovery for rope or cloth.
        Parameters:
            object_name(str):       "rope" or "cloth" depending on the object you need to detect
            use_gripper_prior:      Whether use the information from the grippers for optimization
            use_failure_recovery:   Whether use the code for failure recovery or not
        """
        self.use_gripper_prior = use_gripper_prior
        self.kinect_intrinsics = np.array([1068.842896477257, 0.0, 950.2974736758024, 0.0, 1066.0150152835104, 537.097974092338, 0.0, 0.0, 1.0],
            dtype=np.float32).reshape((3, 3))
        self.kinect_intrinsics[:2] /= 2.0
        self.listener_left = None
        self.listener_right = None
        self.cost_estimator = None
        if(object_name=="rope"):
            self.template_verts, self.template_edges = build_line(1.0, 50)
            self.key_func = chroma_key_rope
        elif(object_name=="cloth"):
            self.template_verts, self.template_edges = build_rectangle(width=0.45, height=0.32, grid_size=0.03)
            self.key_func = chroma_key_mflag_lab

        if(use_gripper_prior==True):
            self.prior = UniformPrior()
            self.optimizer = PriorConstrainedOptimizer(template=self.template_verts, edges=self.template_edges)
            self.listener_left = Listener(topic_name="/left_gripper/prior", topic_type=TransformStamped)
            self.listener_right = Listener(topic_name="/right_gripper/prior", topic_type=TransformStamped)
        else:
            self.prior = ThresholdVisibilityPrior(self.kinect_intrinsics)
            self.optimizer = DistanceConstrainedOptimizer(template=self.template_verts, edges=self.template_edges)
        if(use_failure_recovery==True):
            self.cost_estimator = SmoothFreeSpaceCost(self.kinect_intrinsics)

        self.cpd_params = CPDParams()
        self.cdcpd_params = CDCPDParams(prior=self.prior, optimizer=self.optimizer)
        self.cdcpd = ConstrainedDeformableCPD(template=self.template_verts,
                                         cdcpd_params=self.cdcpd_params)
        # initialize ROS publisher
        self.pub = rospy.Publisher("/cdcpd_tracker/points", PointCloud2, queue_size=10)
        self.listen()
    
    def listen(self):
        self.sub = rospy.Subscriber("/kinect2_victor_head/qhd/points", PointCloud2, self.callback, queue_size=2)
    
    def callback(self, msg: PointCloud2):
        # converting ROS message to dense numpy array
        data = ros_numpy.numpify(msg)
        if(self.use_gripper_prior==True):
            left_data = self.listener_left.get()
            right_data = self.listener_right.get()
        arr = ros_numpy.point_cloud2.split_rgb_field(data)
        point_cloud_img = structured_to_unstructured(arr[['x', 'y', 'z']])
        color_img = structured_to_unstructured(arr[['r', 'g', 'b']])
        mask_img = self.key_func(point_cloud_img, color_img)
        if(self.use_gripper_prior==True):
            left_gripper = [left_data.transform.translation.x,left_data.transform.translation.y,left_data.transform.translation.z]
            right_gripper = [right_data.transform.translation.x,right_data.transform.translation.y,right_data.transform.translation.z]    
            prior_pos = np.array([left_gripper, right_gripper])
            prior_idx = [0, 49]
            self.optimizer.set_prior(prior_pos=prior_pos, prior_idx=prior_idx)

        # invoke tracker
        tracking_result = self.cdcpd.step(point_cloud=point_cloud_img,
                                     mask=mask_img,
                                     cpd_param=self.cpd_params)

        # converting tracking result to ROS message
        if tracking_result.dtype is not np.float32:
            tracking_result = tracking_result.astype(np.float32)
        out_struct_arr = unstructured_to_structured(tracking_result, names=['x', 'y', 'z'])
        pub_msg = ros_numpy.msgify(PointCloud2, out_struct_arr)
        pub_msg.header = msg.header
        self.pub.publish(pub_msg)

def main():
    rospy.init_node('cdcpd_node')
    tracker = Tracker(object_name = "rope", use_failure_recovery=True)
    rospy.spin()

main()
