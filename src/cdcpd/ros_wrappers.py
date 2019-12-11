import rospy
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
        temp = deepcopy(self.data)
        self.data = None

        with self.lock:
            return temp


def wait_for(func):
    """
    Waits for function evaluation to be true. Exits cleanly from ros.
    Introduces sleep delay, not recommended for time critical operations
    """

    while not func() and not rospy.is_shutdown():
        time.sleep(0.01)

def get_ros_param(param_name, default):
    try:
        param = rospy.get_param(param_name)
        rospy.loginfo("Using " + param_name + " from server: " + str(param))
        return param

    except KeyError:
        rospy.logwarn("Default value of " + param_name + " used: " + str(default))
        return default

