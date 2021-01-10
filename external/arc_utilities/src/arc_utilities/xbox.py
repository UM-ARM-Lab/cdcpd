import rospy
from arc_utilities.listener import Listener
from arc_utilities.ros_helpers import joy_to_xbox, wait_for
from sensor_msgs.msg import Joy


class Xbox():
    def __init__(self, joystick_topic="joy", xpad=True):
        """
        Parameters:
        joystick_topic (string): topic name to subscribe to
        xpad (bool): True if using the default xpad driver, False if using xboxdrv

        """
        self.xpad = xpad
        self.xbox_listener = Listener(joystick_topic, Joy)

    def get_buttons_state(self):
        """
        Returns an xbox struct of the last joystick message received
        """
        return joy_to_xbox(self.xbox_listener.get(), self.xpad)

    def get_button(self, button):
        """
        Return value of button or axis of the controller
        0 or 1 for buttons
        -1.0 to 1.0 (at most) for axes
        """
        return getattr(self.get_buttons_state(), button)

    def wait_for_button(self, button, message=True):
        """
        Waits for button press on xbox.

        Parameters:
        button (str):   Name of xbox button. "A", "B", "X", ...
        message (bool): log a message informing the user?
        """
        if message:
            rospy.loginfo("Waiting for xbox button: " + button)

        wait_for(lambda: not self.get_button(button) == 0)