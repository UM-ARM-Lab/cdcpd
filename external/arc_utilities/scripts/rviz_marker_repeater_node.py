import signal
import sys

from PyQt5.QtWidgets import QApplication

import rospy
from arc_utilities.rviz_marker_repeater import RVizRepeater

if __name__ == '__main__':
    rospy.init_node('rviz_marker_repeater')
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    repeater = RVizRepeater()
    sys.exit(app.exec_())
