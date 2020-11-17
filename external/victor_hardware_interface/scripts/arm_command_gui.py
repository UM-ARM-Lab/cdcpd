#!/usr/bin/env python

# [Create a window]

# Python imports
import sys
import signal
import time
import math
import copy
from functools import partial

# Qt imports
from PyQt5.QtCore import Qt
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

# ROS imports
import rospkg
import rospy
from victor_hardware_interface.msg import Robotiq3FingerCommand, MotionCommand, MotionStatus, Robotiq3FingerStatus, \
    ControlModeParameters, ControlMode
from victor_hardware_interface.srv import SetControlMode, GetControlMode
from victor_hardware_interface import victor_utils

finger_range_discretization = 1000
arm_joint_limit_margin = 1

joint_limits = {'joint_1': (-170, 170),
                'joint_2': (-120, 120),
                'joint_3': (-170, 170),
                'joint_4': (-120, 120),
                'joint_5': (-170, 170),
                'joint_6': (-120, 120),
                'joint_7': (-170, 170)}

joint_limits_with_margin = {joint_name: (lower + arm_joint_limit_margin,
                                         upper - arm_joint_limit_margin)
                            for (joint_name, (lower, upper)) in joint_limits.items()}

joint_names = ['joint_' + str(i) for i in range(1, 8)]

finger_names = ['finger_a', 'finger_b', 'finger_c']
finger_joint_names = ['finger_a', 'finger_b', 'finger_c', 'scissor']
finger_column = {'finger_a': 2, 'finger_b': 4, 'finger_c': 6, 'scissor': 9}
finger_joint_labels = {'finger_a': 'Finger A', 'finger_b': 'Finger B', 'finger_c': 'Finger C', 'scissor': 'Scissor'}

finger_command_names = ['position', 'speed', 'force']
finger_command_default = {'position': 0.0, 'speed': 1.0, 'force': 1.0}


def clip(value, joint_name):
    """Clips 'value' between the joint limits of string 'joint_name'"""
    min_lim, max_lim = joint_limits_with_margin[joint_name]
    return min(max(value, min_lim), max_lim)


class Widget(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)

        self.v_layout = QGridLayout()

        self.left_arm = Arm(self.v_layout, 'left_arm', 0)
        self.right_arm = Arm(self.v_layout, 'right_arm', 1)

        self.setLayout(self.v_layout)


class Arm:
    def __init__(self, parent_layout, arm_name, box_row):
        self.name = arm_name
        self.block_command = False

        self.v_layout = QGridLayout()
        self.col_num = 11
        self.row_count = [0] * self.col_num

        self.reset_arm_to_zero_button = self.init_button('Home the Arm', 0, self.reset_arm_command_to_zero)
        self.reset_arm_slider_to_current_value_button = self.init_button(
            'Reset the Sliders', 1, self.reset_arm_slider_to_current_value)
        self.open_gripper_button = self.init_button('Open the Gripper', 2, self.open_gripper_command)
        self.close_gripper_button = self.init_button('Close the Gripper', 4, self.close_gripper_command)
        self.reset_speed_force_button = self.init_button('Reset Speed and Force', 6, self.reset_speed_force_command)
        self.reset_gripper_position_slider_to_current_value_button = self.init_button(
            'Reset the Sliders', 8, self.reset_gripper_position_slider_to_current_value)

        # Create all joint sliders, textboxes, and labels
        self.joint_labels = {}
        self.joint_sliders = {}
        self.joint_textboxes = {}

        for joint_name in joint_names:
            slider_callback = partial(self.joint_slider_moved, joint_name=joint_name)
            text_callback = partial(self.joint_textbox_modified, joint_name=joint_name)

            label = 'Joint ' + joint_name[-1] + ' Command'

            self.joint_labels[joint_name] = self.init_label(label, 0)
            self.joint_sliders[joint_name] = self.init_slider(joint_limits_with_margin[joint_name],
                                                              0, slider_callback)
            self.joint_textboxes[joint_name] = self.init_textbox('0', 1, text_callback, 1)

        # Create all finger joint sliders, textboxes, and labels
        self.finger_labels = {}
        self.finger_sliders = {}
        self.finger_textboxes = {}

        # increment blank cell in the header row
        for col in [3, 5, 7, 9, 10]:
            self.row_count[col] += 1

        for finger_joint_name in finger_joint_names:
            col = finger_column[finger_joint_name]

            for j, finger_command_name in enumerate(finger_command_names):
                slider_callback = partial(self.finger_joint_slider_moved,
                                          finger_joint_name=finger_joint_name,
                                          finger_command_name=finger_command_name)
                text_callback = partial(self.finger_joint_textbox_modified,
                                        finger_joint_name=finger_joint_name,
                                        finger_command_name=finger_command_name)

                label = finger_joint_labels[finger_joint_name] + ' Command ' + \
                        finger_command_name[0].upper() + finger_command_name[1:]

                self.finger_labels[(finger_joint_name, finger_command_name)] = self.init_label(label, col)

                self.finger_sliders[(finger_joint_name, finger_command_name)] = \
                    self.init_slider((0, finger_range_discretization), col, slider_callback,
                                     finger_command_default[finger_command_name] * finger_range_discretization)

                self.finger_textboxes[(finger_joint_name, finger_command_name)] = \
                    self.init_textbox('%5.3f' % finger_command_default[finger_command_name], col + 1, text_callback, 1)

        # Create finger synchronization checkboxes
        self.finger_synchronization_checkboxes = {}
        self.synchonize_label = self.init_label('Synchronize', 8)

        for j, finger_command_name in enumerate(finger_command_names):
            if j == 0:
                checkbox_row_skip = 0
            else:
                checkbox_row_skip = 1

            cb_callback = partial(self.finger_synchronization_checkbox_changed, finger_command_name=finger_command_name)
            self.finger_synchronization_checkboxes[finger_command_name] = \
                self.init_checkbox('', 8, cb_callback, checkbox_row_skip)

        self.control_mode_label = self.init_label('Control Mode', 2, 1)
        control_mode_list = ['JOINT_POSITION', 'CARTESIAN_POSE', 'JOINT_IMPEDANCE', 'CARTESIAN_IMPEDANCE']
        stiffness_list = ['STIFF', 'MEDIUM', 'SOFT']
        s = victor_utils.Stiffness
        self.stiffness_map = [s.STIFF, s.MEDIUM, s.SOFT]
        self.stiffness = self.stiffness_map[0]
        self.control_mode_combobox = self.init_combobox(control_mode_list, 2, self.change_control_mode, 0)
        self.stiffness_combobox = self.init_combobox(stiffness_list, 2, self.set_stiffness, 0)

        self.finger_command = Robotiq3FingerCommand()
        self.arm_command = MotionCommand()
        self.arm_status = MotionStatus()
        self.gripper_status = Robotiq3FingerStatus()

        self.arm_status_updated = False
        self.gripper_status_updated = False

        self.finger_command.finger_a_command.speed = 1.0
        self.finger_command.finger_b_command.speed = 1.0
        self.finger_command.finger_c_command.speed = 1.0
        self.finger_command.scissor_command.speed = 1.0

        self.finger_command_publisher = \
            rospy.Publisher(self.name + '/gripper_command', Robotiq3FingerCommand, queue_size=10)
        self.arm_command_publisher = rospy.Publisher(self.name + '/motion_command', MotionCommand, queue_size=10)

        self.gripper_status_subscriber = \
            rospy.Subscriber(self.name + '/gripper_status', Robotiq3FingerStatus, self.gripper_status_callback)
        self.arm_status_subscriber = \
            rospy.Subscriber(self.name + '/motion_status', MotionStatus, self.arm_status_callback)

        self.fingers_same_command = {finger_command_name: False for finger_command_name in finger_command_names}

        self.groupbox = QGroupBox(self.name)
        self.groupbox.setLayout(self.v_layout)
        self.groupbox.setFlat(False)
        parent_layout.addWidget(self.groupbox, box_row, 0)

        print('Getting', self.name, 'current joint values ...')
        sys.stdout.flush()
        while (not self.arm_status_updated) or (not self.gripper_status_updated):
            time.sleep(0.01)
        print('Done')

        self.reset_arm_slider_to_current_value()
        self.reset_gripper_position_slider_to_current_value()

        print('Getting {} current control mode ...'.format(self.name))
        sys.stdout.flush()
        get_current_control_mode = rospy.ServiceProxy('/' + self.name + '/get_control_mode_service', GetControlMode)
        control_mode = get_current_control_mode()
        self.active_control_mode_int = control_mode.active_control_mode.control_mode.mode
        self.control_mode_combobox.setCurrentIndex(self.active_control_mode_int)
        print('Done')

    def arm_status_callback(self, data):
        self.arm_status = data
        self.arm_status_updated = True

    def gripper_status_callback(self, data):
        self.gripper_status = data
        self.gripper_status_updated = True

    def skip_rows(self, col, num_rows):
        self.row_count[col] = self.row_count[col] + num_rows

    def init_button(self, text, col, slot_function, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        button = QPushButton(text)
        self.v_layout.addWidget(button, self.row_count[col], col)
        button.clicked.connect(slot_function)
        self.row_count[col] += 1

        return button

    def init_slider(self, limits, col, slot_function, init_position=None, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        slider = QSlider()
        slider.setOrientation(Qt.Horizontal)
        slider.setRange(limits[0], limits[1])
        if init_position is not None:
            slider.setValue(init_position)
        slider.valueChanged.connect(slot_function)

        self.v_layout.addWidget(slider, self.row_count[col], col)
        self.row_count[col] += 1

        return slider

    def init_label(self, text, col, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        label = QLabel(text)
        self.v_layout.addWidget(label, self.row_count[col], col)
        self.row_count[col] += 1

        return label

    def init_textbox(self, text, col, slot_function, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        textbox = QLineEdit(text)
        self.v_layout.addWidget(textbox, self.row_count[col], col)
        textbox.editingFinished.connect(slot_function)
        self.row_count[col] += 1

        return textbox

    def init_checkbox(self, text, col, slot_function, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        checkbox = QCheckBox(text)
        self.v_layout.addWidget(checkbox, self.row_count[col], col)
        checkbox.stateChanged.connect(slot_function)
        self.row_count[col] += 1

        return checkbox

    def init_combobox(self, option_texts, col, slot_function, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        combobox = QComboBox()
        for text in option_texts:
            combobox.addItem(text)
        self.v_layout.addWidget(combobox, self.row_count[col], col)
        combobox.currentIndexChanged.connect(slot_function)
        self.row_count[col] += 1

        return combobox

    def finger_synchronization_checkbox_changed(self, finger_command_name):
        if self.finger_synchronization_checkboxes[finger_command_name].isChecked():
            self.fingers_same_command[finger_command_name] = True
        else:
            self.fingers_same_command[finger_command_name] = False

    def open_gripper_command(self):
        for finger_name in finger_names:
            self.finger_sliders[(finger_name, 'position')].setValue(0)

    def close_gripper_command(self):
        for finger_name in finger_names:
            self.finger_sliders[(finger_name, 'position')].setValue(finger_range_discretization)

    def reset_arm_command_to_zero(self):
        for joint in joint_names:
            self.joint_sliders[joint].setValue(0)

    def reset_arm_slider_to_current_value(self):
        local_arm_status = copy.deepcopy(self.arm_status)

        self.block_command = True

        for joint in joint_names:
            pos_rad = getattr(local_arm_status.measured_joint_position, joint)
            pos_deg = round(clip(math.degrees(pos_rad), joint))
            self.joint_sliders[joint].setValue(pos_deg)

        self.block_command = False

        for joint in joint_names:
            pos = getattr(local_arm_status.measured_joint_position, joint)
            vel = getattr(local_arm_status.measured_joint_velocity, joint)
            setattr(self.arm_command.joint_position, joint, pos)
            setattr(self.arm_command.joint_velocity, joint, vel)

    def disable_arm_sliders(self):
        for joint in joint_names:
            self.joint_sliders[joint].setEnabled(False)
            self.joint_textboxes[joint].setReadOnly(True)

    def enable_arm_sliders(self):
        for joint in joint_names:
            self.joint_sliders[joint].setEnabled(True)
            self.joint_textboxes[joint].setReadOnly(False)

    def reset_speed_force_command(self):
        for finger_joint_name in finger_joint_names:
            for finger_command_name in ['speed', 'force']:
                self.finger_sliders[(finger_joint_name, finger_command_name)].setValue(finger_range_discretization)

    def reset_gripper_position_slider_to_current_value(self):
        local_gripper_status = copy.deepcopy(self.gripper_status)

        self.block_command = True

        for finger_joint_name in finger_joint_names:
            pos = getattr(getattr(local_gripper_status, finger_joint_name + '_status'), 'position')
            self.finger_sliders[(finger_joint_name, 'position')].setValue(int(round(pos * finger_range_discretization)))

        self.block_command = False

        for finger_joint_name in finger_joint_names:
            finger_status = getattr(local_gripper_status, finger_joint_name + '_status')
            finger_command = getattr(self.finger_command, finger_joint_name + '_command')
            finger_command.position = finger_status.position

    def move_all_fingers(self, position, finger_command_name):
        value = float(position) / finger_range_discretization
        value = min(max(value, 0), 1)
        position = int(value * finger_range_discretization)

        for finger_name in finger_names:
            self.finger_textboxes[(finger_name, finger_command_name)].setText('%5.3f' % value)
            finger_command = getattr(self.finger_command, finger_name + '_command')
            setattr(finger_command, finger_command_name, value)
            self.finger_sliders[(finger_name, finger_command_name)].setValue(position)

        if not self.block_command:
            self.finger_command_publisher.publish(self.finger_command)

    def finger_joint_textbox_modified(self, finger_joint_name, finger_command_name):
        textbox = self.finger_textboxes[(finger_joint_name, finger_command_name)]
        slider_position = int(float(textbox.displayText()) * finger_range_discretization)
        self.finger_sliders[(finger_joint_name, finger_command_name)].setValue(slider_position)

    def finger_joint_slider_moved(self, position, finger_joint_name, finger_command_name):
        if finger_joint_name != 'scissor' and self.fingers_same_command[finger_command_name]:
            self.move_all_fingers(position, finger_command_name)
        else:
            value = float(position) / finger_range_discretization
            value = min(max(value, 0), 1)
            self.finger_textboxes[(finger_joint_name, finger_command_name)].setText('%5.3f' % value)
            finger_command = getattr(self.finger_command, finger_joint_name + '_command')
            setattr(finger_command, finger_command_name, value)
            if not self.block_command:
                self.finger_command_publisher.publish(self.finger_command)

    def joint_textbox_modified(self, joint_name):
        value = int(self.joint_textboxes[joint_name].displayText())
        value = clip(value, joint_name)
        self.joint_slider_moved(value, joint_name)

    def joint_slider_moved(self, position, joint_name):
        self.joint_textboxes[joint_name].setText(str(position))
        self.joint_sliders[joint_name].setValue(position)
        setattr(self.arm_command.joint_position, joint_name, math.radians(position))
        if not self.block_command:
            self.arm_command_publisher.publish(self.arm_command)

    def set_stiffness(self, stiffness):
        self.stiffness = self.stiffness_map[stiffness]
        print("Stiffness: ".format(self.stiffness))
        self.change_control_mode(self.active_control_mode_int)

    def change_control_mode(self, control_mode):
        if control_mode == ControlMode.JOINT_POSITION:
            self.enable_arm_sliders()
            print('Switching to JOINT_POSITION mode.')
        elif control_mode == ControlMode.CARTESIAN_POSE:
            self.disable_arm_sliders()
            print('Switching to CARTESIAN_POSE mode.')
        elif control_mode == ControlMode.JOINT_IMPEDANCE:
            self.enable_arm_sliders()
            print('Switching to JOINT_IMPEDANCE mode')
        elif control_mode == ControlMode.CARTESIAN_IMPEDANCE:
            print('CARTESIAN_IMPEDANCE mode switch is not implemented yet.')
            return

        result = victor_utils.set_control_mode(control_mode, self.name, self.stiffness)

        if result.success:
            print('Control mode switch success.')
        else:
            print('Control mode switch failure.')
            print(result.message)
        self.active_control_mode_int = control_mode
        self.arm_command.control_mode.mode = control_mode


def main():
    rospy.init_node('arm_command_widget')
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)

    widget = Widget()
    widget.setWindowTitle("Arm Command Widget")
    widget.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
