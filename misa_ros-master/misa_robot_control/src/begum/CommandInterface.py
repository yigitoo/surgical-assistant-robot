
#!/usr/bin/env python
import rospy
import time

import numpy as np
from copy import deepcopy

from std_msgs.msg import Bool, Int32

from misa_files.msg import KinovaCommandMessage, KinovaErrorMessage, KinovaInfoMessage

import InfoLogger, ErrorLogger, Endowrist
from KinovaErrors import KinovaErrors

from numpy.linalg import inv
from numpy.linalg import det
import numpy.matlib

#Joint Dimensions
Lse = 0.18022
Lpe = 0.061 + 0.005/2 + 0.119221

L0x = 0.061
L1x = Lpe
L1z = Lpe
L2y = Lpe
L2z = Lpe
L3x = Lse
L3y = Lse
L4x = 0.120063
L5z = 0.485
L6z = 0.0096
L7z = 0.01183

class CommandInterface:

    def __init__(self, interface_name = "Command Interface", publisher_name = "kinova_command"):
        self._interface_name = interface_name
        self._publisher_name = publisher_name

        self._emergency_mode = False

        self._endowrist = Endowrist.Endowrist()

        self._info_logger = InfoLogger.InfoLogger(owner_name = self._interface_name)
        self._error_logger = ErrorLogger.ErrorLogger(owner_name = self._interface_name)

        self._emergency = rospy.Subscriber("kinova_emergency_situation", Bool, self.emergency_check)

        rospy.loginfo("CommandInterface: " + str(interface_name) + " is created.")
        rospy.loginfo("CommandInterface: Setup publisher.")
        self._command_pub = rospy.Publisher(self._publisher_name, KinovaCommandMessage, queue_size = 100)
        self._safety_pub = rospy.Publisher("kinova_errors", KinovaErrorMessage, queue_size = 10)
        rospy.on_shutdown(self.safe_exit)
        rospy.loginfo("CommandInterface: Setup successful!")

        self._info_logger.report_info(source = self._interface_name, reason = "setup")

    def send_angular_torque_command(self, tArr):
        #tArr is a numpy array.rad refers command in radians.
        if self._emergency_mode:
            self._error_logger.report_error(source = self._interface_name, error = "CommandTrialInEmergencyMode", priority = KinovaErrors.CommandTrialInEmergencyMode.value)
            return

        msg = KinovaCommandMessage()

        msg.name = "angular"
        msg.type = "torque"

        msg.header.stamp = rospy.Time.now()
        msg.data.data = tArr
        self._command_pub.publish(msg)

    def send_angular_position_command(self, qArr, type = "rad"):
        #qArr is a numpy array., rad refers command in radians.
        if self._emergency_mode:
            self._error_logger.report_error(source = self._interface_name, error = "CommandTrialInEmergencyMode", priority = KinovaErrors.CommandTrialInEmergencyMode.value)
            return

        msg = KinovaCommandMessage()

        msg.name = "angular"
        msg.type = "position"

        msg.header.stamp = rospy.Time.now()
        if type == "rad":
            msg.data.data = deepcopy(np.rad2deg(qArr[0:4]))
        else:
            msg.data.data = deepcopy(qArr[0:4])

        self._command_pub.publish(msg)
        if qArr.shape[0] > 4:
            self._endowrist._angle_command(deepcopy(qArr[4:7]))

    def send_angular_velocity_command(self, vArr, type = "rad"):
        #vArr is a numpy array.rad refers command in radians.
        if self._emergency_mode:
            self._error_logger.report_error(source = self._interface_name, error = "CommandTrialInEmergencyMode", priority = KinovaErrors.CommandTrialInEmergencyMode.value)
            return

        msg = KinovaCommandMessage()

        msg.name = "angular"
        msg.type = "velocity"

        msg.header.stamp = rospy.Time.now()

        if type == "rad":
            msg.data.data = deepcopy(np.rad2deg(vArr))
        else:
            msg.data.data = deepcopy(vArr)

        self._command_pub.publish(msg)


    def emergency_check(self,msg):
        #Emergency button procedure.
        if msg.data:
            self.send_angular_velocity_command(vArr = np.array([0,0,0,0]))
            self._info_logger.report_info(source = self._interface_name, reason = "emergency_on", is_functional = True, emergency_mode = True)
        else:
            self._info_logger.report_info(source = self._interface_name, reason = "emergency_off", is_functional = True, emergency_mode = False)
        self._emergency_mode = msg.data

    def safe_exit(self):
        self.send_angular_velocity_command(vArr = np.array([0,0,0,0]))
        self._info_logger.report_info(source = self._interface_name, reason = "termination", is_functional = False)
        self._error_logger.safe_exit()
        self._info_logger.safe_exit()

    def get_description(self):
        #Not used yet.
        pass
