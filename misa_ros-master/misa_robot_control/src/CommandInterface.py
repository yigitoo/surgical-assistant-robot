
#!/usr/bin/env python
import rospy
import time

import numpy as np
from copy import deepcopy

from std_msgs.msg import Bool, Int32

from misa_files.msg import KinovaCommandMessage, KinovaErrorMessage, KinovaInfoMessage

import InfoLogger,ErrorLogger
from KinovaErrors import KinovaErrors

class CommandInterface:
    def __init__(self, interface_name = "Command Interface", publisher_name = "kinova_command"):
        self._interface_name = interface_name
        self._publisher_name = publisher_name

        self._emergency_mode = False

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
            msg.data.data = deepcopy(np.rad2deg(qArr))
        else:
            msg.data.data = deepcopy(qArr)

        self._command_pub.publish(msg)

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
