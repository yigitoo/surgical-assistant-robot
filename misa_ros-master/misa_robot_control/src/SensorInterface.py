
#!/usr/bin/env python
import rospy
import time

import numpy as np
from copy import deepcopy

from misa_files.msg import KinovaSensorMessage, KinovaErrorMessage, KinovaJointStateMessage
from misa_files.srv import KinovaSensorService
from std_msgs.msg import Bool

import InfoLogger, ErrorLogger
from KinovaErrors import KinovaErrors

class SensorInterface:
    def __init__(self, interface_name = "Sensor Interface", subscriber_name = "kinova_joint_states", GUI = False):
        self._interface_name = interface_name
        self._subscriber_name = subscriber_name
        self._GUI = GUI
        self._emergency_mode = False

        self._joint_state_msg = KinovaJointStateMessage()

        rospy.loginfo("SensorInterface: " + str(interface_name) + " is created.")
        self._safety_pub = rospy.Publisher("km_error", KinovaErrorMessage, queue_size = 10)
        self._emergency = rospy.Subscriber("kinova_emergency_situation", Bool, self.emergency_check)

        self._info_logger = InfoLogger.InfoLogger(owner_name = self._interface_name)
        self._error_logger = ErrorLogger.ErrorLogger(owner_name = self._interface_name)

        self._subscriber = rospy.Subscriber(self._subscriber_name, KinovaJointStateMessage,self.kinova_sensor_callback)

        rospy.loginfo("Sensor Interface is set up.")

        rospy.on_shutdown(self.safe_exit)
        self._info_logger.report_info(source = self._interface_name, reason = "setup")

    def gui_sensor_callback(self, qArr, vArr, aArr, tArr):
        pass

    def set_gui_callback(self, callback):
        self.gui_sensor_callback = callback

    def kinova_sensor_callback(self, kjs_message):
        #kjs refers kinova_joint_states
        #This callback saves the last message
        self._joint_state_msg.header = kjs_message.header
        self._joint_state_msg.positions = deepcopy(kjs_message.positions)
        self._joint_state_msg.velocities = deepcopy(kjs_message.velocities)
        self._joint_state_msg.accelerations = deepcopy(kjs_message.accelerations)
        self._joint_state_msg.torques = deepcopy(kjs_message.torques)

        if self._GUI:
            qArr, vArr, aArr, tArr = self.get_joint_states(type = "deg")
            self.gui_sensor_callback(qArr, vArr, aArr, tArr)

    def get_joint_states(self, type = "rad"):
        if type == "rad":
            return np.deg2rad(np.array(self._joint_state_msg.positions)),np.deg2rad(np.array(self._joint_state_msg.velocities)),np.deg2rad(np.array(self._joint_state_msg.accelerations)),np.deg2rad(np.array(self._joint_state_msg.torques))

        return np.array(self._joint_state_msg.positions),np.array(self._joint_state_msg.velocities),np.array(self._joint_state_msg.accelerations),np.array(self._joint_state_msg.torques)

    def get_sensor_values(self, sensor_name):
        if sensor_name == "torques":
            return np.array(self._joint_state_msg.torques)
        elif sensor_name == "positions":
            return np.array(self._joint_state_msg.positions)
        elif sensor_name == "velocities":
            return np.array(self._joint_state_msg.velocities)
        elif sensor_name == "accelerations":
            return np.array(self._joint_state_msg.accelerations)

    def emergency_check(self,msg):
        #Emergency button procedure.
        self._emergency_mode = msg.data
        if self._emergency_mode:
            self._info_logger.report_info(source = self._interface_name, reason = "emergency_on", is_functional = True, emergency_mode = True)
        else:
            self._info_logger.report_info(source = self._interface_name, reason = "emergency_off", is_functional = True, emergency_mode = False)

    def safe_exit(self):
        self._info_logger.report_info(source = self._interface_name, reason = "termination", is_functional = False)
