#!/usr/bin/env python

"""
Version : 1.0.0
Description : This class checks other interfaces' functionality and emergency status.

Initial Goal : It takes kinova error messages
Final Goal : Modular class representation.
                -Easily Appliable to other projects.
"""

import rospy
from std_msgs.msg import Bool

import numpy as np
from copy import deepcopy
from misa_files.msg import KinovaErrorMessage, KinovaInfoMessage
from threading import Thread

class StatusInterface:
    def __init__(self, sub_interface_names, interface_name = "Status Interface" ,GUI = False):
        self._interface_name = interface_name
        self.GUI = GUI
        self._error_subscriber = rospy.Subscriber("kinova_errors", KinovaErrorMessage, self._error_msg_handler)
        self._info_subscriber = rospy.Subscriber("kinova_system_info", KinovaInfoMessage, self._info_msg_handler)
        self._emergency_subsriber = rospy.Subscriber("kinova_emergency_situation", Bool, self._emergency_handler)
        self._emergency_pub = rospy.Publisher("kinova_emergency_situation", Bool, queue_size = 10)

        self.error_threeshold = 5
        self._emergency_mode = False
        self._is_system_functional = False

        self._sub_interface_names = deepcopy(sub_interface_names)
        self._sub_interface_names.append(self._interface_name)
        self._interface_functionalities = np.zeros(len(sub_interface_names), dtype = np.bool)
        self._interface_emergencies = np.ones(len(sub_interface_names), dtype = np.bool)
        self._interface_statuses = dict(zip(self._sub_interface_names, np.resize(np.array([self._interface_functionalities, self._interface_emergencies]),
                                                                                                                    (len(self._sub_interface_names),2))))

        Thread(target = rospy.spin).start()

    def gui_error_callback(self, interface_statuses, error_msg):
        pass

    def gui_info_callback(self, interface_statuses, info_msg):
        pass

    def set_gui_error_callback(self, callback):
        self.gui_error_callback = callback

    def set_gui_info_callback(self, callback):
        self.gui_info_callback = callback

    #Main Callbacks

    def get_interface_statuses(self):
        return self._interface_statuses

    def _error_msg_handler(self, error_msg):
        proc_interface_status = self._interface_statuses.get(error_msg.source)

        if error_msg.priority > self.error_threeshold:
            #Emergency situation.
            proc_interface_status[1] = True
            #Spread the word.
            self._emergency_pub.publish(Bool(True))

        self._interface_statuses[error_msg.source] = deepcopy(proc_interface_status)

        if self.GUI:
            self.gui_error_callback(self._interface_statuses,error_msg)

        self._is_system_functional = self._check_system_functionality()

    def _info_msg_handler(self, info_msg):

        proc_interface_status = self._interface_statuses.get(info_msg.source)
        proc_interface_status = [info_msg.functional, info_msg.emergency]
        self._interface_statuses[info_msg.source] = deepcopy(proc_interface_status)

        if self.GUI:
            self.gui_info_callback(self._interface_statuses, info_msg)

        self._is_system_functional = self._check_system_functionality()

    def _check_system_functionality(self):
        for interface in self._interface_statuses:
            proc_interface_status = self._interface_statuses.get(interface)
            if not proc_interface_status[0]:
                return False
        return True

    def get_unfunctional_source(self):
        temp_list = []
        for interface in self._interface_statuses:
            proc_interface_status = self._interface_statuses.get(interface)
            if not proc_interface_status[0]:
                temp_list.append(interface)
        return temp_list

    def switch_emergency_situation(self, is_emergency_mode):
        if not self._emergency_mode == is_emergency_mode:
            self._emergency_mode = is_emergency_mode
            self._emergency_pub.publish(Bool(is_emergency_mode))
            self._update_self_info()

    def _emergency_handler(self, msg):
        if not self._emergency_mode == msg.data:
            self._error_msg_handler
