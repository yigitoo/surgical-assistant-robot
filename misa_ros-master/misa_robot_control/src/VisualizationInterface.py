
#!/usr/bin/env python
import rospy
import time

import numpy as np
from copy import deepcopy

import std_msgs
from misa_files.srv import KinovaCommandMessage

class VisualizationInterface:
    def __init__(self, interface_name = "visualization_interface", publisher_name = "kinova_command", node_name = "visualization_interface"):
        self._interface_name = interface_name
        self._publisher_name = publisher_name

        rospy.init_node(node_name)
        rospy.loginfo("VisualizationInterface: ",interface_name, " is created.")
        rospy.loginfo("VisualizationInterface: Setup publisher.")
        self._command_pub = rospy.Publisher(self._publisher_name, KinovaSensorService)
        rospy.loginfo("VisualizationInterface: Setup successful!")

    def send_angular_torque_command(self, tArray):
        msg = KinovaCommandMessage()

        msg.name = "angular"
        msg.type = "torque"

        msg.header.stamp = rospy.Time.now()
        msg.data = Float32MultiArray(tArray)
        self._command_pub.publish(msg)

    def send_angular_position_command(self, qArray, limitation = None):
        msg = KinovaCommandMessage()

        msg.name = "angular"
        msg.type = "position"

        msg.header.stamp = rospy.Time.now()
        msg.data = Float32MultiArray(qArray)
        self._command_pub.publish(msg)

    def send_angular_velocity_command(self, vArray):
        msg = KinovaCommandMessage()

        msg.name = "angular"
        msg.type = "velocity"

        msg.header.stamp = rospy.Time.now()
        msg.data = Float32MultiArray(vArray)
        self._command_pub.publish(msg)

    def get_description(self):
        #Not used yet.
        pass
