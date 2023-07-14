#!/usr/bin/env python


import itertools
import numpy as np
import rospy

import pypot.dynamixel
import serial.tools.list_ports

from std_msgs.msg import Float32MultiArray
from copy import deepcopy

#class Endowrist():

class Endowrist():

    def __init__(self):

        self._usb2dyn = self._init_usb2dyn_device()
        self.servo_ids = self._get_servo_ids(4)

        speed_limit = dict(zip(self.servo_ids, itertools.repeat(200)))
        self.set_speed_limit(speed_limit)
        self._usb2dyn.enable_torque(self.servo_ids)

        self.m_offsets = [-21.85,-52.63,-20.67,-8.7]

        init_motors = dict(zip(self.servo_ids, self.m_offsets))
        self._usb2dyn.set_goal_position(init_motors)
        pos = self._usb2dyn.get_present_position(self.servo_ids)
        self._init_ros()

    def set_speed_limit(self,speed_limits):

        self.speed_limits = deepcopy(speed_limits)
        self._usb2dyn.set_moving_speed(self.speed_limits)
        print("Speed limit is set to :")
        print(str(self.speed_limits))

    def _init_ros(self):
        # Initializes the ros node, publishers and subsribers
        queue_size = 100
        self.endowrist_angle_pub = rospy.Publisher("endowrist_motor_angles", Float32MultiArray, queue_size = queue_size)

        print("ROS Initialization Completed.")
        rospy.loginfo("Endowrist initialized.")

    def _angle_command(self,angle_command_msg):
        """
            The callback of the 'endowrist_angle_command topic.
            The inputs:
                angle_command_msg : Float32MultiArray
                    data : servo angle in radians- order is important data[n] contains servo_ids[n]'s command.
        """
        self.curr_angles = np.copy(angle_command_msg)
        angle_in_deg =  np.rad2deg(self.curr_angles[0:3])
        motor_commands = self.rpy_to_motor_commands(roll = angle_in_deg[0], pitch = angle_in_deg[1], yaw = angle_in_deg[2], claw = 0)

        self.goal_pos = dict(zip(self.servo_ids, motor_commands))
        self._usb2dyn.set_goal_position(self.goal_pos)

    def rpy_to_motor_commands(self,roll,pitch,yaw,claw):
        """
            Since the motors are not directly makes roll,pitch or yaw,
            A transformation matrice defined. It is the definition of how behaves
            each motor rotates on the all axis. Constants are not finalized yet.
        """
        ky1 = 1
        ky2 = 1

        kp1 = 0.67
        kp2 = 0.67
        kp3 = 1
        kT = 0.75

        T = np.matrix([[0,kp1,ky1,self.m_offsets[0]],
                        [0, kp2, ky2, self.m_offsets[1]],
                        [kT, 0, 0, self.m_offsets[2]],
                        [0, kp3, 0, self.m_offsets[3]]])
        R = np.matrix([[roll],[pitch],[yaw],[1]])
        motor_matrix = deepcopy(np.matmul(T,R))
        if claw == 1:
            motor_matrix[0] += 60
            motor_matrix[1] -= 60
        elif claw == -1:
            motor_matrix[0] -= 60
            motor_matrix[1] += 60

        return motor_matrix

    def _init_usb2dyn_device(self):
        """
            Initialization of the Usb2Dynamixel device, this devices serial port
            description is 'FT232R USB UART'.
            Method errors when device could not find on the serial bus.
        """
        _usb2dyn_port = self._find_serial_port('FT232R USB UART')

        if _usb2dyn_port is None:
            raise IOError("Usb2dynamixel device could not find. Aborting the process.")

        else:
            print "Connection succesful to _usb2dynamixel device on port " + _usb2dyn_port.device

        _usb2dyn = pypot.dynamixel.Dxl320IO(_usb2dyn_port.device)
        return _usb2dyn

    def _get_servo_ids(self, servo_count):

        print("System is scanning the all servo motors.")
        found_ids = self._usb2dyn.scan([1,2,3,4])

        if len(found_ids) < servo_count:
                raise IOError("The device could not find " + str(servo_count) + " servo/servos. Please make sure all all them are connected.")
        servo_ids = found_ids[:servo_count]

        return deepcopy(servo_ids)

    def _find_serial_port(self,name):

        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if p.description == name:
                return deepcopy(p)

        return None

    def _manual_calibration(self):
        init_motors = dict(zip(self.servo_ids, itertools.repeat(0)))

        self.m_offsets = [-21.85,-52.63,-20.67,-8.7]

        is_calibrated = False
        index = 1
        while not is_calibrated:
            char = input(str(index) + " " + str())

            if char >= 1 and char <= 4:
                index = deepcopy(char)
                print("Index " + str(index) + " has set.")

            elif char == 9:
                    init_motors[index] += 1
                    self._usb2dyn.set_goal_position(init_motors)
                    print(init_motors)
            elif char == 8:
                    init_motors[index] -= 1
                    self._usb2dyn.set_goal_position(init_motors)
                    print(init_motors)
            elif char == 6:
                is_calibrated = True
                break
            else:
                print("illegal")

    def get_position(self):
        return self._usb2dyn.get_present_position(self.servo_ids)
            #msg = Float32MultiArray()
            #msg.data = self._usb2dyn.get_present_position(self.servo_ids)
            #self.endowrist_angle_pub.publish(msg)
