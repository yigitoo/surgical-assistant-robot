import pypot.dynamixel
import numpy as np
import sys
from typing import List, Tuple
import os
from time import sleep as wait

class DynamixelServo(object):
    def __init__(self,motor_ids = {1,2,3,4}):
        self.PORT_NUMS = pypot.dynamixel.get_available_ports()
        self.PORT = self.PORT_NUMS[0]
        self.dxl_io = pypot.dynamixel.Dxl320IO(self.PORT, baudrate=1000000)

        self.m1_home = 0
        self.m2_home = 0
        self.m3_home = 0
        self.m4_home = 0

        self.motor_ids = motor_ids
        self.control_argv()

    def change_port(self, port: str) -> bool:
        self.PORT_NUMS = pypot.dynamixel.get_available_ports()
        if port in self.PORT_NUMS:
            self.dxl_io = pypot.dynamixel.Dxl320IO(port, baudrate=1000000)
            return True
        else:
            return False

    def goto_home_pos(self) -> bool:
        self.dxl_io.set_goal_position({1: self.m1_home, 2: self.m2_home, 3: self.m3_home, 4: self.m4_home})
        print("The robot is now on the home position.")
        return True
    
    def advanced_control(self, roll, pitch, yaw, claw):
        self.m3_angle = (0.65*pitch) + yaw + self.m3_home
        self.m4_angle = (0.65*pitch) + yaw + self.m4_home

        if claw:
            self.m3_angle = self.m3_angle + 35
            self.m4_angle = self.m4_angle - 35
        self.m1_angle = (roll + self.m1_home) * 0.75
        self.m2_angle = pitch + self.m2_home
        self.dxl_io.set_goal_position({1: self.m1_angle, 2: self.m2_angle, 3: self.m3_angle, 4: self.m4_angle})

        return [
            self.m1_angle, self.m2_angle,
            self.m3_angle, self.m4_angle
        ]

    def guide(self) -> str:
        self.filename = sys.argv[0]
        return f'''
        Usages:
        -gotohome: Go to home position # These offsets are setted automatically via File. 
        -help: Show this guide
        
        -roll: Roll angle
        -pitch: Pitch angle
        -yaw: Yaw angle
        -claw: Claw angle
        General Example:
            * {self.filename} -roll/pitch/yaw/claw [value] (can be double/float/integer/long) 

        -set-angle 
        Example: {self.filename} -set-angle x (degree) motor_id (integer)
        -set-angles: Set angles
        Example: {self.filename} -set-angles x y z t # (in degrees)
        '''
    
    def control_argv(self, argv: List[str] = sys.argv) -> None:
        if sys.argv[-1] == "-gotohome":
            self.goto_home_pos()

        elif len(sys.argv) == 5:
            self.dynamixel_control(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), bool(sys.argv[4])),

        elif len(sys.argv) == 4:
            self.dynamixel_control(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), False)
        
        elif sys.argv[-1] == '-help':
            print(self.guide())

        else:
            print("Usage: python3 dynamixel_control.py -gotohome / -help")

    def set_angles(self, m_angles: List[float]):
        if len(m_angles) <= 4:
            self.m1_angle = m_angles[0]
            self.m2_angle = m_angles[1]
            self.m3_angle = m_angles[2]
            self.m4_angle = m_angles[3]

        else:
            raise ValueError("The number of angles must be 4")

    def read_angles(self):
        self.motor_positions = self.dxl_io.get_present_position(self.motor_ids)
        return self.motor_positions

    def __str__(self):
        return str(self.guide())
    
if __name__ == '__main__':
    servos = DynamixelServo()
    servos