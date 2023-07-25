import pypot.dynamixel
import numpy as np
import sys
from typing import List, Tuple
import os
from time import sleep as wait

class DynamixelServo(object):
    def __init__(self):
        self.PORT_NUMS = pypot.dynamixel.get_available_ports()
        self.PORT = self.PORT_NUMS[0]
        self.dxl_io = pypot.dynamixel.Dxl320IO(self.PORT, baudrate=1000000)

        self.m1_home = 0
        self.m2_home = 0
        self.m3_home = 0
        self.m4_home = 0

    def change_port(self, port: str) -> bool:
        self.PORT_NUMS = pypot.dynamixel.get_available_ports()
        if port in self.PORT_NUMS:
            self.dxl_io = pypot.dynamixel.Dxl320IO(port, baudrate=1000000)
            return True
        else:
            return False

    def goto_home_pos(self) -> bool:
        self.dxl_io.set_goal_position({1: self.m1_home, 2: self.m2_home, 3: self.m3_home, 4: self.m4_home})
        return True
    

    def dynamixel_control(self, roll, pitch, yaw, claw):
        self.m3_angle = (0.65*pitch) + yaw + self.m3_home
        self.m4_angle = (0.65*pitch) + yaw + self.m4_home

        if claw:
            self.m3_angle = self.m3_angle + 35
            self.m4_angle = self.m4_angle - 35
        self.m1_angle = (roll + self.m1_home) * 0.75
        self.m2_angle = pitch + self.m2_home
        self.dxl_io.set_goal_position({1: self.m1_angle, 2: self.m2_angle, 3: self.m3_angle, 4: self.m4_angle})

        print("Dynamixel Motor Angles:\n")
        print("{}, {}, {}, {}".format(
            self.m1_angle,
            self.m2_angle,
            self.m3_angle,
            self.m4_angle,
        ))

    def control_argv(self, argv: List[str] = sys.argv) -> None:
        if sys.argv[-1] == "-gotohome":
            servos.home_pos()

        elif len(sys.argv) == 5:
            servos.dynamixel_control(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), bool(sys.argv[4])),

        elif len(sys.argv) == 4:
            servos.dynamixel_control(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), False)
        
        else:
            pass
if __name__ == '__main__':
    servos = DynamixelServo()

    servos.dynamixel_control(15, 25, 40, True)
    servos.goto_home_pos()