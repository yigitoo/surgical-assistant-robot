import pypot.dynamixel
import numpy as np
from time import sleep

class DynamixelServo(object):
    def __init__(self):
        self.dxl_io = pypot.dynamixel.Dxl320IO('COM5', baudrate=1000000)

        self.m1_home = -13
        self.m2_home = 18
        self.m3_home = 0
        self.m4_home = 22

    def home_pos(self):
        self.dxl_io.set_goal_position({1: self.m1_home, 2: self.m2_home, 3: self.m3_home, 4: self.m4_home})

    def dynamixel_control(self, roll, pitch, yaw, claw):
        self.m3_angle = (0.65*pitch) + yaw + self.m3_home
        self.m4_angle = (0.65*pitch) + yaw + self.m4_home

        if claw:
            m3_angle = m3_angle + 35
            m4_angle = m4_angle - 35
        self.m1_angle = (roll + self.m1_home) * 0.75
        m2_angle = pitch + self.m2_home
        self.dxl_io.set_goal_position({1: self.m1_angle, 2: self.m2_angle, 3: self.m3_angle, 4: self.m4_angle})

        print("Dynamixel Motor Angles:\n")
        print("{}, {}, {}, {}".format(
            np.round(self.m1_angle, 2),
            np.round(self.m2_angle, 2),
            np.round(self.m3_angle, 3),
            np.round(self.m4_angle))
        )

if __name__ == '__main__':
    import sys
    servos = DynamixelServo()

    if sys.argv[-1] == "-gotohome":
        servos.home_pos()
