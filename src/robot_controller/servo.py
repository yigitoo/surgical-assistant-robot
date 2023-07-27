import pypot.dynamixel
import numpy as np
import sys
from typing import List, Tuple
import os
import time
import json
angles = json.loads(open('/home/salih/gits/surgical-assistant-robot/src/robot_controller/servo_angles.json').read())

class DynamixelServo(object):
    def __init__(self,motor_ids = {1,2,3,4}):
        self.PORT_NUMS = pypot.dynamixel.get_available_ports()
        self.PORT = self.PORT_NUMS[0]
        self.dxl_io = pypot.dynamixel.Dxl320IO(self.PORT, baudrate=1000000)

        self.m1_home = -13
        self.m2_home = 35
        self.m3_home = 10
        self.m4_home = -149.5

        self.m1_angle = self.m1_home
        self.m2_angle = self.m2_home
        self.m3_angle = self.m3_home
        self.m4_angle = self.m4_home

        self.m_angles = [self.m1_angle, self.m2_angle, self.m3_angle, self.m4_angle]


        self.motor_ids = motor_ids
        
        self.dxl_io.set_joint_mode(ids=motor_ids)
        self.cli_arguments()

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
        self.filename = self.get_arg(0)
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
    
    def is_convertable(self, variable ,toBeConverted=float):
        try:
            result = toBeConverted(variable)
            return True
        except ValueError:
            return False

    def cli_arguments(self, argv: List[str] = sys.argv) -> None:
        if self.get_arg(-1) == '-help':
            print(self.guide())

        elif self.get_arg(-1) == "-gotohome":
            self.goto_home_pos()


        #TODO: IMPLEMENT HERE LATER.
        elif len(self.get_args()) == 3:
            if self.get_arg(1) in ['-roll', '-pitch', '-yaw', '-claw'] and self.is_convertable(self.get_arg(-1)) == True:
                value = float(self.get_arg(-1))
                self.advanced_control(value, value, value, True)

            
        elif self.get_arg(1) == '-set-angles' and len(self.get_args()) >= 6:
            if not self.get_arg(-1) == '-r':
                self.set_angles(self.get_args()[2:])
            else:
                self.set_angles(self.get_args()[2:-1])
                print(self.read_angles())
        
        elif self.get_arg(1) == '-inc-angles' and len(self.get_args()) == 6:
            self.inc_angles(self.get_args()[2:])

        elif len(self.get_args()) == 5:
            self.advanced_control(float(self.get_arg(1)), float(self.get_arg(2)), float(self.get_arg(3)), bool(self.get_arg(4))),

        elif len(self.get_args()) == 4:
            if self.get_arg(1) == '-set-angle':
                self.set_angle(int(self.get_arg(2)), float(self.get_arg(3)))
                return
            if self.get_arg(1) == '-inc-angle':
                self.inc_angle(int(self.get_arg(2)), float(self.get_arg(3)))
                return
                        
            self.advanced_control(float(self.get_arg(1)), float(self.get_arg(2)), float(self.selfget_arg(3)), False)
        
        else:
            print("Usage: python3 dynamixel_control.py -gotohome / -help")

    def get_args(self):
        return sys.argv
    
    def get_arg(self, index: int) -> str:
        try:
            return self.get_args()[index]
        except IndexError:
            return None

    def set_angle(self, motor_id, angle):
        self.set_angles([motor_id, angle])
    def open(self):
        self.set_angles(angles["open"])
    def close(self):
        self.set_angles(angles["close"])
    def set_angles(self, m_angles: List[float]):
        if len(m_angles) == 2 and self.is_convertable(m_angles[0], int):
            payload = {    
                1: self.m1_angle,
                2: self.m2_angle,
                3: self.m3_angle,
                4: self.m4_angle
            }
            
            payload[int(m_angles[0])] = float(m_angles[1])

            self.dxl_io.set_goal_position(
                payload
            )
        elif len(m_angles) == 4:
            self.m_angles = m_angles
            self.m1_angle = m_angles[0]
            self.m2_angle = m_angles[1]
            self.m3_angle = m_angles[2]
            self.m4_angle = m_angles[3]

            self.dxl_io.set_goal_position({1: self.m1_angle,2: self.m2_angle,3: self.m3_angle,4: self.m4_angle})
            time.sleep(0.8)
            self.dxl_io.set_goal_position({1: self.m1_angle,2: self.m2_angle,3: self.m3_angle,4: self.m4_angle})
            time.sleep(0.8)
            self.dxl_io.set_goal_position({1: self.m1_angle,2: self.m2_angle,3: self.m3_angle,4: self.m4_angle})
                
        else:
            raise ValueError("The number of angles must be 4")
        return True
    def inc_angle(self, motor_id, angle):
        self.inc_angles([motor_id, angle])
        return True
    def inc_angles(self, m_angles: List[float]):
        if len(m_angles) == 2 and self.is_convertable(m_angles[0], int):
            payload = { 
                1: self.m1_angle,
                2: self.m2_angle,
                3: self.m3_angle,
                4: self.m4_angle
            }
            # m_angles = [mot_id, inc_angle]
            tours = m_angles[1] // 30
            remained = m_angles[1] % 30

            if tours < 0:                    
                payload[int(m_angles[0])] += float(m_angles[1])
                self.dxl_io.set_goal_position(
                    payload
                )
            else:
                for tour in range(int(tours)):
                    payload[int(m_angles[0])] += 30
                    self.dxl_io.set_goal_position(
                        payload
                    )
                    print(tour)
                payload[int(m_angles[0])] += remained
                self.dxl_io.set_goal_position(
                    payload
                )
                print("+1")
        elif len(m_angles) == 4:
            self.m_angles += m_angles
            self.m1_angle += m_angles[0]
            self.m2_angle += m_angles[1]
            self.m3_angle += m_angles[2]
            self.m4_angle += m_angles[3]

            self.dxl_io.set_goal_position(
                { 
                    1: self.m1_angle,
                    2: self.m2_angle,
                    3: self.m3_angle,
                    4: self.m4_angle
                }
            )
        else:
            raise ValueError("The number of angles must be 4")
    
        return True
    def read_angles(self):
        self.motor_positions = self.dxl_io.get_present_position(self.motor_ids)
        return self.motor_positions

    def __str__(self):
        return str(self.guide())
    
if __name__ == '__main__':
    servos = DynamixelServo()
    servos.goto_home_pos()
    print(servos.read_angles())
    #time.sleep(3)
    #servos.set_angles(angles["open"])
    # data = json.loads(open('servo_angles.json','r').read())
    # for _ in range(5):
    #     servos.set_angles(data["open"])
    #     time.sleep(0.5)
    #     servos.set_angles(data["close"])
    #     time.sleep(0.5)
