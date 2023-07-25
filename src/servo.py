import pypot.dynamixel
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from time import sleep

dxl_io = pypot.dynamixel.Dxl320IO('/dev/ttyUSB0')

m1_home = -13
m2_home = 18
m3_home = 0
m4_home = 22

def callback(data):
    roll = np.rad2deg(data.pose.orientation.z)
    pitch = np.rad2deg(data.pose.orientation.y)
    yaw = np.rad2deg(data.pose.orientation.x)
    claw = np.rad2deg(data.pose.position.x)
    dynamixel_control(roll, pitch, yaw, claw)

def home_pos():
    dxl_io.set_goal_position({1: m1_home, 2: m2_home, 3: m3_home, 4: m4_home})

def dynamixel_control(roll, pitch, yaw, claw):
    m3_angle = (0.65*pitch) + yaw + m3_home
    m4_angle = (0.65*pitch) + yaw + m4_home

    if claw:
        m3_angle = m3_angle + 35
        m4_angle = m4_angle - 35
        m1_angle = (roll + m1_home) * 0.75
        m2_angle = pitch + m2_home
        dxl_io.set_goal_position({1: m1_angle, 2: m2_angle, 3: m3_angle, 4: m4_angle})
    
    print("Dynamixel Motor Angles:\n")
    print("{}, {}, {}, {}".format(np.round(m1_angle, 2), np.round(m2_angle, 2), np.round(m3_angle, 3), np.round(m4_angle)))

if __name__ == '__main__':
    home_pos()
    dynamixel_control()