#!/usr/bin/env python

import rospy
from misa_files.msg import KinovaJointStateMessage

import random

rospy.init_node("fake_sensor_publisher")

pub = rospy.Publisher("kinova_joint_states", KinovaJointStateMessage, queue_size = 10)
msg = KinovaJointStateMessage()
freq = 100

rate = rospy.Rate(freq)

def continously_publish():
    global msg
    while not rospy.is_shutdown():
        msg.positions = [random.randint(0,360), random.randint(0,360), random.randint(0,360), random.randint(0,360)]
        msg.velocities = [random.randint(0,10),random.randint(0,10),random.randint(0,10),random.randint(0,10)]
        msg.accelerations = [random.randint(0,3),random.randint(0,3),random.randint(0,3),random.randint(0,3)]
        msg.torques = [random.randint(0,10),random.randint(0,10),random.randint(0,10),random.randint(0,1)]
        pub.publish(msg)
        rate.sleep()

continously_publish()
