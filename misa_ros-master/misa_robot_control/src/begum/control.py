#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import numpy as np
from kinematics import BiopsyRobotKinematics
import CommandInterface


def publish_angles():
	global pub, kinematics, command_interface

	command = np.copy(-kinematics.q_old.T[0])
	command[2] = -command[2]
	command_interface.send_angular_position_command(command)

	print('Get Pose: ')
	print(kinematics.getPose(kinematics.q_old)[0:3,0] * 100)
	print(np.rad2deg(kinematics.getPose(kinematics.q_old)[3:6,0]))
	print('Pose: ')
	print(kinematics.pose_old[0:3]*100)
	print(np.rad2deg(kinematics.pose_old[3:6]))
	print('Angles: ')
	print(np.round(np.rad2deg(kinematics.q_old), 4))

def poseCallback(data):
	global kinematics
	pose = np.copy(kinematics.pose_old)
	dt = 0.01

	if(np.abs(data.position.x) > 0.001):
		pose[0,0] = pose[0,0] + dt*data.position.x
	if(np.abs(data.position.y) > 0.001):
		pose[1,0] = pose[1,0] + dt*data.position.y
	if(np.abs(data.position.z) > 0.001):
		pose[2,0] = pose[2,0] + dt*data.position.z

	if(np.abs(data.orientation.x) > 0.001):
		pose[3,0] = pose[3,0] + dt*data.orientation.x/10
	if(np.abs(data.orientation.y) > 0.001):
		pose[4,0] = pose[4,0] + dt*data.orientation.y/5
	if(np.abs(data.orientation.z) > 0.001):
		pose[5,0] = pose[5,0] + dt*data.orientation.z/5

	q = kinematics.ikine(pose)
	publish_angles()

def start():
	global pub, kinematics, command_interface
	kinematics = BiopsyRobotKinematics()
	rospy.init_node('position_control')
	command_interface = CommandInterface.CommandInterface()

	rospy.Subscriber('omega_force', Pose, poseCallback, queue_size=1, buff_size=2**24)

	publish_angles()
	rospy.spin()

if __name__ == '__main__':
	start()
