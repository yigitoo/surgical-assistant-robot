#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
from numpy.linalg import det
import numpy.matlib

#Joint Dimensions
Lse = 0.18022
Lpe = 0.061 + 0.005/2 + 0.119221

L0x = 0.061
L1x = Lpe
L1z = Lpe
L2y = Lpe
L2z = Lpe
L3x = Lse
L3y = Lse
L4x = 0.120063
L5z = 0.485
L6z = 0.0096
L7z = 0.01183

class BiopsyRobotKinematics:
	def __init__(self):
		self.q_old = np.matrix([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
		self.pose_old = self.getPose(self.q_old)
		q = self.ikine(self.pose_old)
		print(self.q_old)
		print(q)
		print(self.pose_old)

	#Transformation Matrices
	def transYaw(self, a, d):
		T = np.matrix([[np.cos(a), -np.sin(a), 0, d[0]],
				[np.sin(a), np.cos(a), 0, d[1]],
				[0, 0, 1, d[2]],
				[0,0,0,1]])
		return T

	def transPitch(self, a, d):
		T = np.matrix([[np.cos(a), 0, np.sin(a), d[0]],
				[0, 1, 0, d[1]],
				[-np.sin(a), 0, np.cos(a), d[2]],
				[0,0,0,1]])
		return T

	def transRoll(self, a, d):
		T = np.matrix([[1, 0, 0, d[0]],
				[0, np.cos(a), -np.sin(a), d[1]],
				[0, np.sin(a), np.cos(a), d[2]],
				[0,0,0,1]])
		return T

	def fkine(self, q):
		T1 = self.transRoll(q[0,0], [L0x, 0, 0])
		T2 = self.transYaw(q[1,0], [L1x, 0, L1z])
		T3 = self.transPitch(q[2,0], [0, -L2y, L2z])
		T4 = self.transRoll(q[3,0], [L3x, -L3y, 0])
		T5 = self.transYaw(q[4,0], [L4x, 0, 0])
		T6 = self.transPitch(q[5,0], [0, 0, -L5z])
		T7 = self.transRoll(q[6,0], [0, 0, -L6z])
		T8 = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,-L7z,0],[0,0,0,1]])
		return np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T1,T2),T3),T4),T5),T6),T7),T8)

	def getPose(self, q):
		FK = self.fkine(q)
		x = FK[0,3]
		y = FK[1,3]
		z = FK[2,3]

		Wbz = 0.0
		if(abs(FK[2,0] + 1) <= 1.e-8 + 1.e-5 * abs(-1)):
			Wby = np.pi/2
		elif(abs(FK[2,0] - 1) <= 1.e-8 + 1.e-5 * abs(1)):
			Wby = -np.pi/2
		else:
			Wby = -np.arcsin(FK[2,0])
			Wbz = np.arctan2(FK[1,0]/np.cos(Wby), FK[0,0]/np.cos(Wby))

		Wbx = np.arctan2(FK[0,0], -FK[1,0])
		return np.matrix([[x],[y],[z],[Wbx],[Wby],[Wbz],[0.0]])

	#Create a function from txt file
	def build_function(self, filename):
		with open(filename, 'rU') as f:
			eqn = f.read().strip()
			exec("def jacobian(q1,q2,q3,q4,q5,q6,q7):\n return ({})".format(eqn))
			return locals()['jacobian']

	#Return jacobian inverse for different cases
	def jacobianInv(self, q):
		q1 = q[0,0]
		q2 = q[1,0]
		q3 = q[2,0]
		q4 = q[3,0]
		q5 = q[4,0]
		q6 = q[5,0]
		q7 = q[6,0]
		jacobian = self.build_function('jacobian.txt')
		return inv(jacobian(q1,q2,q3,q4,q5,q6,q7))

	#Inverse Kinematics
	def ikine(self, pose):
		#Calculate inverse kinematics
		Jinv = self.jacobianInv(self.q_old)
		if(not(det(Jinv) == 0)):
			q = self.q_old + np.matmul(Jinv,np.subtract(pose, self.pose_old))

			#Error calculation
			# q_diff = self.calcError(pose, Jinv, q)
			# q = q + q_diff

			#Update global angles and poses
			self.q_old = np.copy(q)

			self.pose_old = np.copy(pose)
			return q
		else:
			print 'Singularity'
			return self.q_old

	def calcError(self, pose, Jinv, q):
		error_margin = np.matrix([[0.008],[0.008],[0.008],[0.008],[0.008],[0.008],[0.008]])
		K = 0.9

		#Get pose of the output
		fk_output = self.getPose(q)

		#Calculate error
		t_error = K*(pose - fk_output)
		q_diff = np.matmul(Jinv, t_error)
		tmp_bool_array = np.abs(q_diff) < error_margin
		for i in range(6):
			if(tmp_bool_array[i]):
				q_diff[i] = 0
		return q_diff
