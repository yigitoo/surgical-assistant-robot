
import numpy as np
import rospy
import timeit

from SensorInterface import SensorInterface
from CommandInterface import CommandInterface
from KinematicsInterface import KinematicsInterface

rospy.init_node("control_interface_test")
"""
print("SensorInterface test begin.")


sensors = SensorInterface()

#qArr = sensors.get_sensor_values("positions")

#assert(type(qArr) == np.ndarray)

print("SensorInterface test is successful!")

"""
print("CommandInterface test begin.")
command = CommandInterface()
rospy.sleep(1)
com = np.array([0,0,0,0])
addrArr = np.array([16,17,18,19])

command.set_torques_zero(addrArr)
#command.send_angular_torque_command(com)
rate = rospy.Rate(20)
for i in range(0,500):
    command.send_angular_torque_command(com)
    rate.sleep()

#command.send_angular_torque_command(com)


#command.send_angular_velocity_command(com)

print("CommandInterface test is successful!")

"""
print("KinematicsInterface speed test is begin.")

kinematics = KinematicsInterface()
short_list = [1.5,1.2,1.3,2.56,2.6,3.14,1.5758,2.3]

#print(kinematics.CalcGravity(5,4,3,2,1,5,4,5))

wrapped = wrapper(kinematics.CalcGravity, 1.5,1.2,1.3,2.56,2.6,3.14,1.5758,2.3)
wrapped2 = wrapper(kinematics._num_grav_cmp_v0, 1.0,1.2,1.3)

print(timeit.timeit(wrapped, number = 1000))
print(timeit.timeit(wrapped2, number = 1000))

print("Kinematics speed test is ended.")
"""
rospy.spin()
