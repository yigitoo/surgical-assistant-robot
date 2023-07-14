
#!/usr/bin/env python
import rospy
import time
import numpy as np

#from sensor_msgs.msg import JointState
from std_msgs.msg import Bool,Int32

from copy import deepcopy

from misa_files.msg import KinovaJointStateMessage
from std_msgs.msg import Float32MultiArray

import SensorInterface, CommandInterface, KinematicsInterface, ErrorLogger, InfoLogger

from threading import Thread,Lock

class ControllerInterface:
    def __init__(self, interface_name = "Controller Interface", GUI = False):

        self._interface_name = interface_name
        self._sensor_interface = SensorInterface.SensorInterface(GUI = GUI)
        self._command_interface = CommandInterface.CommandInterface()
        self._kinematic_interface = KinematicsInterface.KinematicsInterface()
        #self._visualization_interface = VisualizationInterface.VisualizationInterface()
        self._emergency_pub = rospy.Subscriber("kinova_emergency_situation", Bool, self.emergency_check)
        self._set_torque_zero = rospy.Publisher("kinova_set_torque_zero", Int32, queue_size = 10)
        self.actuator_addresses = [16,17,18,19]

        self._theoric_plot_pub = rospy.Publisher("kinova_theoric_values", Float32MultiArray, queue_size = 10)
        self._numeric_plot_pub = rospy.Publisher("kinova_experimental_values", Float32MultiArray, queue_size = 10)

        self._theoric_plot_msg = Float32MultiArray()
        self._numeric_plot_msg = Float32MultiArray()

        self.emergency_mode = False
        self.GUI = GUI
        self._freq = 100
        self._rate = rospy.Rate(self._freq)

        self._control_mode = 0

        self._info_logger = InfoLogger.InfoLogger(owner_name = self._interface_name)
        self._error_logger = ErrorLogger.ErrorLogger(owner_name = self._interface_name)
        self._start = False

        self._info_logger.report_info(source = self._interface_name, reason = "setup")

        self.start_close_loop()

    def set_torque_zero(self,addr):
        self._set_torque_zero.publish(Int32(addr))

    def set_all_torques_zero(self):
        for addr in self.actuator_addresses:
            self._set_torque_zero.publish(Int32(addr))

    def gui_callback(self, tArr, errArr):
        pass

    def set_gui_callback(self, callback):
        self.gui_callback = callback

    def start_close_loop(self):
        self.thread_close_loop = Thread(target = self.close_control_loop)
        self.thread_close_loop.start()

    def close_control_loop(self):
        while not rospy.is_shutdown():
            if self._start and not self.emergency_mode:
                if self._control_mode == 0:
                    tArr, errArr = self.gravity_compensation()
                    if self.GUI:
                        self.gui_callback(tArr, errArr)

                elif self._control_mode == 1:
                    self.impedance_control()
                    """
                    if self.GUI:
                        self.gui_callback(tArr, err)
                    """
                elif self._control_mode == 2:
                    pass

            self._rate.sleep()



    def switch_control_mode(self, mode):
        self._control_mode = mode

    def set_freq(self, freq):
        self._freq = freq
        self._rate = rospy.Rate(freq)

    def calc_torque(self,qArray,vArray,aArray):
        return self._kinematic_interface.calculate()

    def impedance_control(self):
        """
        Pseudo Code of Impedance Control
        WARNING: Velocity limit is not implemented in this pseudo code.
        for each step:
            Take qArr, vArr, aArr from sensor interface
            Compute the theoric torque calculations.
            Take tArr from sensor interface
            Compute external torque which is ex_tau = tArr - tau;
            Compute the impedance(Z) values by ex_tau = Z * V; => Z = ex_tau / V
            Check impedance limits, if any exceeds, then switch system into gravity compensation mode.
        """
        print("impedance control loop")
        return
        qArr,vArr,aArr,tArr = self._sensor_interface.get_joint_states()
        tauArr = self._kinematic_interface.calculate_torques(qArr,vArr,aArr)
        ex_tauArr = np.subtract(tArr,tauArr)

        impArr = np.divide(ex_tauArr, vArr)

        if index in range(0,size(impArr)):
            np.abs(impArr[index]) > imp_limit_arr[i]
            self.emergency_stop()
            return


    def gravity_compensation(self):
        """
        Pseudo code of the gravity compensation mode.
        WARNING: This function sticks the robots arm to a particular position.
        for each step:
            Compute the theoric gravitational compensation values gcArr -*> tArr
            Take qArr, qArr, vArr, aArr from the sensor interface.
            Compute the error and print it.
            Feed tArr to command interface.
        """
        qArr, _, _, tArr = self._sensor_interface.get_joint_states()
        qArr[2] = -qArr[2]
        gcArr = self._kinematic_interface.calc_grav_composite(qArr = deepcopy(-qArr))

        gcArr[2] = -gcArr[2]

        self._command_interface.send_angular_torque_command(tArr = gcArr)
        self._theoric_plot_msg.data = deepcopy(gcArr)
        self._numeric_plot_msg.data = deepcopy(tArr)

        self._theoric_plot_pub.publish(self._theoric_plot_msg)
        self._numeric_plot_pub.publish(self._numeric_plot_msg)

        self._grav_cmp_errArr = np.subtract(gcArr,tArr)
        return tArr, self._grav_cmp_errArr

    def get_ex_tau(self):
        """
        Pseudo Code of External Force Computation
        WARNING: Velocity limit is not implemented in this pseudo code.
        for each step:
            Take qArr, vArr, aArr from sensor interface
            Compute the theoric torque calculations.
            Compute external torque which is ex_tau = tArr - tau;
        """
        pass

    def get_torques(self):
        #Torque coming from sensors.
        return self._sensor_interface.get_sensor_values("torques")

    def get_angles(self):
        #Sensor angles
        return self._sensor_interface.get_sensor_values("positions")

    def get_vels(self):
        return self._sensor_interface.get_sensor_values("velocities")

    def get_accs(self):
        return self._sensor_interface.get_sensor_values("accelerations")

    def emergency_check(self, msg):
        self.emergency_mode = msg.data
        if self.emergency_mode:
            self._info_logger.report_info(source = self._interface_name, reason = "emergency_on", is_functional = True, emergency_mode = True)
        else:
            self._info_logger.report_info(source = self._interface_name, reason = "emergency_off", is_functional = True, emergency_mode = False)
