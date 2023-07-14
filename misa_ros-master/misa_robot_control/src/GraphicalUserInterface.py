#!/usr/bin/env python

import  sys
import numpy as np

sys.path.append(sys.path[0] + "/GUI/")

from PyQt4         import QtGui, QtCore
import rospy
import GUI, ControlInterfaceGUI, SensorInterfaceGUI, TorqueControlGUI, InverseKinematicsGUI # put testUI.py in the same dir as this code
import KinematicsInterface, SensorInterface, ControllerInterface, StatusInterface

from std_msgs.msg import Bool
from threading import Thread

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

class TestWindow(QtGui.QMainWindow):
    def __init__(self):
        super(TestWindow, self).__init__()

        self.main_ui = GUI.Ui_MainWindow() # in this and next line you say that you will use all widgets from testUI over self.ui
        self.main_ui.setupUi(self)
        #so, when you say self.ui.myButton ,that is button_emergency in testUI that has name myButton
        #self.ui.emergencyButton.clicked.connect(self.DoSomething)# connect button clicked with action
        self.main_ui.button_emergency.clicked.connect(self.emergency_stop)
        self.emergency_on = True

        self.emergency_stop(True)
        self.control_torque_ui = TorqueControlGUI.Ui_Form()
        self.control_position_ui = InverseKinematicsGUI.Ui_Form()
        #self.control_torque_ui = TorqueControlGUI.Ui_Form()
        self.main_ui.radio_torque_control.clicked.connect(self.switch_control_mode)
        self.main_ui.radio_position_control.clicked.connect(self.switch_control_mode)
        self.main_ui.radio_velocity_control.clicked.connect(self.switch_control_mode)

        self.main_ui.button_grav_cmp.clicked.connect(self.torque_control_mode)
        self.main_ui.button_imp_control.clicked.connect(self.torque_control_mode)
        self.main_ui.button_manual_command.clicked.connect(self.torque_control_mode)

        self.main_ui.button_reset_torques.clicked.connect(self.reset_torques_protocol)
        self.main_ui.button_move_zero_torque.clicked.connect(self.set_calibration_position)
        self.main_ui.button_move_home.clicked.connect(self.move_home)

        self.ros_init()
        self.interfaces_init()

        if self.status_interface._emergency_mode:
            self.emergency_stop(bool = True)

    def position_manual_command(self):
        for i in range (0,4):
            print i

    def set_calibration_position(self):
        command = np.array([0,-90,90,0])
        self.controller_interface._command_interface.send_angular_position_command(command, type = "deg")

    def move_home(self):
        command = np.array([0,0,0,0])
        self.controller_interface._command_interface.send_angular_position_command(command)

    def reset_torques_protocol(self):
        self.controller_interface.set_all_torques_zero()
        command = np.array([0,0,0,0])
        self.controller_interface._command_interface.send_angular_torque_command(command)

    def sensor_gui_callback(self, qArr, vArr, aArr, tArr):
        for i in range (0, qArr.shape[0]):
            self.main_ui.table_sensors.item(i,0).setText(str(qArr[i]))
            self.main_ui.table_sensors.item(i,1).setText(str(vArr[i]))
            self.main_ui.table_sensors.item(i,2).setText(str(aArr[i]))
            self.main_ui.table_sensors.item(i,3).setText(str(tArr[i]))

    def controller_gui_callback(self, tArr, errArr):
        for i in range(errArr.shape[0]):
            #Position error is not provided yet.
            self.main_ui.table_grav_errors.item(i,0).setText("0")
            self.main_ui.table_grav_errors.item(i,1).setText(str(tArr[i]))
            self.main_ui.table_grav_errors.item(i,2).setText(str(errArr[i]))

    def torque_control_mode(self):
        sender = self.sender()
        if sender.text() == "Gravity Compensation":
            self.controller_interface.switch_control_mode(0)
        elif sender.text() == "Impedance Control":
            self.controller_interface.switch_control_mode(1)
        elif sender.text() == "Manual Command":
            self.controller_interface.switch_control_mode(2)

        if not self.controller_interface._start:
            self.controller_interface._start = True

    def control_mode_velocity(self):
        pass

    def switch_control_mode(self):
        sender = self.sender()
        if sender.text() == "Torque Control" :
            self.main_ui.widget_control_interface.setCurrentIndex(0)
        elif sender.text() == "Position Control":
            self.main_ui.widget_control_interface.setCurrentIndex(1)
        elif sender.text() == "Velocity Control":
            self.main_ui.widget_control_interface.setCurrentIndex(2)

    def emergency_stop(self, bool):
        if self.main_ui.button_emergency.text() == "Emergency Stop":
            #Emergency Stop protocol
            self.main_ui.widget_system_light.setStyleSheet(_fromUtf8("background-color: rgb(255, 0, 0);"))
            self.main_ui.button_emergency.setText("Activate System")
            self.switch_emergency_situation(is_emergency_mode = True)
        else:
            #Activation protocol
            self.main_ui.widget_system_light.setStyleSheet(_fromUtf8("background-color: rgb(0, 170, 0);"))
            self.main_ui.button_emergency.setText("Emergency Stop")
            self.switch_emergency_situation(is_emergency_mode = False)

    def ros_init(self):
        rospy.init_node("graphical_user_interface")
        self.emergency_pub = rospy.Publisher("kinova_emergency_situation", Bool , queue_size = 10)

    def switch_emergency_situation(self, is_emergency_mode):
        if not self.emergency_on == is_emergency_mode:
            self.emergency_pub.publish(Bool(is_emergency_mode))
            self.emergency_on = is_emergency_mode

    def update_interface_statuses(self, interface_statuses):

        for interface in interface_statuses:
            #Find index the corresponding interface

            matched_items = self.main_ui.table_statuses.findItems(interface, QtCore.Qt.MatchRecursive)
            if not len(matched_items) == 0:
                item = matched_items[0]
                status = interface_statuses.get(interface)
                is_functional = status[0]
                is_emergency_mode = status[1]

                if is_functional:
                    if is_emergency_mode:
                        item.setTextColor(0, QtGui.QColor(128,128,0))
                    else:
                        item.setTextColor(0, QtGui.QColor(0,170,0))
                else:
                    item.setTextColor(0, QtGui.QColor(255,0,0))

    def manual_torque_command(self):
        pass

    def status_error_callback(self, interface_statuses, error_msg):
        table = self.main_ui.table_errors
        rowPosition = table.rowCount()
        table.insertRow(rowPosition)

        table.setItem(rowPosition , 0, QtGui.QTableWidgetItem(error_msg.source))
        table.setItem(rowPosition , 1, QtGui.QTableWidgetItem(error_msg.error))
        table.setItem(rowPosition , 2, QtGui.QTableWidgetItem(str(error_msg.priority)))
        self.update_interface_statuses(interface_statuses)

    def status_info_callback(self, interface_statuses, info_msg):
        table = self.main_ui.table_infos

        rowPosition = table.rowCount()
        table.insertRow(rowPosition)

        table.setItem(rowPosition , 0, QtGui.QTableWidgetItem(info_msg.source))
        table.setItem(rowPosition , 1, QtGui.QTableWidgetItem(info_msg.reason))
        table.setItem(rowPosition , 2, QtGui.QTableWidgetItem(str(info_msg.functional)))
        table.setItem(rowPosition , 3, QtGui.QTableWidgetItem(str(info_msg.emergency)))


        self.update_interface_statuses(interface_statuses)

    def interfaces_init(self):
        self.reg_interface_name_list = ["Controller Interface", "Command Interface", "Sensor Interface", "Kinematics Interface", "Kinova API Interface"]

        self.status_interface = StatusInterface.StatusInterface(self.reg_interface_name_list, GUI = True)

        self.status_interface.set_gui_info_callback(self.status_info_callback)
        self.status_interface.set_gui_error_callback(self.status_error_callback)
        rospy.sleep(3)
        self.controller_interface = ControllerInterface.ControllerInterface(GUI = True)

        self.controller_interface._sensor_interface.set_gui_callback(self.sensor_gui_callback)
        self.controller_interface.set_gui_callback(self.controller_gui_callback)


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = TestWindow()
    window.show()
    sys.exit(app.exec_())
