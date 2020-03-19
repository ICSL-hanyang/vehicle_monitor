#! /usr/bin/env python

import sys
import PyQt5
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import uic

import rospy, rospkg
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import BatteryState

rospack = rospkg.RosPack()
vehicle_monitor = rospack.get_path('vehicle_monitor')
monitor_UI = vehicle_monitor +'/vehicle_monitor.ui'

class MainDialog(QDialog):
    def __init__(self, parent=None, flags=Qt.WindowStaysOnTopHint):
        super().__init__(parent=parent, flags=flags)
        uic.loadUi(monitor_UI, self)

        self.arm_pushButton.clicked.connect(lambda state, data  = True : self.arming(state, data))
        self.disarm_pushButton.clicked.connect(lambda state, data  = False : self.arming(state, data))
        self.offboard_pushButton.clicked.connect(lambda state, mode = 'offboard' : self.setMode(state, mode))
        self.manual_pushButton.clicked.connect(lambda state, mode = 'stabilized' : self.setMode(state, mode))
        
        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.stateDisplay)
        self.timer.start()
        self._battery = 0
        self._tar_linear_x = 0
        self._tar_angular_z = 0
        self._cur_linear_x = 0
        self._cur_angular_z = 0

        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        # rospy.wait_for_message('mavros/state', State)
        # rospy.wait_for_message('mavros/battery', BatteryState)
        # rospy.wait_for_message('mavros/setpoit_velocity/cmd_vel_unstamped', Twist)
        rospy.Subscriber('mavros/state', State, self.stateSub)
        rospy.Subscriber('mavros/battery', BatteryState, self.batterySub)
        rospy.Subscriber('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, self.tarVelocitySub)
        rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.curVelocitySub)
    
    def arming(self, state, data):
        try:
            arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            respose = arming_client(data)
        except rospy.ServiceException as e:
            alert = QMessageBox()
            alert.setText("Service call failed : " + e)
            alert.exec_()
    
    def setMode(self, state, mode):
        try:
            set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
            res = set_mode_client(0, mode)
        except rospy.ServiceException as e:
            alert = QMessageBox()
            alert.setText("Service call failed : " + e)
            alert.exec_()

    def stateDisplay(self):
        connected = 'Connected : ' + str(self._connected)
        armed = 'Armed : ' + str(self._armed)
        mode = 'Mode : ' + self._mode
        battery = 'Battery : ' + str(round(self._battery, 0)) + '%'
        cur_velocity_linear = 'Linear : ' + str(round(self._cur_linear_x, 2)) + ' m/s'
        cur_velocity_angular = 'Angula : ' + str(round(self._cur_angular_z, 2)) + ' rad/s'
        cur_velocity = 'Current Velocity\n    ' + cur_velocity_linear + '\n    ' + cur_velocity_angular
        tar_velocity_linear = 'Linear : ' + str(round(self._tar_linear_x, 2)) + ' m/s'
        tar_velocity_angular = 'Angula : ' + str(round(self._tar_angular_z, 2)) + ' rad/s'
        tar_velocity = 'Target Velocity\n    ' + tar_velocity_linear + '\n    ' + tar_velocity_angular
        self.textBrowser.setText(connected + '\n' + armed + '\n' + mode + '\n' + battery + '\n\n' + cur_velocity + '\n' + tar_velocity)

    def stateSub(self, msg):
        self._connected = msg.connected
        self._armed = msg.armed
        self._mode = msg.mode
    
    def batterySub(self, msg):
        self._battery = msg.percentage
        self._battery *= 100

    def tarVelocitySub(self, msg):
        self._tar_linear_x = msg.linear.x
        self._tar_angular_z = msg.angular.z
    
    def curVelocitySub(self, msg):
        self._cur_linear_x = msg.twist.linear.x
        self._cur_angular_z = msg.twist.angular.z


rospy.init_node("vehicle_monitor")
app = QApplication(sys.argv)
main_dialog = MainDialog()
main_dialog.show()
app.exec_()
rospy.spin()