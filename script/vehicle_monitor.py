#! /usr/bin/env python

import sys
import PyQt5
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import uic

import rospy, rospkg
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from sensor_msgs.msg import BatteryState
from swarm_ctrl_pkg.srv import srvGoToVehicle, srvGoToVehicleRequest, srvGoToVehicleResponse, srvMultiSetpointLocal, srvMultiSetpointLocalRequest, srvMultiSetpointLocalResponse

rospack = rospkg.RosPack()
vehicle_monitor = rospack.get_path('vehicle_monitor')
monitor_UI = vehicle_monitor +'/vehicle_monitor.ui'

class Drone:
    def __init__(self, name):
        self._name = name
        att_path = name + '/local_plan/attractive'
        rep_path = name + '/local_plan/repulsive'
        v_rep_path = name + '/local_plan/repulsive_vel'
        setpoint_path = name + '/mavros/setpoint_velocity/cmd_vel_unstamped'
        self._att_sub = rospy.Subscriber(att_path, Vector3, self.attSub)
        self._rep_sub = rospy.Subscriber(rep_path, Vector3, self.repSub)
        self._r_vel_sub = rospy.Subscriber(v_rep_path, Vector3, self.rVelSub)
        self._set_vel_sub = rospy.Subscriber(setpoint_path, Twist, self.setVelSub)
        self._att = Vector3()
        self._rep = Vector3()
        self._v_rep = Vector3()
        self._set_vel = Twist()

    def attSub(self, msg):
        self._att = msg

    def repSub(self, msg):
        self._rep = msg
    
    def rVelSub(self, msg):
        self._v_rep = msg

    def setVelSub(self, msg):
        self._set_vel = msg

    def getState(self):
        att = self._name + ' att : ' + str(round(self._att.x, 2)) + ', ' + str(round(self._att.y, 2)) + ', ' + str(round(self._att.z, 2))
        rep = self._name + ' rep : ' + str(round(self._rep.x, 2)) + ', ' + str(round(self._rep.y, 2)) + ', ' + str(round(self._rep.z, 2))
        r_vel = self._name + ' r vel : ' + str(round(self._v_rep.x, 2)) + ', ' + str(round(self._v_rep.y, 2)) + ', ' + str(round(self._v_rep.z, 2))
        set_vel = self._name + ' set vel : ' + str(round(self._set_vel.linear.x, 2)) + ', ' + str(round(self._set_vel.linear.y, 2)) + ', ' + str(round(self._set_vel.linear.z, 2))
        return att + '\n' + rep + '\n' + r_vel + '\n' + set_vel

class MainDialog(QDialog):
    def __init__(self, parent=None, flags=Qt.WindowStaysOnTopHint):
        super().__init__(parent=parent, flags=flags)
        uic.loadUi(monitor_UI, self)

        self.arm_pushButton.clicked.connect(lambda state, data  = True : self.arming(state, data))
        self.disarm_pushButton.clicked.connect(lambda state, data  = False : self.arming(state, data))
        self.offboard_pushButton.clicked.connect(lambda state, mode = 'offboard' : self.setMode(state, mode))
        self.goToButton.clicked.connect(self.goTo)
        self.goToButton_2.clicked.connect(self.goTo2)
        self.goToButton_3.clicked.connect(self.goTo3)
        self.checkBox.toggle()
        self.checkBox.stateChanged.connect(self.checkBoxState)
        if rospy.get_param('swarm_node/local_plan/use_repulsive_force'):
            self.checkBox_rep.toggle()
        self.checkBox_rep.stateChanged.connect(self.checkBoxRep)
        if rospy.get_param('swarm_node/local_plan/use_adaptive'):
            self.checkBox_rep_vel.toggle()
        self.checkBox_rep_vel.stateChanged.connect(self.checkBoxRepVel)
        if rospy.get_param('swarm_node/use_velocity_controller'):
            self.checkBox_vel.toggle()
        self.checkBox_vel.stateChanged.connect(self.checkBoxVel)
        self.lineEdit_formation.setText("POINT")
        self.lineEdit_max_speed.setText(str(rospy.get_param('swarm_node/local_plan/max_speed')))
        self.lineEdit_max_speed.textChanged.connect(self.lineEditMaxSpeed)
        self.lineEdit_sensing_range.setText(str(rospy.get_param('swarm_node/local_plan/sensing_range')))
        self.lineEdit_sensing_range.textChanged.connect(self.lineEditSensingRange)
        self.lineEdit_safety_range.setText(str(rospy.get_param('swarm_node/local_plan/safety_range')))
        self.lineEdit_safety_range.textChanged.connect(self.lineEditSafetyRange)
        self.lineEdit_kp_att.setText(str(rospy.get_param('swarm_node/local_plan/kp_attractive')))
        self.lineEdit_kp_att.textChanged.connect(self.lineEditKpAtt)
        self.lineEdit_kp_rep.setText(str(rospy.get_param('swarm_node/local_plan/kp_repulsive')))
        self.lineEdit_kp_rep.textChanged.connect(self.lineEditKpRep)
        self.lineEdit_ki_rep.setText(str(rospy.get_param('swarm_node/local_plan/ki_repulsive')))
        self.lineEdit_ki_rep.textChanged.connect(self.lineEditKiRep)
        self.lineEdit_kd_rep.setText(str(rospy.get_param('swarm_node/local_plan/kd_repulsive')))
        self.lineEdit_kd_rep.textChanged.connect(self.lineEditKdRep)
        self.lineEdit_kp_r_vel.setText(str(rospy.get_param('swarm_node/local_plan/kp_repulsive_vel')))
        self.lineEdit_kp_r_vel.textChanged.connect(self.lineEditKpRepVel)
        self.horizontalSlider_max_speed.setRange(0, 100)
        self.horizontalSlider_max_speed.setSingleStep(1)
        self.horizontalSlider_max_speed.setValue(int(rospy.get_param('swarm_node/local_plan/max_speed')*20))
        self.horizontalSlider_max_speed.valueChanged.connect(self.sliderMaxSpeed)
        self.horizontalSlider_sensing_range.setRange(0, 100)
        self.horizontalSlider_sensing_range.setSingleStep(1)
        self.horizontalSlider_sensing_range.setValue(int(rospy.get_param('swarm_node/local_plan/sensing_range')*10))
        self.horizontalSlider_sensing_range.valueChanged.connect(self.sliderSensingRange)
        self.horizontalSlider_safety_range.setRange(0, 100)
        self.horizontalSlider_safety_range.setSingleStep(1)
        self.horizontalSlider_safety_range.setValue(int(rospy.get_param('swarm_node/local_plan/safety_range')*10))
        self.horizontalSlider_safety_range.valueChanged.connect(self.sliderSafetyRange)
        self.horizontalSlider_kp_att.setRange(0, 100)
        self.horizontalSlider_kp_att.setSingleStep(1)
        self.horizontalSlider_kp_att.setValue(int(rospy.get_param('swarm_node/local_plan/kp_attractive')*50))
        self.horizontalSlider_kp_att.valueChanged.connect(self.sliderKpAtt)
        self.horizontalSlider_kp_rep.setRange(0, 100)
        self.horizontalSlider_kp_rep.setSingleStep(1)
        self.horizontalSlider_kp_rep.setValue(int(rospy.get_param('swarm_node/local_plan/kp_repulsive')*50))
        self.horizontalSlider_kp_rep.valueChanged.connect(self.sliderKpRep)
        self.horizontalSlider_ki_rep.setRange(0, 100)
        self.horizontalSlider_ki_rep.setSingleStep(1)
        self.horizontalSlider_ki_rep.setValue(int(rospy.get_param('swarm_node/local_plan/ki_repulsive')*50))
        self.horizontalSlider_ki_rep.valueChanged.connect(self.sliderKiRep)
        self.horizontalSlider_kd_rep.setRange(0, 100)
        self.horizontalSlider_kd_rep.setSingleStep(1)
        self.horizontalSlider_kd_rep.setValue(int(rospy.get_param('swarm_node/local_plan/kd_repulsive')*50))
        self.horizontalSlider_kd_rep.valueChanged.connect(self.sliderKdRep)
        self.horizontalSlider_kp_r_vel.setRange(0, 100)
        self.horizontalSlider_kp_r_vel.setSingleStep(1)
        self.horizontalSlider_kp_r_vel.setValue(int(rospy.get_param('swarm_node/local_plan/kp_repulsive_vel')*50))
        self.horizontalSlider_kp_r_vel.valueChanged.connect(self.sliderKpRepVel)
        
        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.stateDisplay)
        self.timer.start()
        self._battery = 0
        self._tar_linear_x = 0
        self._tar_angular_z = 0
        self._cur_linear_x = 0
        self._cur_angular_z = 0
        self._swarm_formation = ""
        self._swarm_control = True

        self.arming_pub = rospy.Publisher('multi/arming', Bool, queue_size=10)
        self.set_mode_pub = rospy.Publisher('multi/set_mode', String, queue_size=10)

        self._drone_num = rospy.get_param('swarm_node/num_drone')
        self._drones = []
        for i in range(self._drone_num):
            name = 'camila' + str(i+1)
            self._drones.append(Drone(name))


    def arming(self, state, data):
        self.arming_pub.publish(data)
    
    def setMode(self, state, mode):
        self.set_mode_pub.publish(mode)

    def goTo(self):
        self._swarm_target_x = float(self.lineEdit_2.text()) 
        self._swarm_target_y = float(self.lineEdit_3.text()) 
        self._swarm_target_z = float(self.lineEdit_4.text()) 
        
        if self._swarm_control == True:
            self._swarm_formation = self.lineEdit_formation.text()
            try:
                goto_client = rospy.ServiceProxy('multi_setpoint_local', srvMultiSetpointLocal)
                msg = srvMultiSetpointLocalRequest(self._swarm_formation, self._swarm_target_x, self._swarm_target_y, self._swarm_target_z)
                res = goto_client(msg)
            except rospy.ServiceException as e:
                print("Service call failed: %s", e)

        else:
            
            try:
                drone_num = int(self.lineEdit.text())
                gotov_client = rospy.ServiceProxy('goto_vehicle', srvGoToVehicle)
                msg = srvGoToVehicleRequest(drone_num, self._swarm_target_x, self._swarm_target_y, self._swarm_target_z)
                res = gotov_client(msg)
            except rospy.ServiceException as e:
                print("Service call failed: %s", e)
    
    def goTo2(self):
        drone_num = int(self.lineEdit_5.text())
        self._swarm_target_x = float(self.lineEdit_6.text())
        self._swarm_target_y = float(self.lineEdit_7.text())
        self._swarm_target_z = float(self.lineEdit_8.text())            
        try:
            gotov_client = rospy.ServiceProxy('goto_vehicle', srvGoToVehicle)
            msg = srvGoToVehicleRequest(drone_num, self._swarm_target_x, self._swarm_target_y, self._swarm_target_z)
            res = gotov_client(msg)
        except rospy.ServiceException as e:
            print("Service call failed: %s", e)
    
    def goTo3(self):
        drone_num = int(self.lineEdit_10.text())
        self._swarm_target_x = float(self.lineEdit_11.text())
        self._swarm_target_y = float(self.lineEdit_12.text())
        self._swarm_target_z = float(self.lineEdit_13.text())            
        try:
            gotov_client = rospy.ServiceProxy('goto_vehicle', srvGoToVehicle)
            msg = srvGoToVehicleRequest(drone_num, self._swarm_target_x, self._swarm_target_y, self._swarm_target_z)
            res = gotov_client(msg)
        except rospy.ServiceException as e:
            print("Service call failed: %s", e)

    def checkBoxState(self):
        self._swarm_control = self.checkBox.isChecked()
        if self._swarm_control == False:
            try:
                goto_client = rospy.ServiceProxy('multi_setpoint_local', srvMultiSetpointLocal)
                msg = srvMultiSetpointLocalRequest("SINGLE", 0, 0, 0)
                res = goto_client(msg)
            except rospy.ServiceException as e:
                print("Service call failed: %s", e)
    
    def checkBoxRep(self):
        state = self.checkBox_rep.isChecked()
        rospy.set_param('swarm_node/local_plan/use_repulsive_force', state)
    
    def checkBoxRepVel(self):
        state = self.checkBox_rep_vel.isChecked()
        rospy.set_param('swarm_node/local_plan/use_adaptive', state)

    def checkBoxVel(self):
        state = self.checkBox_vel.isChecked()
        rospy.set_param('swarm_node/use_velocity_controller', state)

    def lineEditMaxSpeed(self):
        self.horizontalSlider_max_speed.setValue(int(float(self.lineEdit_max_speed.text())*20))
        rospy.set_param('swarm_node/local_plan/max_speed', float(self.lineEdit_max_speed.text()))

    def lineEditSensingRange(self):
        self.horizontalSlider_sensing_range.setValue(int(float(self.lineEdit_sensing_range.text())*10))
        rospy.set_param('swarm_node/local_plan/sensing_range', float(self.lineEdit_sensing_range.text()))
    
    def lineEditSafetyRange(self):
        self.horizontalSlider_safety_range.setValue(int(float(self.lineEdit_safety_range.text())*10))
        rospy.set_param('swarm_node/local_plan/safety_range', float(self.lineEdit_safety_range.text()))
    
    def lineEditKpAtt(self):
        self.horizontalSlider_kp_att.setValue(int(float(self.lineEdit_kp_att.text())*50))
        rospy.set_param('swarm_node/local_plan/kp_attractive', float(self.lineEdit_kp_att.text()))

    def lineEditKpRep(self):
        self.horizontalSlider_kp_rep.setValue(int(float(self.lineEdit_kp_rep.text())*50))
        rospy.set_param('swarm_node/local_plan/kp_repulsive', float(self.lineEdit_kp_rep.text()))

    def lineEditKiRep(self):
        self.horizontalSlider_ki_rep.setValue(int(float(self.lineEdit_ki_rep.text())*50))
        rospy.set_param('swarm_node/local_plan/ki_repulsive', float(self.lineEdit_ki_rep.text()))
    
    def lineEditKdRep(self):
        self.horizontalSlider_kd_rep.setValue(int(float(self.lineEdit_kd_rep.text())*50))
        rospy.set_param('swarm_node/local_plan/kd_repulsive', float(self.lineEdit_kd_rep.text()))
    
    def lineEditKpRepVel(self):
        self.horizontalSlider_kp_r_vel.setValue(int(float(self.lineEdit_kp_r_vel.text())*50))
        rospy.set_param('swarm_node/local_plan/kp_repulsive_vel', float(self.lineEdit_kp_r_vel.text()))
    
    def sliderMaxSpeed(self):
        position = self.horizontalSlider_max_speed.sliderPosition()
        rospy.set_param('swarm_node/local_plan/max_speed', (position / 20.0))
        self.lineEdit_max_speed.setText(str(rospy.get_param('swarm_node/local_plan/max_speed')))

    def sliderSensingRange(self):
        position = self.horizontalSlider_sensing_range.sliderPosition()
        rospy.set_param('swarm_node/local_plan/sensing_range', (position / 10.0))
        self.lineEdit_sensing_range.setText(str(rospy.get_param('swarm_node/local_plan/sensing_range')))

    def sliderSafetyRange(self):
        position = self.horizontalSlider_safety_range.sliderPosition()
        rospy.set_param('swarm_node/local_plan/safety_range', (position / 10.0))
        self.lineEdit_safety_range.setText(str(rospy.get_param('swarm_node/local_plan/safety_range')))

    def sliderKpAtt(self):
        position = self.horizontalSlider_kp_att.sliderPosition()
        rospy.set_param('swarm_node/local_plan/kp_attractive', (position / 50.0))
        self.lineEdit_kp_att.setText(str(rospy.get_param('swarm_node/local_plan/kp_attractive')))
    
    def sliderKpRep(self):
        position = self.horizontalSlider_kp_rep.sliderPosition()
        rospy.set_param('swarm_node/local_plan/kp_repulsive', (position / 50.0))
        self.lineEdit_kp_rep.setText(str(rospy.get_param('swarm_node/local_plan/kp_repulsive')))

    def sliderKiRep(self):
        position = self.horizontalSlider_ki_rep.sliderPosition()
        rospy.set_param('swarm_node/local_plan/ki_repulsive', (position / 50.0))
        self.lineEdit_ki_rep.setText(str(rospy.get_param('swarm_node/local_plan/ki_repulsive')))

    def sliderKdRep(self):
        position = self.horizontalSlider_kd_rep.sliderPosition()
        rospy.set_param('swarm_node/local_plan/kd_repulsive', (position / 50.0))
        self.lineEdit_kd_rep.setText(str(rospy.get_param('swarm_node/local_plan/kd_repulsive')))

    def sliderKpRepVel(self):
        position = self.horizontalSlider_kp_r_vel.sliderPosition()
        rospy.set_param('swarm_node/local_plan/kp_repulsive_vel', (position / 50.0))
        self.lineEdit_kp_r_vel.setText(str(rospy.get_param('swarm_node/local_plan/kp_repulsive_vel')))

    def stateDisplay(self):
        self.textBrowser_2.setText(self._drones[0].getState())
        self.textBrowser_3.setText(self._drones[1].getState())
        self.textBrowser_4.setText(self._drones[2].getState())

rospy.init_node("vehicle_monitor")
app = QApplication(sys.argv)
main_dialog = MainDialog()
main_dialog.show()
app.exec_()
rospy.spin()