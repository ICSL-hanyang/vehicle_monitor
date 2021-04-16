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
        self.lineEdit_rep_range.setText(str(rospy.get_param('swarm_node/local_plan/repulsive_range')))
        self.lineEdit_rep_range.textChanged.connect(self.lineEditRepRange)
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
        self.horizontalSlider_rep_range.setRange(0, 100)
        self.horizontalSlider_rep_range.setSingleStep(1)
        self.horizontalSlider_rep_range.setValue(int(rospy.get_param('swarm_node/local_plan/repulsive_range')*10))
        self.horizontalSlider_rep_range.valueChanged.connect(self.sliderRange)
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

        self._drone1_att_sub = rospy.Subscriber("camila1/local_plan/attractive", Vector3, self.d1attSub)
        self._drone1_rep_sub = rospy.Subscriber("camila1/local_plan/repulsive", Vector3, self.d1repSub)
        self._drone1_r_vel_sub = rospy.Subscriber("camila1/local_plan/repulsive_vel", Vector3, self.d1rVelSub)
        self._drone1_set_vel_sub = rospy.Subscriber("camila1/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, self.d1SetVelSub)
        self._drone2_att_sub = rospy.Subscriber("camila2/local_plan/attractive", Vector3, self.d2attSub)
        self._drone2_rep_sub = rospy.Subscriber("camila2/local_plan/repulsive", Vector3, self.d2repSub)
        self._drone2_r_vel_sub = rospy.Subscriber("camila2/local_plan/repulsive_vel", Vector3, self.d2rVelSub)
        self._drone2_set_vel_sub = rospy.Subscriber("camila2/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, self.d2SetVelSub)
        self._drone3_att_sub = rospy.Subscriber("camila3/local_plan/attractive", Vector3, self.d3attSub)
        self._drone3_rep_sub = rospy.Subscriber("camila3/local_plan/repulsive", Vector3, self.d3repSub)
        self._drone3_r_vel_sub = rospy.Subscriber("camila3/local_plan/repulsive_vel", Vector3, self.d3rVelSub)    
        self._drone3_set_vel_sub = rospy.Subscriber("camila3/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, self.d3SetVelSub)
        self._drone1_att = Vector3()
        self._drone1_rep = Vector3()
        self._drone1_r_vel = Vector3()
        self._drone1_set_vel = Twist()
        self._drone2_att = Vector3()
        self._drone2_rep = Vector3()
        self._drone2_r_vel = Vector3()
        self._drone2_set_vel = Twist()
        self._drone3_att = Vector3()
        self._drone3_rep = Vector3()
        self._drone3_r_vel = Vector3()
        self._drone3_set_vel = Twist()
        self.arming_pub = rospy.Publisher('multi/arming', Bool, queue_size=10)
        self.set_mode_pub = rospy.Publisher('multi/set_mode', String, queue_size=10)


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

    def lineEditRepRange(self):
        self.horizontalSlider_rep_range.setValue(int(float(self.lineEdit_rep_range.text())*10))
        rospy.set_param('swarm_node/local_plan/repulsive_range', float(self.lineEdit_rep_range.text()))
    
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

    def sliderRange(self):
        position = self.horizontalSlider_rep_range.sliderPosition()
        rospy.set_param('swarm_node/local_plan/repulsive_range', (position / 10.0))
        self.lineEdit_rep_range.setText(str(rospy.get_param('swarm_node/local_plan/repulsive_range')))

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
        d1att = 'D1 att : ' + str(round(self._drone1_att.x, 2)) + ', ' + str(round(self._drone1_att.y, 2)) + ', ' + str(round(self._drone1_att.z, 2))
        d1rep = 'D1 rep : ' + str(round(self._drone1_rep.x, 2)) + ', ' + str(round(self._drone1_rep.y, 2)) + ', ' + str(round(self._drone1_rep.z, 2))
        d1r_vel = 'D1 r vel : ' + str(round(self._drone1_r_vel.x, 2)) + ', ' + str(round(self._drone1_r_vel.y, 2)) + ', ' + str(round(self._drone1_r_vel.z, 2))
        d1set_vel = 'D1 set vel : ' + str(round(self._drone1_set_vel.linear.x, 2)) + ', ' + str(round(self._drone1_set_vel.linear.y, 2)) + ', ' + str(round(self._drone1_set_vel.linear.z, 2))
        self.textBrowser_2.setText(d1att + '\n' + d1rep + '\n' + d1r_vel + '\n' + d1set_vel)
        d2att = 'D2 att : ' + str(round(self._drone2_att.x, 2)) + ', ' + str(round(self._drone2_att.y, 2)) + ', ' + str(round(self._drone2_att.z, 2))
        d2rep = 'D2 rep : ' + str(round(self._drone2_rep.x, 2)) + ', ' + str(round(self._drone2_rep.y, 2)) + ', ' + str(round(self._drone2_rep.z, 2))
        d2r_vel = 'D2 r vel : ' + str(round(self._drone2_r_vel.x, 2)) + ', ' + str(round(self._drone2_r_vel.y, 2)) + ', ' + str(round(self._drone2_r_vel.z, 2))
        d2set_vel = 'D2 set vel : ' + str(round(self._drone2_set_vel.linear.x, 2)) + ', ' + str(round(self._drone2_set_vel.linear.y, 2)) + ', ' + str(round(self._drone2_set_vel.linear.z, 2))
        self.textBrowser_3.setText(d2att + '\n' + d2rep + '\n' + d2r_vel + '\n' + d2set_vel)
        d3att = 'D3 att : ' + str(round(self._drone3_att.x, 2)) + ', ' + str(round(self._drone3_att.y, 2)) + ', ' + str(round(self._drone3_att.z, 2))
        d3rep = 'D3 rep : ' + str(round(self._drone3_rep.x, 2)) + ', ' + str(round(self._drone3_rep.y, 2)) + ', ' + str(round(self._drone3_rep.z, 2))
        d3r_vel = 'D3 r vel : ' + str(round(self._drone3_r_vel.x, 2)) + ', ' + str(round(self._drone3_r_vel.y, 2)) + ', ' + str(round(self._drone3_r_vel.z, 2))
        d3set_vel = 'D3 set vel : ' + str(round(self._drone3_set_vel.linear.x, 2)) + ', ' + str(round(self._drone3_set_vel.linear.y, 2)) + ', ' + str(round(self._drone3_set_vel.linear.z, 2))
        self.textBrowser_4.setText(d3att + '\n' + d3rep + '\n' + d3r_vel + '\n' + d3set_vel)

    def d1attSub(self, msg):
        self._drone1_att = msg

    def d1repSub(self, msg):
        self._drone1_rep = msg
    
    def d1rVelSub(self, msg):
        self._drone1_r_vel = msg

    def d1SetVelSub(self, msg):
        self._drone1_set_vel = msg

    def d2attSub(self, msg):
        self._drone2_att = msg

    def d2repSub(self, msg):
        self._drone2_rep = msg
    
    def d2rVelSub(self, msg):
        self._drone2_r_vel = msg

    def d2SetVelSub(self, msg):
        self._drone2_set_vel = msg

    def d3attSub(self, msg):
        self._drone3_att = msg

    def d3repSub(self, msg):
        self._drone3_rep = msg
    
    def d3rVelSub(self, msg):
        self._drone3_r_vel = msg
    
    def d3SetVelSub(self, msg):
        self._drone3_set_vel = msg


rospy.init_node("vehicle_monitor")
app = QApplication(sys.argv)
main_dialog = MainDialog()
main_dialog.show()
app.exec_()
rospy.spin()