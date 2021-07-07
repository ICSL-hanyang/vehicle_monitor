#! /usr/bin/env python

import sys
from math import acos, pi, sqrt

from PyQt5.QtCore import *
from PyQt5.QtGui import QVector3D
from pyqtgraph.functions import glColor
from PyQt5.QtWidgets import *
import pyqtgraph.opengl as gl

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3
from rospy.exceptions import ROSException

class Drone:
    def __init__(self, name):
        pose_path = name + '/local_plan/global_pose'
        att_path = name + '/local_plan/attractive'
        rep_path = name + '/local_plan/repulsive'
        v_rep_path = name + '/local_plan/repulsive_vel'
        setpoint_path = name + '/mavros/setpoint_velocity/cmd_vel_unstamped'
        self._pose_sub = rospy.Subscriber(pose_path, Vector3, self.poseCB)
        self._att_sub = rospy.Subscriber(att_path, Vector3, self.attCB)
        self._rep_sub = rospy.Subscriber(rep_path, Vector3, self.repCB)
        self._v_rep_sub = rospy.Subscriber(v_rep_path, Vector3, self.vRepCB)
        self._setpoint_sub = rospy.Subscriber(setpoint_path, Twist, self.setpointCB)
        self._pose = Vector3()
        self._att = Vector3()
        self._rep = Vector3()
        self._v_rep = Vector3()
        self._setpoint = Twist()

    def poseCB(self, msg):
        self._pose = msg

    def attCB(self, msg):
        self._att = msg
    
    def repCB(self, msg):
        self._rep = msg

    def vRepCB(self, msg):
        self._v_rep = msg
    
    def setpointCB(self, msg):
        self._setpoint = msg

    def getPose(self):
        return self._pose

    def getAtt(self):
        return self._att
    
    def getRep(self):
        return self._rep

    def getVRep(self):
        return self._v_rep

    def getSetpoint(self):
        return self._setpoint


class VectorMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.scatter_size = 18
        self.arrow_length = 0.3
        self.arrow_size = 0.1
        self.color_red = glColor(255,0,0)
        self.color_green = glColor(0,255,0)
        self.color_blue = glColor(50,50,255)
        self.color_sky = glColor(0, 255, 255)
        self.color_yellow = glColor(255,255,0)
        self.color_orange = glColor(255,140,0)
        self.color_pink = glColor(255,105,180)
        self.color_brown = glColor(150,75,0)
        self.color_grey = glColor(128,128,128)

        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('Terrain')
        self.window.setGeometry(0, 110, 1920, 1080)
        self.window.setCameraPosition(distance=30, elevation=12)

        gz = gl.GLGridItem()
        gz.translate(0, 0, -0.01)
        self.window.addItem(gz)

        ax = gl.GLAxisItem(size=QVector3D(15,15,15))
        self.window.addItem(ax)

        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.updatePlot)
        self.timer.start()
        
        self.colors = []
        self.colors.append(self.color_orange)
        self.colors.append(self.color_pink)
        self.colors.append(self.color_blue)
        self.colors.append(self.color_brown)
        self.colors.append(self.color_red)
        self.colors.append(self.color_grey)
        self.colors.append(self.color_yellow)
        self.colors.append(self.color_green)
        self._drones = []
        self.scatters = []
        self.atts = []
        self.reps = []
        self.v_reps = []
        self.setpoints = []
        self.att_heads = []
        self.rep_heads = []
        self.v_rep_heads = []
        self.setpoint_heads = []
        self._drone_num = rospy.get_param('swarm_node/num_drone')
        pose = np.array([0,0,0])
        self.mesh = gl.MeshData.cylinder(rows=3, cols=10, radius=[self.arrow_size, 0], length=self.arrow_length)
        self.mesh2 = gl.MeshData.cylinder(rows=3, cols=3, radius=[0, 0], length=0)
        
        for i in range(self._drone_num):
            name = 'camila' + str(i+1)
            self._drones.append(Drone(name))
            self.scatters.append(gl.GLScatterPlotItem(pos=(0,0,0), size=self.scatter_size, color=self.color_red))
            self.atts.append(gl.GLLinePlotItem(pos=np.array([pose, pose]), color=self.color_green, width=3, antialias=False))
            self.reps.append(gl.GLLinePlotItem(pos=np.array([pose, pose]), color=self.color_red, width=3, antialias=False))
            self.v_reps.append(gl.GLLinePlotItem(pos=np.array([pose, pose]), color=self.color_yellow, width=3, antialias=False))
            self.setpoints.append(gl.GLLinePlotItem(pos=np.array([pose, pose]), color=self.color_sky, width=3, antialias=False))
            self.att_heads.append(gl.GLMeshItem(meshdata=self.mesh, smooth=True, drawEdges=False, color=self.color_green))
            self.rep_heads.append(gl.GLMeshItem(meshdata=self.mesh, smooth=True, drawEdges=False, color=self.color_red))
            self.v_rep_heads.append(gl.GLMeshItem(meshdata=self.mesh, smooth=True, drawEdges=False, color=self.color_yellow))
            self.setpoint_heads.append(gl.GLMeshItem(meshdata=self.mesh, smooth=True, drawEdges=False, color=self.color_sky))

            self.window.addItem(self.scatters[i])
            self.window.addItem(self.atts[i])
            self.window.addItem(self.reps[i])
            self.window.addItem(self.v_reps[i])
            self.window.addItem(self.setpoints[i])
            self.window.addItem(self.att_heads[i])
            self.window.addItem(self.rep_heads[i])
            self.window.addItem(self.v_rep_heads[i])
            self.window.addItem(self.setpoint_heads[i])

    def scatterDraw(self, i):
        self.window.removeItem(self.scatters[i])
        pose = self._drones[i].getPose()
        pos = np.array([pose.x, pose.y, pose.z])
        self.scatters[i] = gl.GLScatterPlotItem(pos=pos, size=self.scatter_size, color=self.colors[i%8])
        self.window.addItem(self.scatters[i])

    def drawArrow(self, var, var_head, i, color, vec):
        self.window.removeItem(var_head[i])
        pose = self._drones[i].getPose()
        pos = np.array([pose.x, pose.y, pose.z])
        vec_p = np.array([vec.x, vec.y, vec.z])
        var[i].setData(pos=np.array([pos, pos+vec_p]), color=color, width=3, antialias=False)
        if vec.x == 0 and vec.y==0 and vec.z==0:
            var_head[i] = gl.GLMeshItem(meshdata=self.mesh2, smooth=True, drawEdges=False, color=color)
        else:
            var_head[i] = gl.GLMeshItem(meshdata=self.mesh, smooth=True, drawEdges=False, color=color)
            deg, norm, tran = self.calRotate(vec)
            var_head[i].rotate(-deg, norm.x, norm.y ,norm.z)
            var_head[i].translate(pose.x+tran.x, pose.y+tran.y, pose.z+tran.z)
        self.window.addItem(var_head[i])
    
    def calRotate(self, vec):
        z_axis= Vector3(0, 0, 1)
        mag_vec = sqrt(vec.x**2 + vec.y**2 + vec.z**2)
        unit_vec = Vector3(vec.x/mag_vec, vec.y/mag_vec, vec.z/mag_vec)      
        cos = z_axis.x*unit_vec.x + z_axis.y*unit_vec.y + z_axis.z*unit_vec.z
        theta = acos(cos)
        degree = theta * 180 / pi
        normal_vec = Vector3(unit_vec.y, -unit_vec.x, 0)
        translate = Vector3(0, 0, 0)
        translate = Vector3((mag_vec - self.arrow_length)*unit_vec.x, (mag_vec - self.arrow_length)*unit_vec.y, (mag_vec - self.arrow_length)*unit_vec.z)
        return degree, normal_vec, translate      

    def updatePlot(self):
        for i in range(self._drone_num):
            setpoint = self._drones[i].getSetpoint()
            self.scatterDraw(i)
            self.drawArrow(self.atts, self.att_heads, i, self.color_green, self._drones[i].getAtt())
            self.drawArrow(self.reps, self.rep_heads, i, self.color_red, self._drones[i].getRep())
            self.drawArrow(self.setpoints, self.setpoint_heads, i, self.color_sky, Vector3(setpoint.linear.x, setpoint.linear.y, setpoint.linear.z))
            if rospy.get_param('swarm_node/local_plan/use_adaptive'):
                self.drawArrow(self.v_reps, self.v_rep_heads, i, self.color_yellow, self._drones[i].getVRep())
            
        self.window.show()

if __name__ == '__main__':
    rospy.init_node('vector_monitor')
    app = QApplication(sys.argv)
    monitor = VectorMonitor()
    
    sys.exit(app.exec_())