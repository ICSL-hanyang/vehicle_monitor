#! /usr/bin/env python

import sys

from PyQt5.QtCore import *

from PyQt5.QtGui import QVector3D
from pyqtgraph.functions import glColor
import rospy, rospkg
from PyQt5.QtWidgets import *
import pyqtgraph.opengl as gl
import pyqtgraph as pg

import numpy as np
from geometry_msgs.msg import Twist, TwistStamped, Vector3

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

        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('Terrain')
        self.window.setGeometry(0, 110, 1920, 1080)
        self.window.setCameraPosition(distance=30, elevation=12)

        gx = gl.GLGridItem()
        gy = gl.GLGridItem()
        gz = gl.GLGridItem()
        gx.rotate(90, 0, 1, 0)
        gy.rotate(90, 1, 0, 0)
        gx.translate(-10, 0, 0)
        gy.translate(0, -10, 0)
        gz.translate(0, 0, -10)
        self.window.addItem(gx)
        self.window.addItem(gy)
        self.window.addItem(gz)
        ax = gl.GLAxisItem(size=QVector3D(1,1,1))
        self.window.addItem(ax)

        # vector_x = np.array([0, 0, 0])
        # vector_y = np.array([2,2,2])
        # self.vector = gl.GLLinePlotItem(pos=np.array([vector_x, vector_y]),color=glColor(255,0,0),width=3, antialias=False)
        # self.window.addItem(self.vector)

        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.updatePlot)
        self.timer.start()
        
        self.colors = []
        self.colors.append(pg.glColor(255,140,0))
        self.colors.append(pg.glColor(255,105,180))
        self.colors.append(pg.glColor(0,0,255))
        self._drones = []
        self.scatters = []
        self.atts = []
        self.reps = []
        self.v_reps = []
        self._drone_num = rospy.get_param('swarm_node/num_drone')
        pose = np.array([0,0,0])
        for i in range(self._drone_num):
            name = 'camila' + str(i+1)
            self._drones.append(Drone(name))
            self.scatters.append(gl.GLScatterPlotItem(pos=(0,0,0), size=15, color=pg.glColor(255,0,0)))
            self.atts.append(gl.GLLinePlotItem(pos=np.array([pose, pose]), color=glColor(0,255,0), width=3, antialias=False))
            self.reps.append(gl.GLLinePlotItem(pos=np.array([pose, pose]), color=glColor(255,0,0), width=3, antialias=False))
            self.v_reps.append(gl.GLLinePlotItem(pos=np.array([pose, pose]), color=glColor(255,255,0), width=3, antialias=False))
            self.window.addItem(self.scatters[i])
            self.window.addItem(self.atts[i])
            self.window.addItem(self.reps[i])
            self.window.addItem(self.v_reps[i])

    def updatePlot(self):
        # color_cycle= itertools.cycle(["orange","pink","blue","brown","red","grey","yellow","green"])
        for i in range(self._drone_num):
            self.window.removeItem(self.scatters[i])
            self.window.removeItem(self.atts[i])
            self.window.removeItem(self.reps[i])
            self.window.removeItem(self.v_reps[i])

        for i in range(self._drone_num):
            pose = self._drones[i].getPose()
            att = self._drones[i].getAtt()
            rep = self._drones[i].getRep()
            v_rep = self._drones[i].getVRep()
            pos = np.array([pose.x, pose.y, pose.z])
            att_p = np.array([att.x, att.y, att.z])
            rep_p = np.array([rep.x, rep.y, rep.z])
            v_rep_p = np.array([v_rep.x, v_rep.y, v_rep.z])
            self.scatters[i] = gl.GLScatterPlotItem(pos=pos, size=15, color=self.colors[i])
            self.atts[i] = gl.GLLinePlotItem(pos=np.array([pos, pos+att_p]), color=glColor(0,255,0), width=3, antialias=False)
            self.reps[i] = gl.GLLinePlotItem(pos=np.array([pos, pos+rep_p]), color=glColor(255,0,0), width=3, antialias=False)
            self.v_reps[i] = gl.GLLinePlotItem(pos=np.array([pos, pos+v_rep_p]), color=glColor(255,255,0), width=3, antialias=False)

            self.window.addItem(self.scatters[i])
            self.window.addItem(self.atts[i])
            self.window.addItem(self.reps[i])
            self.window.addItem(self.v_reps[i])
        self.window.show()


if __name__ == '__main__':
    rospy.init_node('vector_monitor')
    app = QApplication(sys.argv)
    monitor = VectorMonitor()
    
    sys.exit(app.exec_())