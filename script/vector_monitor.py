#! /usr/bin/env python

import rospy, rospkg
import itertools
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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


class VectorMonitor:
    def __init__(self) -> None:
        self._drones = []
        self.scatters = []
        self.atts = []
        self.reps = []
        self.v_reps = []
        self._drone_num = rospy.get_param('swarm_node/num_drone')
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-10,10])
        self.ax.set_ylim([-10,10])
        self.ax.set_zlim([0,7])
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        for i in range(self._drone_num):
            name = 'camila' + str(i+1)
            self._drones.append(Drone(name))
            self.scatters.append(self.ax.scatter(0,0,0))
            self.atts.append(self.ax.quiver(0,0,0,0,0,0))
            self.reps.append(self.ax.quiver(0,0,0,0,0,0))
            self.v_reps.append(self.ax.quiver(0,0,0,0,0,0))

        for i in range(self._drone_num):
            self.scatters[i].remove()
            self.atts[i].remove()
            self.reps[i].remove()
            self.v_reps[i].remove()
            

    def update(self):
        color_cycle= itertools.cycle(["orange","pink","blue","brown","red","grey","yellow","green"])
        for i in range(self._drone_num):
            pose = self._drones[i].getPose()
            att = self._drones[i].getAtt()
            rep = self._drones[i].getRep()
            v_rep = self._drones[i].getVRep()
            self.scatters[i] = self.ax.scatter(pose.x, pose.y, pose.z, color=next(color_cycle), s=120)
            self.atts[i] = self.ax.quiver(pose.x, pose.y, pose.z, att.x, att.y, att.z, color='green')
            self.reps[i] = self.ax.quiver(pose.x, pose.y, pose.z, rep.x, rep.y, rep.z, color='red')
            self.v_reps[i] = self.ax.quiver(pose.x, pose.y, pose.z, v_rep.x, v_rep.y, v_rep.z, color='orange')

    def draw(self):
        plt.pause(0.3)
        for i in range(self._drone_num):
            self.scatters[i].remove()
            self.atts[i].remove()
            self.reps[i].remove()
            self.v_reps[i].remove()

        # plt.show()
        

if __name__ == '__main__':
    rospy.init_node('vector_monitor')
    monitor = VectorMonitor()
    while not rospy.is_shutdown():
        monitor.update()
        monitor.draw()
    
    plt.show()