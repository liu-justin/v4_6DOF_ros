#!/usr/bin/env python3

import tkinter as tk
from debouncer import Debouncer

import rospy
import v4_6dof.msg as msg
import modern_robotics as mr
from singleMotorControl import SingleMotor
import unpack as unp

class MultipleMotors():
    def __init__(self):

        self.vel_six = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.pos_six = [0.0,0.0,0.0,0.0,0.0,0.0]

        self.M_rest, self.T_list, self.body_list, self.G_list = unp.unpack_XML("/home/brigs/catkin_ws/src/v4_6dof/scripts/6DoF_URDF.xml")

        self.M_current = mr.FKinBody(self.M_rest, self.body_list, self.pos_six)

        self.pub = rospy.Publisher('vel_six_chatter',msg.Float32List,queue_size=1)
        self.sub = rospy.Subscriber('pos_six_chatter',msg.Float32List,self.updateAllPos)


    def updateSingleVel(self, index, vel):
        self.vel_six[index] = vel
        self.pub.publish(self.vel_six)
         # need to figure out correct way of setting up this data

    def publishArray(self):
        arg = msg.Float32List()
        arg.data = self.vel_six
        self.pub.publish(arg)

    def updateAllVel(self, new_vel_six):
        self.vel_six = new_vel_six
        self.pub.publish(self.vel_six)

    def updateAllPos(self, data):
        rospy.loginfo(f"got this data: {data}")
        self.pos_six = data.data

