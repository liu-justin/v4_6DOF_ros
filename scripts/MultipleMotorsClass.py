#!/usr/bin/env python3

import tkinter as tk
from debouncer import Debouncer

import rospy
import std_msgs.msg as msg
import modern_robotics as mr
import math
from singleMotorControl import SingleMotor

class MultipleMotors():
    def __init__(self):

        self.vel_six = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.pos_six = [0.0,0.0,0.0,0.0,0.0,0.0]

        self.pub = rospy.Publisher('vel_six'+self.name,msg.Float32,queue_size=1)
        self.sub = rospy.Subscriber('pos_six'+self.name, msg.Float32,self.updateAllPos)


    def updateSingleVel(self, index, vel):
        self.vel_six[index] = vel
        pub.publish(self.vel_six) # need to figure out correct way of setting up this data

    def updateAllVel(self, new_vel_six):
        self.vel_six = new_vel_six
        pub.publish(self.vel_six)

    def updateAllPos(self, data):
        self.pos_six = data.data

