#!/usr/bin/env python3

import tkinter as tk
from debouncer import Debouncer

import rospy
import std_msgs.msg as msg
import modern_robotics as mr

class SingleMotor(tk.Frame):
    def __init__(self, master, getSpeed, name, increaseKey, decreaseKey):
        super().__init__(master)
        self.name = name

        self.getSpeed = getSpeed

        self._velocity = 0.0

        self._pos = 0.0 

        # setting up main UI
        self.rowconfigure(0, minsize=100, weight=1)
        self.columnconfigure([0, 1, 2, 3, 4], minsize=100, weight=1)

        # name of motor label
        self.name_label = tk.Label(master=self, text=f"{self.name}")
        self.name_label.grid(row=0, column=0)

        # buttons and changing the velocity
        self.btn_decrease = tk.Button(master=self, text="-", command=self.increase)
        self.btn_decrease.bind('<Button-1>', self.decrease)
        self.btn_decrease.grid(row=0, column=1, sticky="nsew")

        self.vel_label = tk.Label(master=self, text=f"{self._velocity}")
        self.vel_label.grid(row=0, column=2)

        self.btn_increase = tk.Button(master=self, text="+", command=self.decrease) # command=self.stop for debouncer fail
        self.btn_increase.bind('<Button-1>', self.increase)
        self.btn_increase.grid(row=0, column=3, sticky="nsew")

        self.pos_label = tk.Label(master=self, text=f"{self._pos}")
        self.pos_label.grid(row=0, column=4)

        # failsafe for when debouncer fails
        # master.bind('<KeyPress-' + increaseKey + '>', self.increase)
        # master.bind('<KeyRelease-' + increaseKey + '>', self.stop)
        # master.bind('<KeyPress-' + decreaseKey + '>', self.decrease)
        # master.bind('<KeyRelease-' + decreaseKey + '>', self.stop)

        # keyboard command setup (currently fails when user mousepress on button, and mouserelease off button)
        self.up_debouncer = Debouncer(self.increase, self.decrease)
        master.bind('<KeyPress-' + increaseKey + '>', self.up_debouncer.pressed)
        master.bind('<KeyRelease-' + increaseKey + '>', self.up_debouncer.released)
        self.down_debouncer = Debouncer(self.decrease, self.increase)
        master.bind('<KeyPress-' + decreaseKey + '>', self.down_debouncer.pressed)
        master.bind('<KeyRelease-' + decreaseKey + '>', self.down_debouncer.released)

        self.pack()

        # establish publisher
        self.pub = rospy.Publisher('vel_'+self.name,msg.Float32,queue_size=1)
        self.sub = rospy.Subscriber('pos_'+self.name, msg.Float32,self.updatePos)

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, new_velocity):
        
        # if it is an equal velocity, dont bother with this stuff and clog the system
        if not mr.NearZero(self._velocity-new_velocity):
            self._velocity = new_velocity          
            self.pub.publish(self._velocity)            # publish the new velocity to this motors chatter_topic
            self.vel_label["text"] = f"{self._velocity}"            # change the label to reflect this new velocity
            rospy.loginfo(f"{self.name} is moving at {self._velocity} rad/s")           

    def keyPress(self, e):
        print(f"keypressed: {e}")

    def increase(self, e=None):
        self.velocity = self.velocity + float(self.getSpeed())

    def decrease(self, e=None):
        self.velocity = self.velocity - float(self.getSpeed())

    def updatePos(self, data):
        self.pos_label["text"] = data
