#!/usr/bin/env python

import rospy

import math
import numpy as np
import time

import tkinter as tk
import singleMotorControl as s
import debouncer as d

SPEEDS = [1,2,4,6,8,10,20]

# send msg with only int8; have 6 different topics for 6 motors
def updateSpeed():
    speedLabel["text"]=f"speed is {speedVariable.get()} rad/s"

def getSpeed():
    return speedVariable.get()

def talker():
    
    # pub0 = rospy.Publisher('chatter_motor0',int8,queue_size=1)
    # pub1 = rospy.Publisher('chatter_motor1',int8,queue_size=1)
    # pub2 = rospy.Publisher('chatter_motor2',int8,queue_size=1)
    # pub3 = rospy.Publisher('chatter_motor3',int8,queue_size=1)
    # pub4 = rospy.Publisher('chatter_motor4',int8,queue_size=1)
    # pub5 = rospy.Publisher('chatter_motor5',int8,queue_size=1)

    window = tk.Tk()

    # establishing frame for setting the speed
    speedFrame = tk.Frame()
    speedFrame.rowconfigure(0, minsize=100, weight=1)
    speedFrame.columnconfigure([0, 1, 2], minsize=100, weight=1)
    speedLabel = tk.Label(master=speedFrame, text="speed (rad/s)")
    speedLabel.grid(row=0, column=0)

    speedVariable = tk.StringVar(speedFrame)
    speedVariable.set(SPEEDS[0])
    speedDropdown = tk.OptionMenu(speedFrame, speedVariable, *SPEEDS)
    speedDropdown.grid(row=0, column=1)

    speedConfirm = tk.Button(master=speedFrame, text="ok", command=updateSpeed)
    speedConfirm.grid(row=0,column=2)

    speedFrame.pack()

    # setting up all the motor button controls
    motorR1 = s.SingleMotor(window, getSpeed, "motorR1", 'q', 'a')
    motorT1 = s.SingleMotor(window, getSpeed, "motorT1", 'w', 's')
    motorT2 = s.SingleMotor(window, getSpeed, "motorT2", 'e', 'd')
    motorR2 = s.SingleMotor(window, getSpeed, "motorR2", 'r', 'f')
    motorT3 = s.SingleMotor(window, getSpeed, "motorT3", 't', 'g')
    motorR3 = s.SingleMotor(window, getSpeed, "motorT2", 'y', 'h')

    rospy.init_node('talker', anonmous=True)
    
    while not rospy.is_shutdown():
        window.mainloop()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass