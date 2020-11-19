#!/usr/bin/env python3

import rospy

import math
import numpy as np
import time
 
import tkinter as tk
from singleMotorControl import SingleMotor

SPEEDS = [1,2,5,10,20]

def talker():
    # send msg with only int8; have 6 different topics for 6 motors

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

    def updateSpeed():
        speedLabel["text"]=f"speed is {speedVariable.get()} rad/s"

    speedConfirm = tk.Button(master=speedFrame, text="ok", command=updateSpeed)
    speedConfirm.grid(row=0,column=2)

    speedFrame.pack()

    def getSpeed():
        return speedVariable.get()

    #setting up all the motor button controls
    motorR1 = SingleMotor(window, getSpeed, "motorR1", 'q', 'a')
    motorT1 = SingleMotor(window, getSpeed, "motorT1", 'w', 's')
    motorT2 = SingleMotor(window, getSpeed, "motorT2", 'e', 'd')
    motorR2 = SingleMotor(window, getSpeed, "motorR2", 'r', 'f')
    motorT3 = SingleMotor(window, getSpeed, "motorT3", 't', 'g')
    motorR3 = SingleMotor(window, getSpeed, "motorR3", 'y', 'h')

    rospy.init_node('talker', anonymous=True)
    
    while not rospy.is_shutdown():
        window.mainloop()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass