#!/usr/bin/env python3

import rospy

import math
import numpy as np
import time
 
import tkinter as tk
from MultipleMotorsClass import MultipleMotors
from speed_selector import SpeedSelector
from home_button import HomeButton
from graph import TransfGraph
import unpack as unp
from functools import partial
import modern_robotics as mr

def a2aPublish():
    # get current angle_list from mm and do a mr.JointTrajectory like in home
    # probably get time from entry
    angle_list = a2a_entry.get()
    placeholder_t = 5
    final_angles = list(map(float, angle_list.split()))
    print(mm.pos_six)
    print(angle_list)
    trajectory = mr.JointTrajectory(mm.pos_six, final_angles, placeholder_t, 10, 3)
    print(trajectory)
    # probably have a function to determine the ideal number of sample times (10 now)

def t2tPublish():
    # create function in mr to convert 6 rpy xyz values into a transf matrix 
    # calc the R with rpyToRotation, then RpToTransf
    rpyxyz_string = t2t_entry.get()
    rpyxyz = list(map(float, rpyxyz_string.split()))
    print(rpyxyz)
    final_transf = mr.rpyxyzToTrans(rpyxyz)
    trajectory = mr.CartesianTrajectory(mm.M_current, final_transf, 5,10,3)
    print(trajectory)

if __name__ == "__main__":
    try:
        mm = MultipleMotors()

        window = tk.Tk()
        window.rowconfigure([0,1,2,3], minsize=100, weight=1)
        window.columnconfigure([0,1,2,3,4,5], minsize=100, weight=1)
        rospy.init_node('talker', anonymous=True)

        # create main containers
        a2a_frame = tk.Frame()
        t2t_frame = tk.Frame()

        # align main containers
        a2a_frame.grid(row=1, column=0, columnspan=3)
        t2t_frame.grid(row=2, column=0, columnspan=3)

        # create widgets for a2a frame
        a2a_label = tk.Label(master=a2a_frame, text="Angle to angle")
        a2a_entry = tk.Entry(master=a2a_frame)
        a2a_button = tk.Button(master=a2a_frame, text="Confirm", command = a2aPublish)

        a2a_frame.rowconfigure(0,minsize=100, weight=1)
        a2a_frame.columnconfigure([0,1,2], minsize=100, weight=1)
        a2a_label.grid(row=0, column=0)
        a2a_entry.grid(row=0, column=1)
        a2a_button.grid(row=0, column=2)

        # create widgets for t2t frame
        t2t_label = tk.Label(master=t2t_frame, text="Enter roll pitch yaw x y z")
        t2t_entry = tk.Entry(master=t2t_frame)
        t2t_button = tk.Button(master=t2t_frame, text="Confirm", command = t2tPublish)

        t2t_frame.rowconfigure(0,minsize=100, weight=1)
        t2t_frame.columnconfigure([0,1,2], minsize=100, weight=1)
        t2t_label.grid(row=0, column=0)
        t2t_entry.grid(row=0, column=1)
        t2t_button.grid(row=0, column=2)        


        # need a tkinter GUI, shows the current transf
        # need an input to the next transf, which will:
            # first find the P2P in cartesian
            # then set up the timer and and publish velocities in time (the bigger the period, the more jagged the pos graph looks)
                # not the same as waiting for the next step like with v1
                # in v4, the pos wasn't updating when homing --> see if we can fix that
            # with individual control (which is not needed for my use case), need to think of something else

        while not rospy.is_shutdown():
            window.mainloop()

    except rospy.ROSInterruptException:
        pass