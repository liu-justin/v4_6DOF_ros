#!/usr/bin/env python3

import rospy

import math
import numpy as np
import time
 
import tkinter as tk
from MultipleMotorsClass import MultipleMotors
from graph import TransfGraph
import modern_robotics as mr

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
                                    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure

def trajectory_publish(angle_six_list, total_time):
    period = total_time/len(angle_six_list)
    # for i in range(1,len(angle_six_list)):
    previousTime = time.perf_counter()
    
    i = 1
    while i < len(angle_six_list):
        currentTime = time.perf_counter()
        if currentTime - previousTime > period:
            previousTime = currentTime
            angular_velocity_six = (angle_six_list[i] - angle_six_list[i-1])/period
            mm.updateAllVel(angular_velocity_six)
            i += 1
            

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
    trajectory_publish(trajectory, placeholder_t)
    # probably have a function to determine the ideal number of sample times (10 now)

def t2tPublish():
    # create function in mr to convert 6 rpy xyz values into a transf matrix 
    # calc the R with rpyToRotation, then RpToTransf
    rpyxyz_string = t2t_entry.get()
    rpyxyz = list(map(float, rpyxyz_string.split()))
    final_transf = mr.rpyxyzToTrans(rpyxyz)
    transfTrajectory = mr.CartesianTrajectory(mm.M_current, final_transf, 5,30,3)
    print(transfTrajectory)
    trajectory = []
    previous_kink = mm.pos_six
    for i in range(1,len(transfTrajectory)):
        current_kink, success = mr.IKinBody(mm.body_list, mm.M_rest, transfTrajectory[i], previous_kink, 0.01, 0.001)
        print(current_kink)
        angles = mr.JointTrajectory(previous_kink, current_kink, 1, 2,3)
        trajectory = [*trajectory, *angles]

        previous_kink = current_kink

    # print(trajectory)


def plotTransf(M):
        R,p = mr.TransToRp(M)
        x_arrow_end = p + 0.2*R[:,0]
        y_arrow_end = p + 0.2*R[:,1]
        z_arrow_end = p + 0.2*R[:,2]
        x_normal = np.c_[p,x_arrow_end]
        y_normal = np.c_[p,y_arrow_end]
        z_normal = np.c_[p,z_arrow_end]
        
        plot_x_norm.set_xdata(x_normal[0])
        plot_x_norm.set_ydata(x_normal[1])
        plot_x_norm.set_3d_properties(x_normal[2])

        plot_y_norm.set_xdata(y_normal[0])
        plot_y_norm.set_ydata(y_normal[1])
        plot_y_norm.set_3d_properties(y_normal[2])

        plot_z_norm.set_xdata(z_normal[0])
        plot_z_norm.set_ydata(z_normal[1])
        plot_z_norm.set_3d_properties(z_normal[2])

        plot_canvas.draw()

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
        plot_frame = tk.Frame()

        # align main containers
        a2a_frame.grid(row=1, column=0, columnspan=3)
        t2t_frame.grid(row=2, column=0, columnspan=3)
        plot_frame.grid(row=0,column=3, columnspan=3)

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

        # create widgets for plot_frame
        plot_fig = Figure(figsize=(3,3), dpi=100)
        plot_canvas = FigureCanvasTkAgg(plot_fig, master=plot_frame)
        plot_canvas.draw()
        plot_ax = plot_fig.add_subplot(111,projection="3d")
        plot_ax.axes.set_xlim3d(left=-2, right=2)
        plot_ax.axes.set_ylim3d(bottom=-2, top=2)
        plot_ax.axes.set_zlim3d(bottom=-1, top=1)
        plot_x_norm, = plot_ax.plot([0,0],[0,0],[0,0])
        plot_y_norm, = plot_ax.plot([0,0],[0,0],[0,0])
        plot_z_norm, = plot_ax.plot([0,0],[0,0],[0,0])
        print(mm.M_current)
        plotTransf(mm.M_current)
        plot_canvas.get_tk_widget().pack()

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