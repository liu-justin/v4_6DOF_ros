#!/usr/bin/env python3

import rospy

import math
import numpy as np
import time
 
import tkinter as tk
from MultipleMotorsClass import MultipleMotors
from graph import TransfGraph
import modern_robotics as mr
import v4_6dof.msg as msg

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
                                    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure

def trajectory_publish(angle_six_list, time_gap):
    # for i in range(1,len(angle_six_list)):
    previousTime = time.perf_counter()
    mm.updateVelGap(angle_six_list[1]-angle_six_list[0], int(1000000*time_gap))
    mm.updateVelGap(angle_six_list[2]-angle_six_list[1], int(1000000*time_gap))
    i = 3
    while i < len(angle_six_list):
        currentTime = time.perf_counter()
        if currentTime - previousTime > time_gap:
            previousTime = currentTime
            angular_velocity_six = (angle_six_list[i] - angle_six_list[i-1])/time_gap
            mm.updateVelGap(angular_velocity_six, int(1000000*time_gap))
            i += 1
    mm.updateVelGap([0,0,0,0,0,0], int(1000000*time_gap))
            

def a2aAbsolutePublish():
    # get current angle_list from mm and do a mr.JointTrajectory like in home
    # probably get time from entry
    angle_list = a2a_entry.get()
    final_angles = list(map(float, angle_list.split()))
    placeholder_t = 5
    points_per_sec = 10
    total_points = placeholder_t*points_per_sec
    trajectory = mr.JointTrajectory(mm.pos_six, final_angles, placeholder_t, total_points, 3)
    gap_in_micros = placeholder_t/(total_points-1)
    trajectory_publish(trajectory, gap_in_micros)
    mm.updatePos(final_angles)
    print(mm.M_current)

def a2aRelativePublish():
    # get current angle_list from mm and do a mr.JointTrajectory like in home
    # probably get time from entry
    angle_list = a2ar_entry.get()
    final_angles = list(map(float, angle_list.split()))
    placeholder_t = 5
    points_per_sec = 10
    total_points = placeholder_t*points_per_sec
    trajectory = mr.JointTrajectory([0,0,0,0,0,0], final_angles, placeholder_t, total_points, 3)
    gap_in_micros = placeholder_t/(total_points-1)
    trajectory_publish(trajectory, gap_in_micros)
    mm.addPos(final_angles)
    print(mm.M_current)
    
    # probably have a function to determine the ideal number of sample times (10 now)

def t2tPublish():
    # create function in mr to convert 6 rpy xyz values into a transf matrix 
    # calc the R with rpyToRotation, then RpToTransf
    rpyxyz_string = t2t_entry.get()
    rpyxyz = list(map(float, rpyxyz_string.split()))
    final_transf = mr.rpyxyzToTrans(rpyxyz)
    placeholder_t = 5
    points_per_sec = 10
    total_points = placeholder_t*points_per_sec
    transfTrajectory = mr.CartesianTrajectory(mm.M_current, final_transf, placeholder_t,total_points,3)
    trajectory = []
    previous_kink = mm.pos_six
    for i in range(1,len(transfTrajectory)):
        current_kink, success = mr.IKinBody(mm.body_list, mm.M_rest, transfTrajectory[i], previous_kink, 0.01, 0.001)
        # angles = mr.JointTrajectory(previous_kink, current_kink, 1, 2,3)
        # trajectory = [*trajectory, *angles] # need to remove the first one to prevent dups, and add the very first one at the end
        trajectory.append(current_kink)
        previous_kink = current_kink
    gap_in_micros = placeholder_t/(total_points-1)
    trajectory_publish(trajectory, gap_in_micros)
    print(trajectory[-1])
    mm.updatePos(trajectory[-1])

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
        window.rowconfigure([0,1,2,3,4], minsize=100, weight=1)
        window.columnconfigure([0,1,2,3,4,5,6], minsize=100, weight=1)
        rospy.init_node('talker', anonymous=True)

        # create main containers
        a2a_frame = tk.Frame()
        a2ar_frame = tk.Frame()
        t2t_frame = tk.Frame()
        display_frame = tk.Frame()
        plot_frame = tk.Frame()

        # align main containers
        a2a_frame.grid(row=0, column=0, columnspan=3)
        a2ar_frame.grid(row=1, column=0, columnspan=3)
        t2t_frame.grid(row=2, column=0, columnspan=3)
        display_frame.grid(row=3,column=0, rowspan=2, columnspan=3)
        plot_frame.grid(row=0,column=4,rowspan=3, columnspan=3)

        # create widgets for a2a frame
        a2a_label = tk.Label(master=a2a_frame, text="Angle to angle Absolute")
        a2a_entry = tk.Entry(master=a2a_frame)
        a2a_button = tk.Button(master=a2a_frame, text="Confirm", command = a2aAbsolutePublish)

        a2a_frame.rowconfigure(0,minsize=50, weight=1)
        a2a_frame.columnconfigure([0,1,2], minsize=100, weight=1)
        a2a_label.grid(row=0, column=0)
        a2a_entry.grid(row=0, column=1)
        a2a_button.grid(row=0, column=2)

        # create widgets for a2ar frame
        a2ar_label = tk.Label(master=a2ar_frame, text="Angle to angle Relative")
        a2ar_entry = tk.Entry(master=a2ar_frame)
        a2ar_button = tk.Button(master=a2ar_frame, text="Confirm", command = a2aRelativePublish)

        a2ar_frame.rowconfigure(0,minsize=50, weight=1)
        a2ar_frame.columnconfigure([0,1,2], minsize=100, weight=1)
        a2ar_label.grid(row=0, column=0)
        a2ar_entry.grid(row=0, column=1)
        a2ar_button.grid(row=0, column=2)

        # create widgets for t2t frame
        t2t_label = tk.Label(master=t2t_frame, text="Enter roll pitch yaw x y z")
        t2t_entry = tk.Entry(master=t2t_frame)
        t2t_button = tk.Button(master=t2t_frame, text="Confirm", command = t2tPublish)

        t2t_frame.rowconfigure(0,minsize=50, weight=1)
        t2t_frame.columnconfigure([0,1,2], minsize=100, weight=1)
        t2t_label.grid(row=0, column=0)
        t2t_entry.grid(row=0, column=1)
        t2t_button.grid(row=0, column=2)

        #create widgets for display_frame
        display_pos_label = tk.Label(master=display_frame, text="pos")
        display_vel_label = tk.Label(master=display_frame, text="vel")
        display_pos_value = []
        display_vel_value = []
        for i in range(0,6):
            display_pos_value.append(tk.Label(master=display_frame, text=f"{round(mm.pos_six[i],3)}"))
            display_vel_value.append(tk.Label(master=display_frame, text=f"{round(mm.vel_six[i],3)}"))

        display_frame.rowconfigure([0,1],minsize=50, weight=1)
        display_frame.columnconfigure([0,1,2,3,4,5,6], minsize=50, weight=1)
        display_pos_label.grid(row=0, column=0)
        display_vel_label.grid(row=1, column=0)
        for i in range(0,6):
            display_pos_value[i].grid(row=0, column=i+1)
            display_vel_value[i].grid(row=1, column=i+1)

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