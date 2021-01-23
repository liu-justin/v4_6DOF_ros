#!/usr/bin/env python3

import rospy

import math
import numpy as np
import time
 
import tkinter as tk
from singleMotorControl import SingleMotor
from speed_selector import SpeedSelector
from home_button import HomeButton
from graph import TransfGraph
import unpack as unp


if __name__ == "__main__":
    try:
        # send msg with only int8; have 6 different topics for 6 motors

        window = tk.Tk()
        window.rowconfigure([0,1,2,3,4,5,6,7], minsize=100, weight=1)
        window.columnconfigure([0,1,2,3,4,5,6,7], minsize=100, weight=1)
        rospy.init_node('talker', anonymous=True)

        # creating all main containers
        ss = SpeedSelector(window)
        motors = []
        motors.append(SingleMotor(window, ss.getSpeed, "motorR1", 'q', 'a'))
        motors.append(SingleMotor(window, ss.getSpeed, "motorT1", 'w', 's'))
        motors.append(SingleMotor(window, ss.getSpeed, "motorT2", 'e', 'd'))
        motors.append(SingleMotor(window, ss.getSpeed, "motorR2", 'r', 'f'))
        motors.append(SingleMotor(window, ss.getSpeed, "motorT3", 't', 'g'))
        motors.append(SingleMotor(window, ss.getSpeed, "motorR3", 'y', 'h'))

        home = HomeButton(window, motors)
        transf_graph = TransfGraph(window, motors)

        # aligning all main containers
        ss.grid(row=0, column=0, columnspan=3)
        home.grid(row=0, column=3)
        transf_graph.grid(row=0, column=4, columnspan=4, rowspan=4)
        for i in range(len(motors)):
            motors[i].grid(row=i+1, column=0, columnspan=4)

        
        # creating widgets for speed selector frame and aligning widgets inside the frame, same for home and transf_graph
        ss.createWidgets()
        ss.layout()

        home.createWidgets()
        home.layout()

        transf_graph.createWidgets()

        for m in motors:
            m.createWidgets()
            m.layout()

        while not rospy.is_shutdown():
            window.mainloop()

    except rospy.ROSInterruptException:
        pass