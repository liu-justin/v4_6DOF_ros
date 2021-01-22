#!/usr/bin/env python3

import rospy

import math
import numpy as np
import time
 
import tkinter as tk
from singleMotorControl import SingleMotor
from speed_selector import SpeedSelector
from home_button import HomeButton
import unpack as unp


if __name__ == "__main__":
    try:
        T_ee, T_list, body_list, G_list = unp.unpack_XML("scripts/6DoF_URDF.xml")
        theta_home = np.array([0,-1*np.pi/2, np.pi/2,0,0,0])
        M_home = mr.FKinBody(T_ee, body_list, theta_home)

        # send msg with only int8; have 6 different topics for 6 motors

        window = tk.Tk()
        rospy.init_node('talker', anonymous=True)

        ss = SpeedSelector(window)
        
        #setting up all the motor button controls
        motors = []
        motors.append(SingleMotor(window, ss.getSpeed, "motorR1", 'q', 'a'))
        motors.append(SingleMotor(window, ss.getSpeed, "motorT1", 'w', 's'))
        motors.append(SingleMotor(window, ss.getSpeed, "motorT2", 'e', 'd'))
        motors.append(SingleMotor(window, ss.getSpeed, "motorR2", 'r', 'f'))
        motors.append(SingleMotor(window, ss.getSpeed, "motorT3", 't', 'g'))
        motors.append(SingleMotor(window, ss.getSpeed, "motorR3", 'y', 'h'))

        homeButton = HomeButton(window, motors)
        
        while not rospy.is_shutdown():
            window.mainloop()

    except rospy.ROSInterruptException:
        pass