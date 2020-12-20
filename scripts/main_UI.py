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

def test():
    print("testing")

def talker():
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


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass