#!/usr/bin/env python

import rospy

import math
import numpy as np
import time

import tkinter as tk
from threading import Timer
import std_msgs.msg as msg

# How long a single key press lasts (as opposed to a press-and-hold).
SINGLE_PRESS_MAX_SECONDS = 0.05

class Debouncer(object):
    ''' Debounces key events for Tkinter apps, so that press-and-hold works. '''
    def __init__(self, pressed_cb, released_cb):
        self.key_pressed = False
        self.key_released_timer = None

        self.pressed_cb = pressed_cb
        self.released_cb = released_cb


    def _key_released_timer_cb(self, event):
        ''' Called when the timer expires for a key up event, signifying that a
            key press has actually ended. '''
        self.key_pressed = False
        self.released_cb(event)


    def pressed(self, event):
        ''' Callback for a key being pressed. '''
        # If timer set by up is active, cancel it, because the press is still
        # active.
        if self.key_released_timer:
            self.key_released_timer.cancel()
            self.key_released_timer = None

        # If the key is not currently pressed, mark it so and call the callback.
        if not self.key_pressed:
            self.key_pressed = True
            self.pressed_cb(event)


    def released(self, event):
        ''' Callback for a key being released. '''
        # Set a timer. If it is allowed to expire (not reset by another down
        # event), then we know the key has been released for good.
        self.key_released_timer = Timer(SINGLE_PRESS_MAX_SECONDS,
                                        self._key_released_timer_cb, [event])
        self.key_released_timer.start()

class SingleMotor(tk.Frame):
    def __init__(self, master, getSpeed, name, increaseKey, decreaseKey):
        super().__init__(master)
        self.name = name
        self.frame = tk.Frame()

        self.getSpeed = getSpeed

        # setting up main UI
        self.frame.rowconfigure(0, minsize=100, weight=1)
        self.frame.columnconfigure([0, 1, 2, 3], minsize=100, weight=1)

        # name of motor label
        self.name_label = tk.Label(master=self.frame, text=f"{self.name}")
        self.name_label.grid(row=0, column=0)

        # buttons and changing status label
        self.btn_decrease = tk.Button(master=self.frame, text="-", command=self.increase)
        self.btn_decrease.bind('<Button-1>', self.decrease)
        self.btn_decrease.grid(row=0, column=1, sticky="nsew")

        self.label = tk.Label(master=self.frame, text="0")
        self.label.grid(row=0, column=2)

        self.btn_increase = tk.Button(master=self.frame, text="+", command=self.decrease) # command=self.stop for debouncer fail
        self.btn_increase.bind('<Button-1>', self.increase)
        self.btn_increase.grid(row=0, column=3, sticky="nsew")

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

        self.frame.pack()

        # establish publisher
        self.pub = rospy.Publisher('chatter_'+self.name,msg.Int8,queue_size=1)

    def keyPress(self, e):
        print(f"keypressed: {e}")

    def increase(self, e=None):
        value = int(self.label["text"])
        new_value = value+int(self.getSpeed())
        self.label["text"] = f"{new_value}"
        self.pub.publish(new_value)
        rospy.loginfo(f"{self.name} is moving at {new_value} rad/s")

    def decrease(self, e=None):
        value = int(self.label["text"])
        new_value = value-int(self.getSpeed())
        self.label["text"] = f"{new_value}"
        self.pub.publish(new_value)
        rospy.loginfo(f"{self.name} is moving at {new_value} rad/s")

SPEEDS = [1,2,4,6,8,10,20]

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
    motorR3 = SingleMotor(window, getSpeed, "motorT2", 'y', 'h')

    rospy.init_node('talker', anonymous=True)
    
    while not rospy.is_shutdown():
        window.mainloop()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass