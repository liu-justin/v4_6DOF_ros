#!/usr/bin/env python

import rospy

import std_msgs.msg as import int8
import math
import numpy as np
import time

# send msg with only int8; have 6 different topics for 6 motors

def server():
    
    pub0 = rospy.Publisher('chatter_motor0',int8,queue_size=1)
    pub1 = rospy.Publisher('chatter_motor1',int8,queue_size=1)
    pub2 = rospy.Publisher('chatter_motor2',int8,queue_size=1)
    pub3 = rospy.Publisher('chatter_motor3',int8,queue_size=1)
    pub4 = rospy.Publisher('chatter_motor4',int8,queue_size=1)
    pub5 = rospy.Publisher('chatter_motor5',int8,queue_size=1)

    rospy.init_node('talker', anonmous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        

    value = int(lbl_value["text"])
    lbl_value["text"] = f"{value - 1}"

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass