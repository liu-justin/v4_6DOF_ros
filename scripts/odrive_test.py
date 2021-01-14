#!/usr/bin/env python3

import rospy
import std_msgs.msg as m
import modern_robotics as mr
import odrive
import time
import numpy as np

class DifferentialGearUnpack():
    # order of events:
    #   subscriber starts a callback on one of the robot axes (T3, R3)
    #   callback sets its robot axis (T3, R3) to its velocity
    #   setter for robot axes velocity calls updateMotorVel
    #   updateMotorVel sets motor(A,B) vels
    #   setter for motor(A,B) velocity calls the odrv to set its velocity

    # need to change it to pos_control, vel_control doesn't work in this application (speed too low) (and doesn't hold pos)
    # need to find the smallest increment of turns to go
    # or take in the time between velocity sendings and find the position from that

    def __init__(self):
        self._vel_A = 0.0
        self._vel_B = 0.0
        self._pos_A = 0.0
        self._pos_B = 0.0

        self._vel_R3 = 0.0
        self._vel_T3 = 0.0

        self.speed_ratio = 3.95 # 20 to 89 teeth
        self.rad_to_rev = 1 /(2*np.pi)
        self.final_multipler = self.speed_ratio*self.rad_to_rev

        self.previous_time_A = 0
        self.previous_time_B = 0

        # self.minorSteps = np.pi/100
        self.minorSteps = 0.005

        self.pubR3 = rospy.Publisher('pos_motorR3',msg.Float32,queue_size=1)
        self.pubT3 = rospy.Publisher('pos_motorT3',msg.Float32,queue_size=1)

    @property
    def vel_R3(self):
        return self._vel_R3

    @vel_R3.setter
    def vel_R3(self, new_vel):
        if not mr.NearZero(self._vel_R3 - new_vel):
            self._vel_R3 = new_vel
            self.updateMotorVel()

    @property
    def vel_T3(self):
        return self._vel_T3

    @vel_T3.setter
    def vel_T3(self, new_vel):
        if not mr.NearZero(self._vel_T3 - new_vel):
            self._vel_T3 = new_vel
            self.updateMotorVel()

    @property
    def vel_A(self):
        return self._vel_A

    @vel_A.setter
    def vel_A(self, new_vel):
        if not mr.NearZero(self._vel_A - new_vel):
            self._vel_A = new_vel
            self.previous_time_A = time.perf_counter()

    @property
    def vel_B(self):
        return self._vel_B

    @vel_B.setter
    def vel_B(self, new_vel):
        if not mr.NearZero(self._vel_B - new_vel):
            self._vel_B = new_vel
            self.previous_time_B = time.perf_counter()

    def updateMotorVel(self):
        self.vel_A = self.final_multipler*(-1*self._vel_R3 + self._vel_T3)
        a = self.vel_A
        self.vel_B = self.final_multipler*(-1*self._vel_R3 + -1*self._vel_T3)
        b = self.vel_B
        rospy.loginfo(f"vel_A: {a} - vel_B: {b}")

    def updateMotorPos(self):
        pos_A = odrv0.axis0.controller.input_pos
        pos_B = odrv0.axis1.controller.input_pos
        pos_R3 = (-0.5*pos_A - 0.5*pos_B)/self.final_multipler
        self.pubR3.publish(pos_R3)
        pos_T3 = ( 0.5*pos_A - 0.5*pos_B)/self.final_multipler
        self.pubT3.publish(pos_T3)

    def callbackR3(self, data):
        self.vel_R3 = data.data
    
    def callbackT3(self, data):
        self.vel_T3 = data.data

    def step(self):
        if (self.vel_A != 0):
            current_time = time.perf_counter()
            if (current_time - self.previous_time_A) > (self.minorSteps/abs(self.vel_A+0.000001)):
                odrv0.axis0.controller.input_pos += np.sign(self.vel_A)*self.minorSteps
                updateMotorPos()
                self.previous_time_A = current_time

        if (self.vel_B != 0):
            current_time = time.perf_counter()
            if (current_time - self.previous_time_B) > (self.minorSteps/abs(self.vel_B+0.000001)):
                odrv0.axis1.controller.input_pos += np.sign(self.vel_B)*self.minorSteps
                updateMotorPos()
                self.previous_time_B = current_time

# need to add another topic in main_UI for home, and a subscriber in this file to catch it and home
# look into startup sequence in odrive

odrv0 = odrive.find_any()
print(str(odrv0.vbus_voltage))

def listener():
    a = DifferentialGearUnpack()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("vel_motorT3", m.Float32, a.callbackT3)
    rospy.Subscriber("vel_motorR3", m.Float32, a.callbackR3)
    rospy.loginfo("ready to read from chatter_motorT3/R3")
    while True:
        a.step()
        #rospy.spinOnce()

if __name__ == '__main__':
    listener()