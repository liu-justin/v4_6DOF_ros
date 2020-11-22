#!/usr/bin/env python3

import rospy
import std_msgs.msg as m
import math
import modern_robotics as mr
import odrive

class DifferentialGearUnpack():
    # order of events:
    #   subscriber starts a callback on one of the robot axes (T3, R3)
    #   callback sets its robot axis (T3, R3) to its velocity
    #   setter for robot axes velocity calls updateMotorVel
    #   updateMotorVel sets motor(A,B) vels
    #   setter for motor(A,B) velocity calls the odrv to set its velocity

    def __init__(self):
        self._vel_A = 0.0
        self._vel_B = 0.0

        self._vel_R3 = 0.0
        self._vel_T3 = 0.0

        self.speed_ratio = 3.95 # 20 to 89 teeth
        self.rad_to_rev = 2*math.pi
        self.final_multipler = self.speed_ratio*self.rad_to_rev

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
            odrv0.axis0.controller.input_vel = self.final_multipler*self._vel_A

    @property
    def vel_B(self):
        return self._vel_B

    @vel_B.setter
    def vel_B(self, new_vel):
        if not mr.NearZero(self._vel_B - new_vel):
            self._vel_B = new_vel
            odrv0.axis1.controller.input_vel = self.final_multipler*self._vel_B

    def updateMotorVel(self):
        self.vel_A = -1*self._vel_R3 + self._vel_T3
        self.vel_B = -1*self._vel_R3 + -1*self._vel_T3

    def callbackR3(self, data):
        self.vel_R3 = data.data
    
    def callbackT3(self, data):
        self.vel_T3 = data.data

# need to add another topic in main_UI for home, and a subscriber in this file to catch it and home
# look into startup sequence in odrive

odrv0 = odrive.find_any()
print(str(odrv0.vbus_voltage))

def listener():
    a = DifferentialGearUnpack()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter_motorT3", m.Float32, a.callbackT3)
    rospy.Subscriber("chatter_motorR3", m.Float32, a.callbackR3)
    rospy.loginfo("ready to read from chatter_motorT3/R3")
    rospy.spin()

if __name__ == '__main__':
    listener()