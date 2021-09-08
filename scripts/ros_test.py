import rospy
import numpy as np
from modules.MotorController import MotorController

# import v4_6dof.msg as msg

mc = MotorController()
rospy.init_node('talker', anonymous=True)
print("starting")

at_rest_str = input("is arm at rest position y/n?")
at_rest = True if at_rest_str=="y" else False
# # if at rest, go to cobra position
if at_rest: mc.anglePublish([0, -np.pi/2, np.pi/2, 0, -np.pi/2, 0], 4, True)

# [float(x) for x in a])
while True:
    angles_str = input("input 6 angles: ")
    angles = [float(x) for x in (angles_str.split(" "))]
    print(angles)
    time_str = input("input the time: ")
    time = float(time_str)
    mc.anglePublish(angles, time, False)

#09-08-2021 max speed is 0.8rad/sec for all motors