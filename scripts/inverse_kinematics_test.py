import rospy
import numpy as np
from modules.MotorController import MotorController
from modules import modern_robotics as mr

# import v4_6dof.msg as msg

mc = MotorController()
rospy.init_node('talker', anonymous=True)
print("starting")

at_rest_str = input("is arm at rest position y/n?")
at_rest = True if at_rest_str=="y" else False
# # if at rest, go to cobra position
# if at_rest: mc.anglePublish([0, -7*np.pi/12, 11*np.pi/12, 0, -3*np.pi/12, 0], 3, True)
if at_rest: mc.anglePublish([0, -2*np.pi/3, 2*np.pi/3, 0, -7*np.pi/12, 0], 3, True)

print(f"this is the transf matrix right now: {mc.M_current}")
while True:
    xyz_str = input("input xyz values: ")
    xyz = [float(xyz) for xyz in xyz_str.split(" ")]
    rpy = [0,0,0]
    rpyxyz = rpy+xyz
    transf = mr.rpyxyzToTrans(rpyxyz)
    time_str = input("input the time: ")
    time = float(time_str)
    
    # mc.transfMatrixCartesianPublish(transf, time)
    mc.transfMatrixJointPublish(transf, time)

#09-08-2021 max speed is 0.8rad/sec for all motors