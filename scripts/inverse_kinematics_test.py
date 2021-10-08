import rospy
import numpy as np
from modules.MotorController import MotorController
from modules import modern_robotics as mr

# import v4_6dof.msg as msg

mc = MotorController()
these_angles = [0*np.pi/2, -1*np.pi/2, 1*np.pi/2, 0*np.pi/2, -0*np.pi/2, 0*np.pi/2]
# these_angles = [0,0,0,0,0,np.pi/2]
bodyM = mr.FKinBody(mc.M_rest, mc.body_list, these_angles)
spaceM = mr.FKinSpace(mc.M_rest, mc.space_list, these_angles)
print(spaceM)
print(bodyM)

# rospy.init_node('talker', anonymous=True)

# at_rest_str = input("is arm at rest position y/n?")
# at_rest = True if at_rest_str=="y" else False
# # # if at rest, go to cobra position
# if at_rest: mc.anglePublish([0, -1*np.pi/2, 1*np.pi/2, 0*np.pi/2, -1*np.pi/2, 1*np.pi/2], 0.3, True)
# # if at_rest: mc.anglePublish([0, -2*np.pi/3, 2*np.pi/3, 0, -1*np.pi/2, 0], 0.3, True)
# print(f"this is the anglelist before: {mc.pos_six}")
# #2.09

# print(f"this is the transf matrix right now: \n{mc.M_current}")
# while True:
#     xyz_str = input("input xyz values: ")
#     xyz = [float(xyz) for xyz in xyz_str.split(" ")]
#     rpy = [0,0,0]
#     rpyxyz = rpy+xyz
#     transf = mr.rpyxyzToTrans(rpyxyz)
#     time_str = input("input the time: ")
#     time = float(time_str)
    
#     # mc.transfMatrixCartesianPublish(transf, time)
#     # mc.transfMatrixJointPublish(transf, time)
#     mc.transfMatrixAnalyticalPublish(transf, time)

# #09-08-2021 max speed is 0.8rad/sec for all motors